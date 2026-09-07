# mcl 代码架构设计

| 项 | 值 |
|---|---|
| 文档名称 | mcl 代码架构设计 |
| 版本 | v0.1.0（草案） |
| 日期 | 2026-09-05 |
| 前置 | `docs/spec/mcl_spec.md`（功能基线）、`docs/规范/architecture.md`（分层/依赖规则）、`docs/spec/mcl_fixed_point.md`（定点说明） |
| 参考 | VESC 固件（`reference/bldc/`）电机控制架构 |

---

## 1. 设计目标

把 mcl 设计为一个 **自包含、可复用的电机控制库**，核心原则：

1. **平台无关（ARM 通用）**：核心算法零寄存器依赖，硬件全部走注入的 HAL 接口。
2. **运行时注入 + 多实例**：不用 VESC 的宏抽象（编译期单例），改用函数指针 + `void *ctx`，同一份库可实例化多个电机。
3. **纯算法与编排分离**：借鉴 VESC「`foc_math`（纯算法）↔ `mcpwm_foc`（硬件编排）」的分离，mcl 内算法模块零硬件依赖，编排逻辑集中在门面。
4. **可测试**：纯算法模块可脱离硬件单测；门面可接 mock HAL 做 SIL 仿真。
5. **符合工程规范**：遵循 `coding_standard.md`（命名/风格/无 `_t`/self 指针）与 `architecture.md`（DAG、依赖注入、回调带 ctx）。

---

## 2. 总体架构（三层 + 一组接口）

```text
┌───────────────────────────────────────────────────────────────┐
│  门面层  mcl（mcl.c / mcl.h）                                   │
│  · 状态机（IDLE/ALIGN/RUN/FAULT）、模式切换、指令下发、遥测      │
│  · 环级联调度（位置→速度→电流）、控制周期编排                    │
└──────────────┬────────────────────────────────────────────────┘
               │ 编排调用（纯算法，无硬件）
┌──────────────▼────────────────────────────────────────────────┐
│  算法编排层  mcl_foc / mcl_bldc_comm                            │
│  · 把 transform/pid/svpwm/observer/pll 串成一次完整控制周期     │
└──────────────┬────────────────────────────────────────────────┘
               │ 纯算法调用
┌──────────────▼────────────────────────────────────────────────┐
│  纯算法层（平台无关，可独立单测）                                │
│  mcl_math（可插拔数学） mcl_transform  mcl_pid  mcl_svpwm       │
│  mcl_observer  mcl_pll  mcl_mtpa_fw                             │
│  横切：mcl_protection（保护）  mcl_calibration（校准）           │
└──────────────┬────────────────────────────────────────────────┘
               │ 访问硬件（运行时注入）
┌──────────────▼────────────────────────────────────────────────┐
│  HAL 抽象  mcl_hal.h（mcl_hal_ops + void *ctx，宿主实现）        │
│  pwm_set_duty / adc_read_phase / enc_read_angle / micros / …   │
└───────────────────────────────────────────────────────────────┘
```

**与 VESC 的映射**：

| VESC | mcl | 说明 |
|---|---|---|
| `mc_interface` | `mcl` 门面 | 状态机 + 指令 + 遥测（去掉通信/存储/统计） |
| `mcpwm_foc` / `mcpwm` | `mcl_foc` / `mcl_bldc_comm` | 算法编排，但硬件经 HAL 注入 |
| `foc_math` | transform/pid/svpwm/observer/pll/mtpa_fw | 纯算法，零硬件依赖 |
| `hw.h` 宏抽象 | `mcl_hal.h` 接口注入 | 宏→函数指针+ctx（多实例） |
| `mc_configuration` | `mcl_config` | 集中配置（去掉通信/存储字段） |
| `encoder/` 门面 | 由宿主实现（库只定义 `enc_read_*` 接口） | 库不内置具体编码器驱动 |

---

## 3. 目录结构

```text
include/
├── mcl.h               库总头（聚合所有公开头 + 门面 API）
├── mcl_types.h         版本宏、枚举、集中配置 mcl_config、对象类型
├── mcl_math.h          数学库抽象（可插拔）
├── mcl_hal.h           HAL 抽象接口 mcl_hal_ops
├── mcl_transform.h     Clark / Park / 反 Park
├── mcl_pid.h           通用 PID
├── mcl_svpwm.h         SVPWM 调制
├── mcl_observer.h      观测器载体（可插拔 ops 接口）
├── mcl_observer_flux.h 磁链观测器参考实现（插槽模板）
├── mcl_observer_smo.h  滑模观测器（反电动势 + 相位补偿）
├── mcl_observer_ortega.h ORTEGA 型磁链观测器（相位收敛）
├── mcl_pll.h           PLL 锁相环（相位跟踪 + 速度估计）
├── mcl_foc.h           FOC 电流环编排
├── mcl_mtpa_fw.h       MTPA + 弱磁
├── mcl_bldc_comm.h     六步换相编排
├── mcl_protection.h    保护
└── mcl_calibration.h   校准

src/                         实现（.c，与 include 一一对应）
```

include 约定：根 `include/` 进 `-I`，引用写 `#include "mcl.h"`（裸文件名，架构规范 §7）。

---

## 4. 模块职责与依赖 DAG

### 4.1 模块职责

| 模块 | 公开前缀 | 职责 | 平台无关 |
|---|---|---|---|
| `mcl` | `mcl_` | 门面：状态机、模式、指令、遥测、环级联、控制周期 | ✅ |
| `mcl_math` | `mcl_math_` | 数学库抽象：可插拔实现（查表/libm/CMSIS/芯片 API） | ✅ |
| `mcl_foc` | `mcl_foc_` | FOC 电流环编排：Park→PI→解耦/弱磁→反Park→SVPWM | ✅ |
| `mcl_bldc_comm` | `mcl_bldc_comm_` | 六步换相编排（hall / 无感 BEMF） | ✅ |
| `mcl_transform` | `mcl_transform_` | Clark / Park / 反 Park | ✅ |
| `mcl_pid` | `mcl_pid_` | 通用 PID（限幅 + 抗积分饱和） | ✅ |
| `mcl_svpwm` | `mcl_svpwm_` | SVPWM（含过调制），输出三相占空比 | ✅ |
| `mcl_observer` | `mcl_observer_` | 观测器载体：统一 ops 接口，可插拔多种算法 | 接口 |
| `mcl_observer_flux` | `mcl_observer_flux_` | 磁链观测器参考实现（插槽模板） | ✅ |
| `mcl_observer_smo` | `mcl_observer_smo_` | 滑模观测器（反电动势低通 + 相位补偿） | ✅ |
| `mcl_observer_ortega` | `mcl_observer_ortega_` | ORTEGA 型磁链观测器（相位收敛） | ✅ |
| `mcl_pll` | `mcl_pll_` | PLL 锁相环，相位跟踪 + 速度估计 | ✅ |
| `mcl_mtpa_fw` | `mcl_mtpa_fw_` | MTPA 电流分配 + 弱磁 Id 调节 | ✅ |
| `mcl_protection` | `mcl_protection_` | 过流/过压/欠压/过温（降额+关断）/堵转检测 + 温度降额系数 | ✅ |
| `mcl_calibration` | `mcl_calibration_` | 电流零漂、编码器对齐、相序、相电阻/电感 | ✅ |
| `mcl_hal` | `mcl_hal_` | HAL 抽象接口（宿主实现） | 接口 |

### 4.2 依赖 DAG

```text
                    mcl（门面）
        ┌──────┬──────┼──────────┬──────────┐
     mcl_foc  mcl_bldc_comm  mcl_protection  mcl_calibration
        │          │
   ┌────┼─────┐   mcl_transform
mcl_pid  mcl_transform  mcl_svpwm
        │
mcl_observer → mcl_transform
mcl_pll
mcl_mtpa_fw（被 mcl_foc 调用）
```

- 依赖方向自上而下，**无环**。
- 依赖经 `init(self, 依赖…)` 注入；模块间不 include 对方实现（只 include 公开头）。
- 所有模块通过门面持有的 `mcl_hal_ops` 访问硬件，算法模块自身**不持 HAL**（由门面/编排层传入采样值）。

---

## 5. 对象模型

### 5.1 集中配置 `mcl_config`

对齐 VESC 的 `mc_configuration`——一个结构体装全部参数，作为「唯一事实源」，支持整体拷贝 / 运行时更新：

```c
typedef struct
{
    /* 电机参数 */
    uint8_t  pole_pairs;         /**< 极对数 */
    float    phase_resistance;   /**< 相电阻 Ω */
    float    phase_inductance;   /**< 相电感 H */
    float    bemf_const;         /**< 反电动势常数 V/(rad/s) */
    float    rated_current;      /**< 额定电流 A */
    float    rated_speed_rpm;    /**< 额定转速 rpm */

    /* 运行配置 */
    uint32_t pwm_freq_hz;        /**< PWM 频率 */
    uint32_t current_loop_freq_hz; /**< 电流环频率 */
    uint8_t  speed_loop_divider; /**< 速度环分频（相对电流环） */
    uint8_t  pos_loop_divider;   /**< 位置环分频（相对速度环） */
    float    max_duty;           /**< 最大占空比 0~1 */
    float    bus_voltage;        /**< 标称母线电压 V */

    /* 控制环参数 */
    mcl_pid_params current_pid;  /**< 电流环 PID */
    mcl_pid_params speed_pid;    /**< 速度环 PID */
    mcl_pid_params pos_pid;      /**< 位置环 PID */

    /* 反馈配置 */
    mcl_feedback_cfg feedback;   /**< 编码器/霍尔/无感/观测器 */

    /* 保护阈值 */
    mcl_protection_limits limits;/**< 保护阈值 + enabled 使能位掩码 */

    /* 校准 */
    float current_offset[3];     /**< 电流零漂（三相） */
    float encoder_offset;        /**< 编码器电零位 rad */
} mcl_config;
```

### 5.2 电机对象 `mcl`

对应 VESC 的 `motor_all_state_t`（电机完整运行时状态），但收敛为门面对象，由宿主持有：

```c
typedef struct
{
    mcl_config cfg;              /**< 配置快照 */
    mcl_mode   mode;             /**< 运行模式 */
    mcl_state  state;            /**< 运行状态 */
    mcl_fault  fault;            /**< 当前故障 */

    const mcl_hal_ops *hal;      /**< HAL 注入 */
    void              *hal_ctx;  /**< HAL 上下文 */

    /* 子模块实例（内部） */
    mcl_foc        foc;          /**< FOC 编排 */
    mcl_bldc_comm  bldc;         /**< 六步编排 */
    mcl_observer   observer;     /**< 观测器载体（ops + impl + params） */
    mcl_pll        pll;          /**< PLL */
    mcl_pid        pid_speed;    /**< 速度环 */
    mcl_pid        pid_pos;      /**< 位置环 */
    mcl_mtpa_fw    mtpa_fw;      /**< MTPA/弱磁 */
    mcl_protection protection;   /**< 保护 */
    mcl_calibration calibration; /**< 校准 */

    /* 运行时变量（内部） */
    mcl_scalar iq_ref;           /**< 当前 Iq 目标 */
    mcl_scalar speed_ref_rpm;    /**< 速度目标 */
    mcl_scalar pos_ref_rad;      /**< 位置目标 */
    mcl_scalar phase_rad;        /**< 当前电气角 */
    mcl_scalar speed_rad_s;      /**< 当前速度 */
    /* …（遥测缓存等） */
} mcl;
```

- 实例由宿主持有（`static mcl s_motor;`），库不分配堆内存。
- 结构体暴露在 `mcl_types.h` 仅为满足静态分配；内部字段标注 private，宿主不直接访问。

### 5.3 HAL 抽象 `mcl_hal_ops`

```c
typedef struct
{
    void (*pwm_set_duty)(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc);   /**< 三相占空比 [-1,1] */
    int (*adc_read_phase)(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic); /**< 相电流 A */
    int (*adc_read_bus)(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus);       /**< 母线电压/电流 */
    int (*enc_read_angle)(void *ctx, mcl_scalar *angle_rad);             /**< 转子电气角（有感） */
    int (*enc_read_speed)(void *ctx, mcl_scalar *speed_rad_s);           /**< 转子速度（有感） */
    int (*read_temp)(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet); /**< 温度 ℃ */
    uint32_t (*micros)(void *ctx);                                   /**< 时间基准 µs */
} mcl_hal_ops;
```

- 全回调带 `void *ctx`（多实例，编码规范 §8）。
- 库不注册中断、不碰寄存器；宿主在电流环中断里调用 `mcl_control_tick()`，并保证采样值在调用前就绪。

---

## 6. 公共 API 分层

### 6.1 门面 API（`mcl.h`，宿主唯一入口）

```c
/* 生命周期 */
void mcl_init(mcl *self, const mcl_config *cfg,
              const mcl_hal_ops *hal, void *hal_ctx,
              const mcl_observer_ops *obs_ops, void *obs_impl, void *obs_params);
void mcl_deinit(mcl *self);

/* 模式与启停 */
int mcl_set_mode(mcl *self, mcl_mode mode);
int mcl_start(mcl *self);
int mcl_stop(mcl *self);

/* 指令（按控制环选择其一） */
int mcl_set_current(mcl *self, mcl_scalar iq_ref);
int mcl_set_speed(mcl *self, mcl_scalar speed_rpm);
int mcl_set_position(mcl *self, mcl_scalar pos_rad);
int mcl_set_torque(mcl *self, mcl_scalar torque_nm);

/* 查询 */
int mcl_get_state(mcl *self, mcl_state *out);
int mcl_get_fault(mcl *self, mcl_fault *out);
int mcl_get_fault_info(mcl *self, mcl_fault_info *out);   /**< 故障现场快照 */
int mcl_fault_assert(mcl *self, mcl_fault fault);          /**< 硬件保护上报 */
int mcl_clear_fault(mcl *self);
int mcl_get_telemetry(mcl *self, mcl_telemetry *out);

/* 控制节拍（宿主在电流环中断内调用） */
void mcl_control_tick(mcl *self);

/* 校准（阻塞式，宿主调用时电机停转） */
int mcl_calibrate_offset(mcl *self);
int mcl_calibrate_align(mcl *self);
int mcl_calibrate_resistance(mcl *self, float *resistance);
int mcl_calibrate_inductance(mcl *self, float *inductance);
```

### 6.2 算法模块 API（供库内部 + 高级用户）

各算法模块暴露独立接口（`mcl_transform_*` / `mcl_pid_*` / …），可脱离门面单独使用与单测。例如：

```c
void mcl_transform_clarke(mcl_scalar ia, mcl_scalar ib, mcl_scalar ic, mcl_scalar *alpha, mcl_scalar *beta);
void mcl_transform_park(mcl_scalar alpha, mcl_scalar beta, mcl_scalar phase, mcl_scalar *id, mcl_scalar *iq);
void mcl_transform_inv_park(mcl_scalar vd, mcl_scalar vq, mcl_scalar phase, mcl_scalar *alpha, mcl_scalar *beta);

void mcl_pid_init(mcl_pid *self, const mcl_pid_params *params);
mcl_scalar mcl_pid_run(mcl_pid *self, mcl_scalar error, mcl_scalar dt);

void mcl_observer_update(mcl_observer *self, mcl_scalar v_alpha, mcl_scalar v_beta,
                         mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                         mcl_scalar *phase, mcl_scalar *speed);
void mcl_pll_run(mcl_pll *self, mcl_scalar phase, mcl_scalar dt, mcl_scalar *phase_out, mcl_scalar *speed_out);
void mcl_svpwm_run(mcl_scalar v_alpha, mcl_scalar v_beta, mcl_scalar max_duty,
                   mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc);
```

---

## 7. 控制循环数据流（`mcl_control_tick` 内）

```text
mcl_control_tick(self)
  │
  ├─ 1. 采样：hal->adc_read_phase() → ia/ib/ic，hal->adc_read_bus() → vbus
  ├─ 2. Clarke：mcl_transform_clarke() → i_alpha/i_beta
  ├─ 3. 相位/速度（按模式）：
  │      · 有感 FOC：hal->enc_read_angle() → phase；enc_read_speed() → speed
  │      · 无感 FOC：mcl_observer_update() → phase；mcl_pll_run() → phase/speed
  │      · 六步 BLDC：hall 或 BEMF 过零换相（mcl_bldc_comm）
  ├─ 4. 外环分频（库内计数器）：
  │      · 位置环：mcl_pid_run(pid_pos) → speed_ref
  │      · 速度环：mcl_pid_run(pid_speed) → iq_ref
  ├─ 5. FOC 电流环（mcl_foc）：
  │      Park → id/iq → PI(pid_d/pid_q) → 解耦 + MTPA/弱磁 → vd/vq
  │      → 反 Park → v_alpha/v_beta → mcl_svpwm_run() → da/db/dc
  ├─ 6. 保护：mcl_protection_check() → 越限则关断、置 FAULT
  └─ 7. 输出：hal->pwm_set_duty(da, db, dc)
```

- 电流环每次 tick 都跑；速度环每 `speed_loop_divider` 个 tick 跑一次；位置环再按 `pos_loop_divider` 分频。
- 六步方波模式下，第 4~5 步替换为 `mcl_bldc_comm` 的换相逻辑（梯形波调制）。

---

## 8. 关键设计决策（对 VESC 的取舍）

| 决策 | 理由 |
|---|---|
| **宏抽象 → 函数指针注入** | VESC 用 `hw.h` 宏（`GET_CURRENT1`、`TIMER_UPDATE_DUTY`），编译期单例、无法多实例；mcl 要可复用，改用 `mcl_hal_ops` + `void *ctx` |
| **PLL 独立成模块** | VESC 的 `foc_pll_run` 埋在 `foc_math` 里；mcl 拆出 `mcl_pll`，与观测器解耦，可单独替换（如换滑模观测器） |
| **门面瘦身** | VESC 的 `mc_interface` 混入通信/存储/统计/里程；mcl 门面只保留状态机 + 指令 + 遥测 + 环级联，其余 out of scope |
| **配置集中但裁剪** | 借鉴 `mc_configuration` 的「一个结构体装全部」，但去掉通信/存储/双电机宏相关字段 |
| **库不注册中断** | 宿主在 PWM 中断调 `mcl_control_tick()`，库保持「只被调用、不主动抢占」的姿态 |
| **采样与 PWM 同步由宿主保证** | VESC 的 V0/V7 采样、双电机分时是硬件编排；mcl 只约定「tick 前采样就绪」 |
| **编码器驱动不内置** | VESC 内置 16 种编码器；mcl 只定义 `enc_read_angle/speed` 接口，具体驱动归宿主（符合库边界） |

---

## 9. 与规格书的对应

| 规格书条目 | 本设计落点 |
|---|---|
| FR-1.1/1.2 FOC | `mcl_foc` + `mcl_transform` + `mcl_observer` + `mcl_pll` |
| FR-1.3 六步 BLDC | `mcl_bldc_comm` |
| FR-2 控制环 | `mcl_pid`（电流/速度/位置复用）+ 门面环级联 |
| FR-2.4 MTPA/弱磁 | `mcl_mtpa_fw` |
| FR-3 调制 | `mcl_svpwm` |
| FR-5 保护 | `mcl_protection` |
| FR-6 校准 | `mcl_calibration` |
| NFR-1 可移植 | 纯算法层零硬件依赖，硬件走 `mcl_hal` |
| NFR-4 可测试 | 算法模块独立接口 + mock HAL |

---

## 10. 变更记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1.0 | 2026-09-05 | 初稿：三层架构、目录、模块 DAG、对象模型、API 分层、数据流、对 VESC 的取舍 |
