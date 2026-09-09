# mcl 电机控制库规格书

| 项 | 值 |
|---|---|
| 文档名称 | mcl 电机控制库规格书（Motor Control Library Specification） |
| 版本 | v0.1.0（草案） |
| 状态 | 评审中 |
| 日期 | 2026-09-05 |
| 代码位置 | 根目录 `include/` + `src/`（自包含、可复用电机控制库） |
| 目标架构 | ARM Cortex-M 系列（通用，不绑定具体型号） |

---

## 1. 引言

### 1.1 目的

本文档定义 **mcl（Motor Control Library）** 的功能需求、软件架构、公共 API、时序约束与验收标准。它是后续设计、编码、测试的唯一功能基线；实现与本规格书冲突时，以本规格书为准。

### 1.2 范围

本文档覆盖 mcl 库自身：控制算法、控制环、调制输出、位置/速度反馈、保护与校准。不覆盖宿主应用的业务逻辑与对外通信（见 §2.3）。

### 1.3 术语与缩写

| 术语 | 含义 |
|---|---|
| FOC | Field Oriented Control，磁场定向控制（矢量控制） |
| BLDC | Brushless DC，无刷直流电机（六步方波驱动） |
| PMSM | Permanent Magnet Synchronous Motor，永磁同步电机 |
| MTPA | Maximum Torque Per Ampere，最大转矩电流比 |
| FW | Field Weakening，弱磁控制 |
| SVPWM | Space Vector PWM，空间矢量脉宽调制 |
| d/q 轴 | 同步旋转坐标系（d 磁链轴 / q 转矩轴） |
| α/β 轴 | 静止两相正交坐标系 |
| 观测器 | 无传感器位置/速度估计器（磁链观测器/滑模等） |
| 宿主 | 调用 mcl 的上层应用（app / middleware 层），负责接线与业务 |
| HAL | 本库定义的外设抽象接口，由宿主实现并注入 |

### 1.4 参考文档

| 文档 | 说明 |
|---|---|
| `docs/规范/architecture.md` | 架构规范：分层、依赖 DAG、组合根、初始化、命名 |
| `docs/规范/coding_standard.md` | C 编码规范（可复用库回调一律带 ctx） |
| `docs/spec/mcl_fixed_point.md` | 定点使用说明：三种精度、归一化、转换/运算宏、角度约定 |
| `reference/bldc/` | VESC 固件（vedderb/bldc），FOC/BLDC 参考实现 |

---

## 2. 总体描述

### 2.1 定位

mcl 是一个 **自包含、可复用的电机控制库**，以 C 源码 / 静态库形式交付，被宿主工程通过 **C API** 调用。它实现电机控制的核心算法与控制环，**不内置任何通信协议、参数存储或人机交互**——这些由宿主负责。

作为自包含库，mcl 满足：

- **自包含**：源码内聚，仅依赖 `stdint.h` / `stdbool.h`；数学函数走可插拔的 `mcl_math` 层（默认内置查表），不依赖任何厂商 SDK 或 RTOS。
- **可整体摘走复用**：目录可整体拷贝到任意 ARM Cortex-M 工程，不改源码即可使用。
- **保留 `mcl_` 前缀**：所有公开符号带 `mcl_` 前缀，避免与宿主 / 其它库裸名冲突（见编码规范 §1.4）。

### 2.2 设计目标与原则

1. **平台无关（ARM 通用）**：核心算法不含任何寄存器 / 芯片相关代码；硬件访问一律通过注入的 HAL 抽象接口。
2. **自包含可复用**：不依赖厂商 SDK / RTOS；可整体摘走，跨 ARM Cortex-M 芯片复用。
3. **依赖注入**：模块之间不互相 `#include` 实现，依赖统一走 `init(self, 依赖…)`，由宿主接线。
4. **回调带 ctx**：所有对外回调（HAL 抽象）一律带 `void *ctx`，支持多实例（编码规范 §8 的可复用库铁律）。
5. **确定性实时**：控制路径无动态内存分配、无阻塞等待、无浮点异常路径（含除零保护）。
6. **可测试**：纯算法模块与硬件解耦，可脱离硬件做单元测试 / 仿真（SIL）。

### 2.3 范围边界

**在范围内（in scope）：**

- FOC 矢量控制（有感 + 无感）
- 六步方波 BLDC 换相
- 电流环（Id/Iq）、速度环、位置环
- MTPA 与弱磁（FW）
- Clark / Park 变换及逆变换、SVPWM 调制
- 无传感器位置/速度观测器
- 保护（过流 / 过压 / 欠压 / 过温 / 堵转）
- 校准（电流零漂、编码器对齐、相序检测、相电阻/电感测量）
- 公共 API 与 HAL 抽象接口定义

**不在范围内（out of scope）：**

- 任何对外通信（UART / CAN / USB / 无线）及协议栈
- 参数持久化存储（Flash 读写 / EEPROM）
- 应用层业务逻辑（遥控解析、上位机命令、状态机产品化）
- 具体功率级 / 驱动板硬件设计
- GUI / 上位机软件
- 厂商 SDK / 寄存器级驱动（属于宿主工程的 `mcal` / `hal` 层）

> 上述 out-of-scope 项由宿主通过调用 mcl 的 API 实现。

---

## 3. 硬件平台

### 3.1 目标架构

| 项 | 值 |
|---|---|
| 目标架构 | ARM Cortex-M 系列（通用，覆盖 M0 / M3 / M4 / M7） |
| 编程语言 | C（C99+） |
| 标准依赖 | `stdint.h` / `stdbool.h`（数学经 `mcl_math` 可插拔层，默认内置查表） |
| 浮点 | 优先单精度 FPU（M4F / M7）；无 FPU 时可配置软件浮点 / 定点回退 |

> mcl 不绑定具体芯片型号；任何满足上述条件的 ARM Cortex-M 芯片均可承载。

### 3.2 外设抽象（不绑定具体型号）

库核心通过 HAL 接口访问以下**抽象外设**，具体外设由宿主按芯片实现：

| 抽象能力 | 说明 |
|---|---|
| PWM 输出 | 三相互补 PWM，死区插入（高级定时器 / HRTIM 等） |
| 模拟采样 | 多通道同步 ADC（相电流 / 母线电压 / 母线电流） |
| 信号调理 | 运放（PGA）/ 比较器（电流调理、硬件过流） |
| 编码器接口 | 正交解码（ABI）/ 捕获（hall）/ SPI（绝对位置） |
| 时间基准 | 微秒级计时（用于无感估计与超时） |
| 加速器（可选） | CORDIC / FMAC 等，非必需，仅作优化 |

### 3.3 参考落地实例

| 项 | 值 |
|---|---|
| 芯片 | STM32G474VET6（Cortex-M4F，170 MHz，FPU + DSP） |
| 特点 | HRTIM / 多 ADC / OPAMP / COMP / CORDIC / FMAC，适合电机控制 |

> 该实例仅用于**验证 mcl 的可移植性与性能**；mcl 源码不包含、不依赖其具体外设寄存器。

### 3.4 功率级与传感器假设

- 三相电压型逆变器（6 个 MOSFET），母线电压范围由宿主配置。
- 电流采样：三电阻 / 双电阻 / 单电阻（三选一，通过配置与 HAL 适配）。
- 位置反馈（有感）：ABI 编码器、霍尔传感器、SPI 绝对位置编码器（任选其一，经统一抽象）。
- 无感模式：仅依赖相电流与母线电压。

---

## 4. 软件架构

### 4.1 代码位置

mcl 是工程**第一方代码**，源码置于根目录 `include/`（公开头）与 `src/`（实现），
作为自包含电机控制库独立于 `app → middleware → drivers → hal → bsp` 分层链之外：

```text
app / middleware              宿主业务与通信
        ↓ 调用 mcl API
include/ + src/               mcl 库（本规格书范围，自包含、可复用）
        ↓ 注入 hal 抽象接口（宿主实现）
hal / bsp                     宿主工程的外设薄封装与板级
```

> mcl 依赖的是**接口**（`mcl_hal_ops`），不依赖 hal 的具体实现；宿主把 hal 实现注入给 mcl，保证库对芯片/板卡透明。

### 4.2 模块划分

| 模块 | 公开符号前缀 | 职责 | 平台无关 |
|---|---|---|---|
| `mcl` | `mcl_` | 库总入口 / 电机对象：状态机（IDLE/RUN/FAULT）、模式切换、环级联、对外 API | ✅ |
| `mcl_math` | `mcl_math_` | 数学库抽象：可插拔 + 三精度（float / Q15 / Q31，默认查表） | ✅ |
| `mcl_transform` | `mcl_transform_` | Clark / Park 变换及逆变换 | ✅ |
| `mcl_foc` | `mcl_foc_` | FOC 电流环核心：dq 解耦、电压前馈、限幅 | ✅ |
| `mcl_svpwm` | `mcl_svpwm_` | SVPWM 调制，含过调制 | ✅ |
| `mcl_observer` | `mcl_observer_` | 观测器载体：统一 ops 接口，可插拔多种算法（磁链/滑模/…） | 接口 |
| `mcl_observer_flux` | `mcl_observer_flux_` | 磁链观测器参考实现（插槽模板） | ✅ |
| `mcl_observer_smo` | `mcl_observer_smo_` | 滑模观测器（电流滑模 + 反电动势低通 + 相位补偿） | ✅ |
| `mcl_observer_ortega` | `mcl_observer_ortega_` | ORTEGA 型磁链观测器（带相位收敛的非线性反馈） | ✅ |
| `mcl_pid` | `mcl_pid_` | 通用 PID 控制器（电流/速度/位置环复用） | ✅ |
| `mcl_mtpa_fw` | `mcl_mtpa_fw_` | MTPA 与弱磁控制（IPMSM 支持，float 中转保证三精度） | ✅ |
| `mcl_bldc_comm` | `mcl_bldc_comm_` | 六步换相逻辑（hall 换相 / 无感 BEMF 积分换相） | ✅ |
| `mcl_protection` | `mcl_protection_` | 过流/过压/欠压/过温/堵转保护与故障处理 | ✅ |
| `mcl_calibration` | `mcl_calibration_` | 校准：电流零漂、编码器对齐、相电阻 R、相电感 L、霍尔相序检测、磁链 λ 测量 | ✅ |
| `mcl_hal` | `mcl_hal_` | 库定义的 HAL 抽象接口（PWM/ADC/编码器/定时器），由宿主实现 | 接口定义 |

### 4.3 依赖关系（DAG）

```text
               mcl（总入口，聚合所有）
        ┌────────┼────────────┬───────────┐
     mcl_foc  mcl_bldc_comm mcl_mtpa_fw mcl_protection
        │          │            │
  mcl_transform mcl_observer mcl_transform
        │          │            │
    mcl_svpwm  mcl_transform  mcl_pid
        │
     mcl_hal（接口，注入）
```

- 依赖方向自上而下，无环。
- 各算法模块之间不互相 include，依赖经 `init(self, 依赖…)` 注入（如 `mcl_foc` 注入 `mcl_pid`、`mcl_svpwm`、`mcl_hal`）。
- `mcl` 是唯一组合根，负责实例化所有子模块并接线。

### 4.4 目录结构

```text
include/
├── mcl.h                  库总头（聚合所有公开头）
├── mcl_types.h            公共类型、枚举、版本宏
├── mcl_math.h             数学库抽象（可插拔）
├── mcl_hal.h              HAL 抽象接口
├── mcl_transform.h
├── mcl_foc.h
├── mcl_svpwm.h
├── mcl_observer.h
├── mcl_observer_flux.h
├── mcl_observer_smo.h
├── mcl_observer_ortega.h
├── mcl_pid.h
├── mcl_mtpa_fw.h
├── mcl_bldc_comm.h
├── mcl_protection.h
└── mcl_calibration.h

src/
├── mcl.c
├── mcl_math.c
├── mcl_transform.c
├── mcl_foc.c
├── mcl_svpwm.c
├── mcl_observer.c
├── mcl_observer_flux.c
├── mcl_observer_smo.c
├── mcl_observer_ortega.c
├── mcl_pid.c
├── mcl_mtpa_fw.c
├── mcl_bldc_comm.c
├── mcl_protection.c
└── mcl_calibration.c
```

**include 约定**：

- 根 `include/` 进工程 `-I`。
- 对外引用写 `#include "mcl.h"`（裸文件名，架构规范 §7）。
- 库内部也统一用裸文件名 `#include "mcl_xxx.h"` 引用自身公开头。

### 4.5 实例化与回调桥接

- 实例由**宿主**持有并注入：宿主组合根 `static mcl s_mcl;`，调用 `mcl_init()` 传入实例与 HAL。
- 库内部**不自持全局单例**；所有跨模块回调走 `init` 注入的依赖或 `void *ctx`（多实例安全）。
- 若宿主中断需桥接到库回调，由宿主保存 `mcl *` 并通过 `ctx` 传回（见编码规范 §8）。

---

## 5. 功能需求

### FR-1 控制算法

#### FR-1.1 FOC 矢量控制（有感）

- 基于转子位置（编码器/霍尔）完成 Park 变换，实现 d/q 轴解耦电流环。
- 支持 SPMSM 的 `Id = 0` 控制与 IPMSM 的 MTPA 控制。
- 支持开环电压 / 闭环电流两种启动。

#### FR-1.2 FOC 矢量控制（无感）

- 通过观测器（磁链观测器或滑模观测器）估计转子位置与转速。
- 支持低速开环切入（I/F 或 V/f 拖动）、中高速无感闭环，切换无冲击。
- 提供观测器收敛判据（可上报估计可信度）。

#### FR-1.3 六步方波 BLDC

- 六步换相，支持霍尔换相与无感 BEMF 过零换相。
- 支持梯形波电流 / 电压 PWM 调制。
- 支持 120° / 60° 换相逻辑与相序自学习。

### FR-2 控制环

| 编号 | 控制环 | 需求 |
|---|---|---|
| FR-2.1 | 电流环 | d/q 轴 PI（或 P）闭环，限幅 + 抗积分饱和，电压前馈 |
| FR-2.2 | 速度环 | PI 速度闭环，级联于电流环之上，输出 Iq 参考，限幅 |
| FR-2.3 | 位置环 | P（或 P+前馈）位置闭环，级联于速度环之上，输出速度参考 |
| FR-2.4 | MTPA/弱磁 | 转矩电流按 MTPA 分配 d/q；弱磁区按电压/速度极限动态调节 Id |

- 各环可独立使能 / 旁路（单环、级联环均可）。
- 所有环支持限幅、抗积分饱和、参数运行时更新。

### FR-3 调制与 PWM

- SVPWM（含零矢量分配、过调制）与六步方波调制。
- 死区插入（由宿主 HAL 层 PWM 实现，库输出占空比）。
- 调制比 / 电压利用率可配置。

### FR-4 位置 / 速度反馈

- 统一编码器抽象：ABI、霍尔、SPI 绝对位置编码器。
- 位置/速度经滤波（含可选 FMAC 或软件滤波）后供控制环使用。
- 无感模式由观测器输出替代物理传感器。

### FR-5 保护与安全

**软件保护**（控制环内，`mcl_protection_check`）：

| 保护 | 触发条件 | 动作 |
|---|---|---|
| 过流 | 任一相电流超阈值 | 立即关断 PWM，进 FAULT |
| 过压/欠压 | 母线电压越限 | 关断 |
| 过温 | 温度越限 | 两级：先线性降额（减小电流限幅），超上限关断 |
| 堵转 | 转速低于阈值持续超时 | 关断，报堵转 |

**硬件保护**（电路直断，宿主在中断里上报 `mcl_fault_assert`）：

| 保护 | 触发来源 | 上报 |
|---|---|---|
| 过流 | 比较器 → BRK 刹车中断 | `mcl_fault_assert(MCL_FAULT_OVERCURRENT)` |
| 门驱故障 | DRV nFAULT → EXTI 中断 | `mcl_fault_assert(MCL_FAULT_DRV)` |

**故障处理机制**：

- 各保护项可**独立使能**：`limits.enabled` 位掩码（`MCL_PROTECT_OVERCURRENT/OVERVOLTAGE/UNDERVOLTAGE/OVERTEMP/STALL`，默认全开 `MCL_PROTECT_ALL`）；过温项关掉时降额一并关闭。
- 故障状态可查询（`mcl_get_fault`），触发瞬间记录**现场快照**（`mcl_get_fault_info`：电流/电压/转速/温度/时刻）。
- 故障恢复可配置：`fault_stop_time`（>0 自动恢复，=0 手动清除）。
- 温度降额：`temp_derate_start ~ overtemp` 之间线性减小电流限幅（`mcl_protection_derate`）。
- 故障动作优先级：硬件保护（宿主中断上报）> 软件保护（控制环内）。

### FR-6 校准与自检

- 电流传感器零漂自校准。
- 编码器电气零位对齐（对齐到转子 d 轴）。
- 相序检测与自纠正。
- 相电阻 / 相电感测量（供 PI 参数整定）。

---

## 6. 公共 API

> 本节为接口规约；具体签名以 `mcl.h` / `mcl_types.h` 为准。命名遵循编码规范：对象类型不带 `_t`，实例指针用 `self`，依赖走 `init(self, …)`，公开符号带 `mcl_` 前缀，库暴露版本宏（编码规范 §1.5）。

### 6.1 版本宏

```c
#define MCL_VERSION_MAJOR    0u
#define MCL_VERSION_MINOR    1u
#define MCL_VERSION_PATCH    0u
#define MCL_VERSION_STRING   "0.1.0"
#define MCL_VERSION_NUM      ((MCL_VERSION_MAJOR << 16) | (MCL_VERSION_MINOR << 8) | MCL_VERSION_PATCH)
```

### 6.2 对象与生命周期

```c
typedef struct
{
    /* 电机参数：极对数、额定电流/电压、相电阻、相电感、反电动势常数 */
    /* 运行配置：PWM 频率、电流环频率、限幅、保护阈值 */
    /* 反馈配置：编码器类型、无感/有感、观测器参数 */
} mcl_config;   /**< 电机参数与运行配置 */

void mcl_init(mcl *self, const mcl_config *cfg,
              const mcl_hal_ops *hal, void *hal_ctx);
void mcl_deinit(mcl *self);
```

- 实例由宿主持有（`static mcl s_mcl;`），库不分配堆内存。

### 6.3 核心类型

```c
typedef enum
{
    MCL_MODE_FOC_SENSORED,      /**< FOC 有感 */
    MCL_MODE_FOC_SENSORLESS,    /**< FOC 无感 */
    MCL_MODE_BLDC_HALL,         /**< 六步方波（霍尔换相） */
    MCL_MODE_BLDC_SENSORLESS,   /**< 六步方波（无感 BEMF） */
} mcl_mode;

typedef enum
{
    MCL_STATE_IDLE,             /**< 未启动 */
    MCL_STATE_ALIGN,            /**< 对齐/校准 */
    MCL_STATE_RUN,              /**< 正常运行 */
    MCL_STATE_FAULT,            /**< 故障 */
} mcl_state;

typedef enum
{
    MCL_FAULT_NONE = 0,
    MCL_FAULT_OVERCURRENT,      /**< 过流 */
    MCL_FAULT_OVERVOLTAGE,      /**< 过压 */
    MCL_FAULT_UNDERVOLTAGE,     /**< 欠压 */
    MCL_FAULT_OVERTEMP,         /**< 过温 */
    MCL_FAULT_STALL,            /**< 堵转 */
    MCL_FAULT_DRV,              /**< 门驱故障（nFAULT 引脚） */
} mcl_fault;
```

### 6.4 控制与状态接口

```c
int mcl_set_mode(mcl *self, mcl_mode mode);               /**< 切换运行模式 */
int mcl_start(mcl *self);                                 /**< 启动 */
int mcl_stop(mcl *self);                                  /**< 停止（可配置减速/立即） */

int mcl_set_current(mcl *self, mcl_scalar iq_ref);        /**< 电流环指令 */
int mcl_set_speed(mcl *self, mcl_scalar speed_rpm);       /**< 速度环指令 */
int mcl_set_position(mcl *self, mcl_scalar pos_rad);      /**< 位置环指令 */
int mcl_set_torque(mcl *self, mcl_scalar torque_nm);      /**< 转矩指令（经 MTPA 换算） */

int mcl_get_state(mcl *self, mcl_state *out);             /**< 状态查询 */
int mcl_get_fault(mcl *self, mcl_fault *out);             /**< 故障查询 */
int mcl_get_fault_info(mcl *self, mcl_fault_info *out);   /**< 故障现场快照 */
int mcl_fault_assert(mcl *self, mcl_fault fault);         /**< 硬件保护上报（宿主中断内调用） */
int mcl_clear_fault(mcl *self);                           /**< 清除故障 */
int mcl_get_telemetry(mcl *self, mcl_telemetry *out);     /**< 遥测：转速/电流/电压/位置/温度 */
```

- 返回值为错误码枚举（非裸 `-1`），失败不改变内部状态。

### 6.5 控制节拍（由宿主在中断内调用）

```c
void mcl_control_tick(mcl *self);   /**< 电流环中断内调用，执行完整控制周期 */
```

- 该函数**必须**由宿主在电流环定时中断（如 PWM 更新中断）内以固定频率调用；库自身不注册任何中断。
- 速度环 / 位置环在库内按分频执行（见 §7.1）。

### 6.6 HAL 抽象接口（由宿主实现并注入）

```c
typedef struct
{
    /* PWM 输出：占空比 [-1, 1]（负代表换相/反向） */
    void (*pwm_set_duty)(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc);
    /* 电流采样：三相电流（A），可返回 2 相由库重构第三相 */
    int (*adc_read_phase)(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic);
    int (*adc_read_bus)(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus);
    /* 位置/速度反馈（有感）；无感时返回 0 并置 err */
    int (*enc_read_angle)(void *ctx, mcl_scalar *angle_rad);
    int (*enc_read_speed)(void *ctx, mcl_scalar *speed_rad_s);
    /* 温度（电机/FET，℃） */
    int (*read_temp)(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet);
    /* 时间基准 */
    uint32_t (*micros)(void *ctx);
} mcl_hal_ops;
```

- 所有回调带 `void *ctx`，支持多实例（编码规范 §8 的可复用库铁律）。
- 库核心不包含寄存器操作；`mcl_hal_ops` 实现由宿主在 `hal` / `bsp` 层按芯片完成。

---

## 7. 控制时序与实时性

### 7.1 执行频率

| 控制环 | 典型频率 | 说明 |
|---|---|---|
| 电流环（FOC） | 10~20 kHz | 与 PWM 更新同步，`mcl_control_tick` 触发 |
| 速度环 | 1~5 kHz | 电流环整数倍分频 |
| 位置环 | 0.1~1 kHz | 速度环整数倍分频 |
| 观测器 | 与电流环同频 | 无感位置/速度估计 |

- 分频关系在 `mcl_config` 中配置，库内自动分频。

### 7.2 中断模型

- 库**不注册中断**；宿主在 PWM 更新中断（或定时中断）内调用 `mcl_control_tick()`。
- 若电流采样用 ADC 注入组 / DMA，采样完成需在 tick 前就绪；由宿主保证时序。
- 控制路径无阻塞、无锁、无动态分配，保证确定性执行。

### 7.3 数据流

```text
指令（mcl_set_speed / mcl_set_current / mcl_set_position）
        ↓
    控制环级联（位置→速度→电流）
        ↓
    d/q → α/β → SVPWM → 占空比 → HAL pwm_set_duty
        ↑
    反馈：相电流（ADC）→ Clark/Park → Id/Iq
    反馈：位置/速度（编码器 或 观测器）
        ↓
    遥测（mcl_get_telemetry）→ 宿主
```

---

## 8. 非功能需求

### NFR-1 可移植性

- 核心算法 100% 平台无关，仅依赖 `stdint.h` / `stdbool.h`；数学经 `mcl_math` 可插拔层（默认查表）。
- 不依赖厂商 SDK / RTOS；跨 ARM Cortex-M 芯片（M0/M3/M4/M7）复用，源码不改。
- 换芯片 / 换板：仅由宿主改 HAL 实现与板级，mcl 源码不变。

### NFR-2 实时性与确定性

- 电流环单周期执行时间：≤ 目标周期（20 kHz 时 ≤ 50 µs，M4F + FPU 参考值）。
- 控制路径无阻塞、无动态内存、无递归；浮点异常路径有防护（除零、NaN 检查）。

### NFR-3 资源占用（目标，以 Cortex-M4F 为参考，-O2）

| 资源 | 预算 |
|---|---|
| Flash（库代码） | ≤ 48 KB |
| RAM（实例 + 缓冲） | ≤ 8 KB（单电机实例） |
| CPU（20 kHz 电流环） | ≤ 60%（含 3 环 + 观测器） |

> 预算为设计上限，具体随编译器 / 架构（有无 FPU）浮动，以构建产物实测为准。

### NFR-4 可测试性

- 纯算法模块（transform / pid / svpwm / observer / mtpa_fw）可脱离硬件单元测试。
- 提供 SIL 仿真桩（mock `mcl_hal_ops`）用于整库回归。
- 每个公开 API 有错误码约定与输入校验（NULL 检查、越界检查）。

### NFR-5 代码质量

- 符合 `docs/规范/coding_standard.md`（命名、风格、Doxygen 注释）。
- 符合 `docs/规范/architecture.md` 对自包含库的要求（自包含、回调带 ctx、保留前缀）。
- 无编译警告（`-Wall -Wextra`），MISRA-C 相关规则作为参考（非强制）。
- 公开头文件自包含（可独立编译）。

---

## 9. 性能指标

| 指标 | 目标值 | 条件 |
|---|---|---|
| 电流环带宽 | ≥ 1 kHz | FOC，20 kHz 采样 |
| 速度环带宽 | ≥ 100 Hz | 级联闭环 |
| 速度稳态精度 | ≤ 1% | 额定负载 |
| PWM 频率 | 10~100 kHz | 视宿主定时器 / HRTIM |
| 电流采样精度 | 12/16-bit ADC | 视宿主 ADC 配置 |
| 无感最低运行转速 | ≤ 额定转速 5% | 视电机参数 |
| 无感切入转速 | 可配置（如 3~10% 额定） | 平滑切换 |

> 具体数值随电机与功率级变化，最终以台架实测验收。

---

## 10. 测试与验收

### 10.1 测试策略

| 层级 | 内容 | 工具 |
|---|---|---|
| 单元测试 | transform / pid / svpwm / observer / mtpa_fw 数值正确性 | C 测试框架（如 Unity/CMock） |
| SIL 仿真 | 整库 + mock HAL，验证状态机、环级联、保护 | 宿主测试程序（PC 端） |
| HIL 硬件在环 | 真实 ADC/PWM/编码器，验证时序与实时性 | 参考实例开发板（如 STM32G474） |
| 台架测试 | 真实电机 + 功率级，验证性能指标 | 测功台 |

### 10.2 验收标准

- 全部 FR（功能需求）逐项有对应测试用例并通过。
- 全部 NFR（非功能需求）满足：资源占用达标、无编译警告、代码符合规范。
- 保护项逐项触发并验证正确关断与可恢复。
- 性能指标（§9）实测达标或给出偏差说明。
- 可移植性验证：至少在两款不同 ARM Cortex-M 芯片（如 STM32G474 + 另一款）上构建并通过 HIL。

---

## 11. 变更记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1.0 | 2026-09-05 | 初稿：定位、ARM 通用架构、范围、模块划分、功能需求、API、时序、NFR、性能与验收 |
