# mcl 使用指南

本文档面向**使用 mcl 库开发电机控制的宿主工程师**，手把手说明：如何集成、如何写 HAL、如何配置、如何选择观测器与精度、如何做保护与校准。库的定位、架构设计、定点归一化理论见同目录的 `mcl_spec.md`、`mcl_architecture.md`、`mcl_fixed_point.md`，本文不重复，只讲"怎么用"。

---

## 1. 库是什么、不是什么

**mcl 是一个嵌入式电机控制"库"**，不是完整的应用框架：

- ✅ 提供：FOC（有感/无感）+ 六步 BLDC 的全部控制算法、配置、保护、校准、HAL 抽象
- ❌ 不碰：寄存器、中断（除 `mcl_control_tick` 由你在中断里调用）、对外通信、参数存储
- **你负责**：写 `mcl_hal_ops`（硬件驱动）、写 main + 中断、测电机参数

设计原则（贯穿全文）：
1. **库不逆向调用你** —— 除了 HAL 回调，库不注册任何中断、不阻塞等待。
2. **你持有实例** —— `static mcl s_motor;` 静态分配，无堆分配。
3. **一个控制节拍一个入口** —— 中断里固定频率调 `mcl_control_tick()`。

---

## 2. 五分钟上手

### 2.1 编译

```powershell
# 把 include/ 加入头文件搜索路径，src/*.c 加入编译
gcc -std=c99 -Iinclude ... src/*.c
```

三种精度（默认 float）：

```text
-DMCL_USE_Q15     Q1.15 定点（int16_t，范围 [-1,1)）
-DMCL_USE_Q31     Q31 定点（int32_t，范围 [-1,1)）
（都不加）        float 单精度
```

### 2.2 最小集成骨架

```c
#include "mcl.h"
#include "mcl_observer_flux.h"   /* 或用 ortega/smo，见 §6 */

static mcl               s_motor;
static mcl_observer_flux s_obs;   /* 无感才需要；有感可省 */

void app_init(void)
{
    mcl_config cfg;
    mcl_config_default(&cfg);          /* 1. 拿一份安全默认 */
    cfg.pole_pairs = 7;                /* 2. 只改你需要的 */
    cfg.phase_resistance  = MCL_FROM_FLOAT(0.5f);
    cfg.phase_inductance  = MCL_FROM_FLOAT(0.4e-3f);
    cfg.bemf_const        = MCL_FROM_FLOAT(0.009f);   /* 磁链 λ */
    cfg.rated_current     = MCL_FROM_FLOAT(20.0f);

    mcl_observer_flux_params op = {
        .lambda     = MCL_FROM_FLOAT(0.009f),
        .resistance = MCL_FROM_FLOAT(0.5f),
        .inductance = MCL_FROM_FLOAT(0.4e-3f),
        .gain       = MCL_FROM_FLOAT(200.0f),
    };

    /* 3. 你的 HAL（见 §4） */
    static const mcl_hal_ops hal = { .pwm_set_duty = my_pwm, .adc_read_phase = my_adc, ... };

    /* 4. 初始化，注入观测器 */
    mcl_init(&s_motor, &cfg, &hal, hal_ctx, &mcl_observer_flux_ops, &s_obs, &op);

    /* 5. 设模式 + 指令 + 启动 */
    mcl_set_mode(&s_motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_speed(&s_motor, MCL_FROM_FLOAT(3000.0f));
    mcl_start(&s_motor);
}

/* 6. 在电流环中断里高频调用（频率 = cfg.current_loop_freq_hz） */
void pwm_isr(void)
{
    mcl_control_tick(&s_motor);
}
```

跑起来后，用 `mcl_get_telemetry()` 读转速/电流，用 `mcl_get_fault()` 查故障。

---

## 3. 核心概念

### 3.1 运行状态机

```
IDLE ──mcl_start()──▶ RUN ──故障/保护──▶ FAULT ──(fault_stop_time 或 clear)──▶ IDLE
```

- 只有 `RUN` 状态才会跑控制循环。
- 进 FAULT 自动关 PWM；`fault_stop_time > 0` 自动恢复，`= 0` 需手动 `mcl_clear_fault()`。

### 3.2 控制模式

`mcl_set_mode()` 选**整体模式**（FOC 有感 / FOC 无感 / BLDC），`mcl_set_speed/current/position/torque` 选**控制环层级**：

| 指令 | 控制环 | 输出 |
|---|---|---|
| `mcl_set_current(iq)` | 只有电流环 | 直接设 Iq |
| `mcl_set_speed(rpm)` | 速度环→电流环 | 速度参考 |
| `mcl_set_position(rad)` | 位置环→速度环→电流环 | 位置参考 |
| `mcl_set_torque(nm)` | 电流环（MTPA 换算 Id/Iq） | 转矩参考 |
| `mcl_set_openloop_*` | 开环（VF/IF/ALIGN） | 启动/校准用 |

### 3.3 `mcl_scalar` 与单位

所有数值都是 `mcl_scalar`。**非常关键**：

- **float 模式**：物理量直接填（Ω、H、V、A、rad/s、rpm、℃、s）
- **定点模式**：物理量必须先按 per-unit 归一化到 `[-1,1)`，用 `MCL_FROM_FLOAT(物理量/基值)` 填

**角度约定**（最容易踩坑）：

| 模式 | 角度单位 | 一圈 |
|---|---|---|
| float | 弧度 | `2π ≈ 6.28` |
| Q15/Q31 | 归一化角 | `1.0` |

所以含角度的量（`mcl_set_position` 的目标角、`openloop_seed_angle`）在定点下要按"圈"填，不是弧度。详见 `docs/spec/mcl_fixed_point.md` §4。

---

## 4. 实现 HAL（`mcl_hal_ops`）

这是**唯一需要你写硬件的部分**。接口定义见 `include/mcl_hal.h`，共 10 个回调，按"必做/按需"分三档：

### 4.1 必做（FOC 无感）

| 回调 | 作用 | 注意 |
|---|---|---|
| `pwm_set_duty(ctx, da, db, dc)` | 输出三相占空比 | **da/db/dc ∈ [-1,1]**（相对中性点，不是 [0,1]），要映射到你的 PWM 定时器 |
| `adc_read_phase(ctx, *ia,*ib,*ic)` | 读三相电流 | 返回**物理 A**（float）或 **per-unit**（定点） |

### 4.2 按模式按需

| 回调 | FOC 有感 | FOC 无感 | BLDC |
|---|---|---|---|
| `adc_read_bus`（母线电压/电流） | ✓ | ✓ | ✓ |
| `enc_read_angle` / `enc_read_speed` | ✓ | | |
| `read_hall` | | | ✓（霍尔换相） |
| `adc_read_phase_voltage` | | | ✓（BEMF 换相，无感） |
| `read_temp`（电机/FET 温度） | 保护用，建议 | 保护用，建议 | 保护用，建议 |
| `micros`（微秒时间基准） | 校准用，建议 | 校准用，建议 | 校准用，建议 |

### 4.3 HAL 回调约定

- 采样类回调返回 `MCL_OK` 或 `MCL_ERR_HAL`；失败时库会安全关断。
- `pwm_set_duty` **无返回值**（高频路径），出错由保护/`mcl_fault_assert` 处理。
- 所有回调带 `void *ctx`，支持多电机实例——把实例指针传进去区分。

### 4.4 最小示例（伪代码）

```c
static void my_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    /* [-1,1] → 定时器 CCR：mid + duty*(mid-dead) */
    TIM1->CCR1 = PWM_MID + MCL_TO_FLOAT(da) * (PWM_MID - DEADTIME);
    TIM1->CCR2 = PWM_MID + MCL_TO_FLOAT(db) * (PWM_MID - DEADTIME);
    TIM1->CCR3 = PWM_MID + MCL_TO_FLOAT(dc) * (PWM_MID - DEADTIME);
}

static int my_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    float a = adc_to_ampere(ADC1->DR);
    float b = adc_to_ampere(ADC2->DR);
    /* float：直接物理；定点：除以 I_BASE 再 MCL_FROM_FLOAT */
    *ia = MCL_FROM_FLOAT(a);
    *ib = MCL_FROM_FLOAT(b);
    *ic = MCL_FROM_FLOAT(-(a+b));   /* 两电阻采样重建第三相 */
    return MCL_OK;
}
```

---

## 5. 配置（`mcl_config`）

用 `mcl_config_default()` 拿默认值，**只改你要改的字段**（字段语义见 `mcl_config.h` 的逐字段注释）。

### 5.1 必改的电机参数

| 字段 | 含义 | 备注 |
|---|---|---|
| `pole_pairs` | 极对数 | 必填 >0 |
| `phase_resistance` | 相电阻 | 校准可测（§8） |
| `phase_inductance` | q 轴电感 Lq | 校准可测 |
| `bemf_const` | 磁链 λ (Wb) | 校准可测 |
| `rated_current` | 额定电流 | 保护/限幅基准 |

`ld_lq_diff`（= Lq−Ld）为 0 是表贴式 SPMSM；>0 是内嵌式 IPMSM（启用 MTPA）。

### 5.2 频率

| 字段 | 含义 |
|---|---|
| `current_loop_freq_hz` | 电流环频率 = `mcl_control_tick` 调用频率 |
| `pwm_freq_hz` | PWM 开关频率 |
| `speed_loop_divider` / `pos_loop_divider` | 外环分频（相对电流环） |

> `mcl_control_tick` 必须**严格按 `current_loop_freq_hz` 频率**调用，否则 dt 错、闭环失稳。

### 5.3 保护阈值

`limits`（见 §7），默认全开。定点下阈值也要 per-unit 归一化。

---

## 6. 观测器选择（无感 FOC）

库提供三个可插拔观测器（`mcl_observer_ops` 接口），注入方式一样：

```c
mcl_init(&sm, &cfg, &hal, ctx, &XXX_ops, &XXX_inst, &XXX_params);
```

| 观测器 | 文件 | 适用 | 特点 |
|---|---|---|---|
| **磁链 flux** | `mcl_observer_flux.h` | 参考/学习 | 朴素电压积分，需 gain 校正 |
| **ORTEGA** | `mcl_observer_ortega.h` | **无感闭环首选** | 幅值误差反馈，全局收敛，Q15 变速稳定 |
| **SMO 滑模** | `mcl_observer_smo.h` | 中高速稳态 | AN1078 式，恒速 ~1°；**Q15 变速相位不准**（见下） |

**选型建议**（我们的验证结论）：

- **无感闭环（含变速、零速启动）→ 用 ORTEGA**，三精度都稳。
- SMO 恒速很准（~1°），但 **Q15 变速相位会 ±130° 振荡**（16 位量化极限）；变速场景请用 Q31/float 或换 ORTEGA。
- 有初始速度、稳速运行的场景，SMO 是低成本好选择。

---

## 7. 保护

### 7.1 软件保护（控制环内自动）

`limits.enabled` 位掩码开关，默认全开：

| 保护 | 触发 | 使能位 |
|---|---|---|
| 过流 | 任一相 `\|i\|>overcurrent` | `MCL_PROTECT_OVERCURRENT` |
| 过压/欠压 | 母线越限 | `MCL_PROTECT_OVERVOLTAGE` / `UNDERVOLTAGE` |
| 过温 | 温度超过 `overtemp`（降额起始 `temp_derate_start`） | `MCL_PROTECT_OVERTEMP` |
| 堵转 | 速度 < `stall_speed` 持续 `stall_time` | `MCL_PROTECT_STALL` |

温度有**两级**：`temp_derate_start ~ overtemp` 之间线性降额（减电流），超 `overtemp` 关断。

### 7.2 硬件保护（你在中断里上报）

过流比较器、门驱 nFAULT 等硬件故障，在中断里调：

```c
mcl_fault_assert(&s_motor, MCL_FAULT_DRV);   /* 立即关 PWM + 记录快照 */
```

> **没有独立的"失步"保护**（对齐 VESC 设计）：失步的后果由过流/堵转自然兜底。

### 7.3 故障处理

- `mcl_get_fault()` 查当前故障，`mcl_get_fault_info()` 查关断瞬间的快照（电流/电压/转速/温度/tick）。
- `fault_stop_time > 0`：持续该时长后自动恢复；`= 0`：需 `mcl_clear_fault()` 手动清除。

---

## 8. 校准（阻塞式，停转时调）

`mcl_calibrate_*` 是阻塞式，电机**必须停转**时调用：

```c
mcl_calibrate_offset(&sm);          /* 电流零漂 → 写入 cfg.current_offset */
mcl_calibrate_align(&sm);           /* 编码器电零位对齐 */
mcl_calibrate_resistance(&sm, &R);  /* 测相电阻 */
mcl_calibrate_inductance(&sm, &L);  /* 测相电感 */
```

依赖 HAL 的 `micros()` 做忙等待延时。测完把 R/L/λ 回填进 `cfg` 再 `mcl_init`。

> 磁链 λ 测量 `mcl_cal_flux_linkage`、霍尔相序 `mcl_cal_hall_detect` 见 `mcl_calibration.h`。

---

## 9. 常见问题（FAQ）

**Q1：电机不转 / 抖动**
→ 先查 `mcl_get_fault()`，多半是保护触发（过流/堵转）；再核对 `pole_pairs`、相序、电流采样方向。

**Q2：无感起不来 / 启动失败**
→ 无感从零启动需要"自动开环→切闭环"（库已内置，配置 `openloop_*` 系列字段）。启动参数（`openloop_rpm`、`openloop_seed_angle` 等）需针对具体电机标定。若仍失败，检查观测器参数归一化是否正确。

**Q3：切定点后数值全错**
→ 物理量没归一化。定点范围 `[-1,1)`，所有物理量要除基值。见 `mcl_fixed_point.md` §6 七步切换清单。

**Q4：PID 参数怎么定？**
→ 电流环先调（一般 kp≈L·带宽、ki≈R·带宽），再速度环（kp/ki 远小于电流环）。定点下增益必须 <1（per-unit 重调）。`mcl_pid_params.ki` 是连续域（库内部乘 dt），`kd` 是离散域（已含 1/dt）。

**Q5：怎么知道控制节拍频率对不对？**
→ `mcl_control_tick` 调用频率必须等于 `current_loop_freq_hz`。用示波器量一个 GPIO 翻转周期，或让中断里计数核对。

**Q6：SMO 还是 ORTEGA？**
→ 见 §6：闭环/变速用 ORTEGA，稳速用 SMO（且 SMO Q15 变速不准）。

---

## 10. 下一步

- 完整规格与需求：`docs/spec/mcl_spec.md`
- 架构与模块依赖：`docs/spec/mcl_architecture.md`
- 定点归一化与切换：`docs/spec/mcl_fixed_point.md`
- 逐字段配置语义：`include/mcl_config.h`
- 可运行示例：`tests/sim_test.c`、`tests/openloop_test.c`（PC 端 mock HAL，看懂后仿写真实 HAL）
