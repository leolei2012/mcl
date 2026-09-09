# mcl 定点使用说明

| 项 | 值 |
|---|---|
| 文档名称 | mcl 定点使用说明 |
| 版本 | v0.2.1（草案） |
| 日期 | 2026-09-06 |
| 前置 | `docs/spec/mcl_spec.md`、`docs/spec/mcl_architecture.md` |

本文档说明 mcl 的三种数值精度（`float` / `Q1.15` / `Q31`）如何选择、如何归一化、如何切换。

---

## 1. 三种精度

mcl 用一个抽象标量类型 `mcl_scalar` 承载所有算法数值，编译期三选一：

| 精度 | 底层类型 | 范围 | 精度（LSB） | 适用芯片 |
|---|---|---|---|---|
| `float`（默认） | `float` | ±3.4e38 | ~1e-7 | M4F / M7（有 FPU） |
| `Q1.15`（Q15） | `int16_t` | `[-1, 1)` | 2^-15 ≈ 3e-5 | M0 / M3（无 FPU） |
| `Q31` | `int32_t` | `[-1, 1)` | 2^-31 ≈ 4.7e-10 | M0 / M3（无 FPU，高精度） |

选择方式：编译参数加宏，**不定义时默认 `float`**。

```text
-DMCL_USE_Q15    → Q1.15 定点
-DMCL_USE_Q31    → Q31 定点
（不定义）        → float
```

---

## 2. 核心概念：归一化（per-unit）

这是定点最关键、也最容易错的一点。**Q15/Q31 范围都是 `[-1, 1)`，物理量必须先归一化才能用。**

归一化 = 选一个「基值」，把物理量除以基值，映射进 `[-1, 1)`：

```text
归一化值 = 物理量 / 基值
```

基值选择原则：**让归一化后所有取值都落在 `[-1, 1)` 内，且尽量接近 1**（越接近 1，定点精度越高，但也不能溢出）。

典型基值示例（以 20A 电流、60V 母线、8000rpm 电机为例）：

| 物理量 | 基值建议 | 说明 |
|---|---|---|
| 电流 | `I_BASE = 20A`（或额定 2 倍） | 保证峰值不溢出 |
| 电压 | `V_BASE = 60V`（或母线最大 1.5 倍） | 含反电动势峰值 |
| 转速 | `W_BASE = 额定最大` | 电角速度 |

> 例：3.5A 电流，基值 20A → 归一化 0.175。Q15 下存 `0.175 × 32768 = 5734`。

---

## 3. 转换宏与运算宏

mcl 提供了统一的宏，**float 模式下是恒等（零开销），定点模式下自动转换/校正**：

### 3.1 转换宏

```c
MCL_FROM_FLOAT(x)   /* float 物理量/标幺值 → mcl_scalar */
MCL_TO_FLOAT(x)     /* mcl_scalar → float */
```

| 模式 | `MCL_FROM_FLOAT(x)` | `MCL_TO_FLOAT(x)` |
|---|---|---|
| float | `(x)`（恒等） | `(x)`（恒等） |
| Q15 | `mcl_q15_from_float(x)` | `mcl_q15_to_float(x)` |
| Q31 | `mcl_q31_from_float(x)` | `mcl_q31_to_float(x)` |

### 3.2 运算宏

```c
MCL_MUL(a, b)   /* 乘法：float 直接乘；定点做移位校正 + 饱和 */
MCL_ADD(a, b)   /* 加法：float 直接加；定点做饱和加 */
MCL_SAT(x)      /* 饱和：float 恒等；定点钳位到 [-1,1) */
```

**为什么定点乘法要校正**：`Q15 × Q15 = Q30`，要右移 15 位回到 Q15；`Q31 × Q31 = Q62`，要右移 31 位。直接用 `*` 会得到错误量纲。

**算法层约定**：凡涉及乘法/加法的核心运算，用 `MCL_MUL` / `MCL_ADD`，不要直接用 `*` / `+`。这样切精度时运算语义自动正确。

---

## 4. 角度约定（重要）

定点下「弧度」会溢出（π ≈ 3.14 > 1），所以角度采用**归一化表示**：

| 模式 | 角度表示 | 一圈 |
|---|---|---|
| float | 弧度 | `2π ≈ 6.28` |
| Q15 / Q31 | 归一化角度 | `1.0` |

即定点下 `1.0 = 2π = 一圈`，`0.5 = π = 半圈`。

- `mcl_math_sin/cos` 输入：float 弧度；定点归一化角度 `[0, 1)`。
- `mcl_math_atan2` 输出：float 弧度 `[-π, π]`；定点归一化 `[-0.5, 0.5]`。

> 切换定点时，算法里所有「角度」变量的量纲要从弧度换成归一化角度。

---

## 5. 使用示例

### 5.1 在 HAL 层做采样归一化

```c
#define I_BASE 20.0f    /* 电流基值 20A */
#define V_BASE 60.0f    /* 电压基值 60V */

int adc_read_phase(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    float a = /* ADC 算出 A 相电流（安培） */;
    float b = /* ADC 算出 B 相电流（安培） */;

    /* 归一化到 [-1,1) 再转当前精度：
       float 模式：MCL_FROM_FLOAT 恒等，但除基值仍会执行 → 存标幺值
       定点模式：MCL_FROM_FLOAT 转 Q 格式 */
    *ia = MCL_FROM_FLOAT(a / I_BASE);
    *ib = MCL_FROM_FLOAT(b / I_BASE);
    *ic = MCL_FROM_FLOAT(-(a + b) / I_BASE);
    return MCL_OK;
}
```

### 5.2 填配置参数

```c
mcl_config cfg = { 0 };
cfg.pole_pairs = 7;

/* 保护阈值：物理量 → 归一化 → 当前精度 */
cfg.limits.overcurrent  = MCL_FROM_FLOAT(10.0f / I_BASE);   /* 10A */
cfg.limits.overvoltage  = MCL_FROM_FLOAT(63.0f / V_BASE);   /* 63V */

/* PID 参数：定点下量纲是「归一化」，通常也在 float 下先整定，
   再整体换成 MCL_FROM_FLOAT 包裹（量纲换算见 §6） */
cfg.current_pid.kp = MCL_FROM_FLOAT(0.8f);
cfg.current_pid.ki = MCL_FROM_FLOAT(0.05f);
```

### 5.3 读取遥测

```c
mcl_telemetry t;
mcl_get_telemetry(&motor, &t);

/* 反归一化回物理量给上层 */
float speed_rpm = MCL_TO_FLOAT(t.speed_rpm);   /* 若 speed 归一化则再乘基值 */
```

---

## 6. 从 float 切到定点的步骤

1. **编译加宏**：`-DMCL_USE_Q15`（或 `-DMCL_USE_Q31`）。
2. **定基值**：为电流、电压、转速各选一个基值（见 §2）。
3. **改 HAL 层**：采样回调里，物理量先除基值，再用 `MCL_FROM_FLOAT` 转精度。
4. **改配置**：`mcl_config` 里所有物理量参数（阈值、电机参数、PID）套上归一化 + `MCL_FROM_FLOAT`。
5. **改角度量纲**：算法里弧度 → 归一化角度（`MCL_INV_TWO_PI` 换算）。
6. **改运算**：核心乘法/加法换成 `MCL_MUL` / `MCL_ADD`（若之前用了裸 `*`/`+`）。
7. **验证**：先跑 float 确认逻辑，再切定点对比遥测数值，逐步排查溢出/精度。

> 注意：**这不是「改个宏就行」，而是每个精度都要重新做一遍归一化和调参**。抽象类型解决的是「让切换成为可能」，不解决「归一化」。

---

## 7. 常见坑

| 坑 | 原因 | 对策 |
|---|---|---|
| 切定点后数值全错 | 物理量没归一化，溢出成垃圾值 | 按 §6 逐项归一化 |
| 电流读到负数异常 | 基值选太小，峰值超出 `[-1,1)` | 基值留 1.5~2 倍裕量 |
| PID 整定好的参数切定点后振荡 | 定点下 PID 量纲变了 | 定点下重新整定，或先推导量纲换算 |
| 增益 >1 被饱和 | `ki/kp > 1` 存 Q15 会饱和到 ~1 | per-unit 下增益重新整定到 `<1` |
| 乘法结果偏差大 | 用了裸 `*` 没走 `MCL_MUL` | 核心运算统一用宏 |
| 角度错乱 | 弧度/归一化角度混用 | 明确角度约定（§4） |
| 精度不够 | Q15 只有 3e-5 | 改用 Q31 |
| **时间阈值提前触发** | dt 归一化后（dt_pu=dt/T_BASE），`stall_time`/`fault_stop_time` 仍按物理秒数存，导致 50 步就超时（如堵转保护误关断 PWM） | 时间类阈值随 time_base 一起归一化：`T_pu = T/T_BASE = T·W_BASE` |
| **基值 >1 溢出** | W_BASE/I_BASE/V_BASE 常 >1，直接 `MCL_FROM_FLOAT` 饱和 | 基值留在宿主 float 域；进定点用 `<1` 的倒数（如 `T_BASE=1/W_BASE`） |

---

## 8. 当前实现状态

| 项 | float | Q15 / Q31 |
|---|---|---|
| `mcl_scalar` 类型 | ✅ | ✅ |
| 转换宏 / 运算宏（含 `MCL_DIV`） | ✅（恒等） | ✅ |
| 数学层 `mcl_math` | ✅ 查表 | ✅ 基础实现（转 float，已验证） |
| 变换 / SVPWM / PID | ✅ | ✅（三维度仿真验证） |
| 观测器（flux/SMO/ORTEGA） | ✅ | ✅（per-unit 归一化，三精度相位误差与 float 同量级；flux ~4°、SMO ~4°、ORTEGA ~1°） |
| FOC 电流环闭环 | ✅ | ✅（per-unit 三精度一致精确收敛，Q31 无偏） |
| FOC 速度环闭环 + 机械方程 | ✅ | ✅（电机 float 物理 + HAL 边界归一化，三精度 ~500 rpm <1% 误差） |
| MTPA / 弱磁 | ✅ | ✅（float 中转，避开 8/4/√3 常数溢出，三精度一致；IPMSM 支持） |
| dt 归一化 | — | ✅（`cfg.time_base`，dt_pu = dt/T_BASE） |
| 速度环 rpm↔rad/s 换算 | ✅ | ⚠️ 归一化路径（`#if` 分离，speed/speed_ref 在边界归一化） |
| 时间阈值（stall_time 等）归一化 | — | ⚠️ 需随 time_base 归一化（见 §7 坑） |

> 定点数学层当前是「基础实现」（转 float 计算再转回），后续可优化为纯定点查表；
> 详见 tests/fixed_point_test.c、fixed_point_observer_test.c、fixed_point_foc_test.c。

---

## 9. 归一化基值约定（定点闭环补充）

per-unit 基值必须满足物理约束，否则电压方程在归一化后不成立：

```text
V_BASE = W_BASE · λ_BASE = R_BASE · I_BASE = L_BASE · W_BASE · I_BASE
```

且建议 **V_BASE = V_BUS（母线电压）**，使 SVPWM 的「母线归一化」与 per-unit 电压一致，
避免额外 scale 转换。

> 坑：基值本身（W_BASE、I_BASE、V_BASE）常 >1，**不能存进 Q15/Q31 的 `mcl_scalar`**。
> 时间归一化用 `T_BASE = 1/W_BASE`（通常 <1，可存定点），`dt_pu = dt/T_BASE`。

### 9.1 时间类阈值归一化

dt 归一化后（`dt_pu = dt/T_BASE`，典型 dt_pu≈0.01 而非 0.0001），
所有「以秒为单位、在算法里与 dt 比较/累加」的阈值也必须随 time_base 归一化，
否则会提前 `1/T_BASE` 倍触发：

| 字段 | float（物理秒） | 定点（per-unit 时间） |
|---|---|---|
| `limits.stall_time` | 0.5 s | 0.5/T_BASE = 0.5·W_BASE |
| `fault_stop_time` | 1.0 s | 1.0/T_BASE = 1.0·W_BASE |

> 典型踩坑：堵转保护。测试电机 speed=0（固定相位），若 `stall_time` 仍存物理 0.5 而
> `dt_pu=0.01`，则 `stall_timer += dt_pu` 每步 +0.01，500 步就超 0.5（而非 5000 步），
> 电机被误判堵转、PWM 被关断。

> 规避：物理量仍用 float 存于配置时，在 `mcl_init` 里统一乘 time_base 归一化；
> 或约定「配置里所有时间字段在定点模式下已是 per-unit 时间」。

---

## 10. 变更记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1.0 | 2026-09-05 | 初稿：三种精度、归一化、转换/运算宏、角度约定、切换步骤、常见坑 |
| v0.2.0 | 2026-09-06 | 定点化落地：MCL_DIV、观测器/变换/SVPWM/PID 向量化、dt 归一化 time_base、per-unit 闭环验证（三精度一致精确收敛），补 §8/§9 |
| v0.2.1 | 2026-09-06 | 补坑：增益 >1 饱和、基值 >1 溢出、时间阈值需随 time_base 归一化（§7/§9.1） |
| v0.3.0 | 2026-09-06 | 定点速度环闭环 + 机械方程验证通过（三精度 ~500 rpm），电机模型 float 物理 + HAL 边界归一化方案 |
| v0.3.1 | 2026-09-06 | SMO 完善：反电动势低通相位补偿 + seed 接口 + flux 参数 + sign/sat 定点修复；开环误差 7.4°→1.8° |
| v0.3.2 | 2026-09-06 | flux/SMO 定点 per-unit 验证补全（三观测器三精度一致，误差与 float 同量级） |
| v0.4.0 | 2026-09-06 | MTPA/弱磁定点化 + IPMSM 支持（config 加 ld_lq_diff，float 中转算 8/4/√3，三精度一致）；修 torque/telemetry/保护降额/开环斜坡 4 处裸除法 |
