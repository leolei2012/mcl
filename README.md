# mcl — Motor Control Library

可复用的嵌入式电机控制库，目标平台为 ARM Cortex-M 系列（M0/M3/M4/M7）。支持 FOC（有感/无感）与六步方波 BLDC，数值精度可编译期切换（float / Q1.15 / Q31），观测器与数学库均为可插拔设计。

> 本仓库即 mcl 库本身，不包含具体产品应用。

## 特性

- **FOC 矢量控制**：有感（编码器/霍尔）与无感（磁链观测器 / 滑模观测器）
- **六步方波 BLDC**：霍尔换相与无感 BEMF 换相
- **完整控制环**：电流环、速度环、位置环、MTPA/弱磁
- **三种数值精度**：`float` / `Q1.15` / `Q31`（编译期切换）
- **可插拔观测器**：统一 `mcl_observer_ops` 接口，磁链观测器 + 滑模观测器已实现
- **可插拔数学库**：默认查表，可切换标准 libm / CMSIS-DSP / 芯片硬件 API
- **HAL 抽象**：硬件经 `mcl_hal_ops` 注入，库不碰寄存器、不注册中断
- **PC 仿真测试**：无需硬件即可验证算法与闭环

## 目录结构

```text
include/                 公开头文件（平铺，库前缀 mcl_）
├── mcl.h                门面总头（宿主唯一入口）
├── mcl_types.h          类型、枚举、集中配置 mcl_config、标量类型 mcl_scalar
├── mcl_math.h           数学库抽象（可插拔）
├── mcl_hal.h            HAL 抽象接口
├── mcl_transform.h      Clark / Park 变换
├── mcl_pid.h            通用 PID
├── mcl_svpwm.h          SVPWM 调制
├── mcl_observer.h       观测器载体（可插拔 ops 接口）
├── mcl_observer_flux.h  磁链观测器（参考实现）
├── mcl_observer_smo.h   滑模观测器（参考实现）
├── mcl_pll.h            PLL 锁相环
├── mcl_foc.h            FOC 电流环编排
├── mcl_mtpa_fw.h        MTPA + 弱磁
├── mcl_bldc_comm.h      六步换相
├── mcl_protection.h     保护检测
└── mcl_calibration.h    校准

src/                     实现（与 include 一一对应）

docs/
├── 规范/                工程规范（架构、编码、子模块）
└── spec/                规格书、架构设计、定点说明

tests/
├── sim_test.c           PC 仿真测试（6 场景）
├── build.ps1            构建脚本
└── sim_output.html      波形输出（构建后生成）
```

## 快速开始

### 环境

- gcc（MinGW，`C:\mingw64\bin` 已加入 PATH）

### 构建并运行仿真

```powershell
pwsh -File tests\build.ps1
```

仿真覆盖 6 个场景：Clark/Park 变换、PID 阶跃、SVPWM 波形、PLL 跟踪、多观测器对比、FOC 无感闭环。波形输出到 `tests/sim_output.html`，浏览器打开查看。

### 集成到你的工程

```c
#include "mcl.h"
#include "mcl_observer_flux.h"

static mcl s_motor;
static mcl_observer_flux s_obs;

void app_init(void)
{
    mcl_config cfg = { /* 电机参数、PID、保护阈值… */ };
    mcl_observer_flux_params op = { /* λ、R、L、gain */ };

    /* 宿主实现 mcl_hal_ops（PWM 输出、电流采样、编码器） */
    static const mcl_hal_ops hal = { .pwm_set_duty = ..., .adc_read_phase = ... };

    mcl_init(&s_motor, &cfg, &hal, hal_ctx, &mcl_observer_flux_ops, &s_obs, &op);
    mcl_set_mode(&s_motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_current(&s_motor, 1.0f);
    mcl_start(&s_motor);
}

/* 在电流环中断（如 PWM 更新中断）内调用 */
void pwm_isr(void)
{
    mcl_control_tick(&s_motor);
}
```

## 三种数值精度

编译期宏切换（默认 `float`）：

```text
-DMCL_USE_Q15    Q1.15 定点（int16_t）
-DMCL_USE_Q31    Q31 定点（int32_t）
（不定义）        float 单精度浮点
```

- 类型：`mcl_scalar`（自动对应三种精度）
- 运算：`MCL_MUL` / `MCL_ADD` / `MCL_SUB` / `MCL_SAT`（定点自动做移位校正与饱和）
- 转换：`MCL_FROM_FLOAT` / `MCL_TO_FLOAT`

> ⚠️ 定点范围 `[-1, 1)`，物理量必须先归一化（per-unit）。详见 `docs/spec/mcl_fixed_point.md`。

## 文档索引

| 文档 | 内容 |
|---|---|
| `docs/spec/mcl_spec.md` | 规格书：定位、功能需求、API、性能指标 |
| `docs/spec/mcl_architecture.md` | 代码架构设计：分层、模块 DAG、对象模型 |
| `docs/spec/mcl_fixed_point.md` | 定点使用说明：归一化、转换宏、切换步骤 |
| `docs/规范/` | 工程规范：架构、编码、子模块 |

## 当前状态

- 控制闭环完整，float 精度通过 PC 仿真验证
- Q15/Q31 定点：类型/运算宏/数学层骨架就绪，算法层定点迁移为后续工作（代码内 `TODO` 标注）
- 参考：`reference/bldc/`（VESC 固件）

## License

待定（尚未指定）。
