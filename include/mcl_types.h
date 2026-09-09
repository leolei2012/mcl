/**
 * @file    mcl_types.h
 * @brief   mcl 电机控制库：公共类型、枚举、版本宏与集中配置
 */

#ifndef MCL_TYPES_H
#define MCL_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 版本宏 ============================ */

#define MCL_VERSION_MAJOR    0u
#define MCL_VERSION_MINOR    1u
#define MCL_VERSION_PATCH    0u
#define MCL_VERSION_STRING   "0.1.0"
#define MCL_VERSION_NUM      ((MCL_VERSION_MAJOR << 16) | (MCL_VERSION_MINOR << 8) | MCL_VERSION_PATCH)

/* ============================ 功能裁剪（编译期，默认全开） ============================ */

/**
 * mcl 支持按功能裁剪，用编译参数定义下列宏即可去掉不需要的模块，
 * 节省 Flash / RAM 并避免编译无用代码。不定义任何宏 = 全功能（向后兼容）。
 *
 *   #define MCL_DISABLE_BLDC          裁掉六步方波换相（含 mcl_bldc_comm 与相关 mode）
 *   #define MCL_DISABLE_OBSERVER      裁掉无感观测器（无感 FOC 不可用）
 *   #define MCL_DISABLE_POSITION      裁掉位置环
 *   #define MCL_DISABLE_CALIBRATION   裁掉校准（编码器对齐/电阻/电感/相序/磁链测量）
 *   #define MCL_DISABLE_OPENLOOP      裁掉开环（VF/IF/ALIGN + 自动开环切闭环）
 *
 * 裁剪后：对应子模块结构体成员、public API、实现代码一并移除；
 * 若宿主仍调用被裁的 API（如 mcl_set_mode(BLDC)），编译期报错提醒。
 * 示例：gcc -DMCL_DISABLE_BLDC ...（不需要六步换相时）
 */

/* ============================ 标量类型与精度 ============================ */

/**
 * 数值精度（编译期选择，未定义时默认 float）：
 *   MCL_USE_Q15   Q1.15 定点（int16_t，范围 [-1, 1)，LSB = 2^-15）
 *   MCL_USE_Q31   Q31 定点（int32_t，范围 [-1, 1)，LSB = 2^-31）
 *   （默认）       float 单精度浮点
 *
 * 注意：定点范围受限于 [-1, 1)，物理量必须先归一化（per-unit），
 * 转换与运算请使用 MCL_FROM_FLOAT / MCL_TO_FLOAT / MCL_MUL / MCL_ADD / MCL_SAT。
 * 详见 docs/spec/mcl_fixed_point.md。
 */

#if defined(MCL_USE_Q15)
    typedef int16_t mcl_scalar;
#elif defined(MCL_USE_Q31)
    typedef int32_t mcl_scalar;
#else
    typedef float   mcl_scalar;
#endif

/* ---- 定点 inline 实现（仅定点模式编译） ---- */

#if defined(MCL_USE_Q15)

static inline int16_t mcl_q15_sat(int32_t x)
{
    if (x > 32767) { return 32767; }
    if (x < -32768) { return -32768; }
    return (int16_t)x;
}

static inline int16_t mcl_q15_from_float(float x)
{
    float v = x * 32768.0f;
    if (v > 32767.0f) { v = 32767.0f; }
    if (v < -32768.0f) { v = -32768.0f; }
    return (int16_t)v;
}

static inline float mcl_q15_to_float(int16_t x)
{
    return (float)x / 32768.0f;
}

static inline int16_t mcl_q15_mul(int16_t a, int16_t b)
{
    int32_t t = (int32_t)a * b;
    t = (t + 0x4000) >> 15;
    return mcl_q15_sat(t);
}

static inline int16_t mcl_q15_add(int16_t a, int16_t b)
{
    return mcl_q15_sat((int32_t)a + b);
}

static inline int16_t mcl_q15_sub(int16_t a, int16_t b)
{
    return mcl_q15_sat((int32_t)a - b);
}

static inline int16_t mcl_q15_neg(int16_t x)
{
    return mcl_q15_sat(-(int32_t)x);
}

static inline int16_t mcl_q15_abs(int16_t x)
{
    return (x < 0) ? mcl_q15_neg(x) : x;
}

static inline int16_t mcl_q15_div(int16_t a, int16_t b)
{
    int32_t t;

    if (b == 0)
    {
        return (a >= 0) ? 32767 : -32768;
    }

    /* Q15/Q15 = Q0，左移 15 位转回 Q15；|a|≤32768 时中间量不溢出 int32 */
    t = ((int32_t)a << 15) / b;
    return mcl_q15_sat(t);
}

#elif defined(MCL_USE_Q31)

static inline int32_t mcl_q31_sat(int64_t x)
{
    if (x > 2147483647LL) { return 2147483647; }
    if (x < -2147483648LL) { return -2147483648; }
    return (int32_t)x;
}

static inline int32_t mcl_q31_from_float(float x)
{
    double v = (double)x * 2147483648.0;
    if (v > 2147483647.0) { v = 2147483647.0; }
    if (v < -2147483648.0) { v = -2147483648.0; }
    return (int32_t)v;
}

static inline float mcl_q31_to_float(int32_t x)
{
    return (float)x / 2147483648.0f;
}

static inline int32_t mcl_q31_mul(int32_t a, int32_t b)
{
    int64_t t = (int64_t)a * b;
    t = (t + 0x40000000LL) >> 31;
    return mcl_q31_sat(t);
}

static inline int32_t mcl_q31_add(int32_t a, int32_t b)
{
    return mcl_q31_sat((int64_t)a + b);
}

static inline int32_t mcl_q31_sub(int32_t a, int32_t b)
{
    return mcl_q31_sat((int64_t)a - b);
}

static inline int32_t mcl_q31_neg(int32_t x)
{
    return mcl_q31_sat(-(int64_t)x);
}

static inline int32_t mcl_q31_abs(int32_t x)
{
    return (x < 0) ? mcl_q31_neg(x) : x;
}

static inline int32_t mcl_q31_div(int32_t a, int32_t b)
{
    int64_t t;

    if (b == 0)
    {
        return (a >= 0) ? 2147483647 : -2147483648;
    }

    /* Q31/Q31 = Q0，左移 31 位转回 Q31；用 int64 中间量防溢出 */
    t = ((int64_t)a << 31) / b;
    return mcl_q31_sat(t);
}

#endif

/* ---- 统一转换宏（float 模式恒等，定点模式转换） ---- */

#if defined(MCL_USE_Q15)
    #define MCL_FROM_FLOAT(x)  mcl_q15_from_float(x)
    #define MCL_TO_FLOAT(x)    mcl_q15_to_float(x)
#elif defined(MCL_USE_Q31)
    #define MCL_FROM_FLOAT(x)  mcl_q31_from_float(x)
    #define MCL_TO_FLOAT(x)    mcl_q31_to_float(x)
#else
    #define MCL_FROM_FLOAT(x)  (x)
    #define MCL_TO_FLOAT(x)    (x)
#endif

/* ---- 统一运算宏（float 直接运算，定点校正/饱和） ---- */

#if defined(MCL_USE_Q15)
    #define MCL_MUL(a, b)  mcl_q15_mul((a), (b))
    #define MCL_ADD(a, b)  mcl_q15_add((a), (b))
    #define MCL_SUB(a, b)  mcl_q15_sub((a), (b))
    #define MCL_NEG(x)     mcl_q15_neg(x)
    #define MCL_ABS(x)     mcl_q15_abs(x)
    #define MCL_SAT(x)     mcl_q15_sat((x))
    #define MCL_DIV(a, b)  mcl_q15_div((a), (b))
#elif defined(MCL_USE_Q31)
    #define MCL_MUL(a, b)  mcl_q31_mul((a), (b))
    #define MCL_ADD(a, b)  mcl_q31_add((a), (b))
    #define MCL_SUB(a, b)  mcl_q31_sub((a), (b))
    #define MCL_NEG(x)     mcl_q31_neg(x)
    #define MCL_ABS(x)     mcl_q31_abs(x)
    #define MCL_SAT(x)     mcl_q31_sat((x))
    #define MCL_DIV(a, b)  mcl_q31_div((a), (b))
#else
    #define MCL_MUL(a, b)  ((a) * (b))
    #define MCL_ADD(a, b)  ((a) + (b))
    #define MCL_SUB(a, b)  ((a) - (b))
    #define MCL_NEG(x)     (-(x))
    #define MCL_ABS(x)     ((x) < 0 ? -(x) : (x))
    #define MCL_SAT(x)     (x)
    #define MCL_DIV(a, b)  ((a) / (b))
#endif

/* ============================ 运行模式 ============================ */

typedef enum
{
    MCL_MODE_FOC_SENSORED = 0,  /**< FOC 有感 */
    MCL_MODE_FOC_SENSORLESS,    /**< FOC 无感 */
    MCL_MODE_BLDC_HALL,         /**< 六步方波（霍尔换相） */
    MCL_MODE_BLDC_SENSORLESS,   /**< 六步方波（无感 BEMF） */
} mcl_mode;

/* ============================ 运行状态 ============================ */

typedef enum
{
    MCL_STATE_IDLE = 0,         /**< 未启动 */
    MCL_STATE_ALIGN,            /**< 对齐 / 校准 */
    MCL_STATE_RUN,              /**< 正常运行 */
    MCL_STATE_FAULT,            /**< 故障 */
} mcl_state;

/* ============================ 故障码 ============================ */

typedef enum
{
    MCL_FAULT_NONE = 0,
    MCL_FAULT_OVERCURRENT,      /**< 过流 */
    MCL_FAULT_OVERVOLTAGE,      /**< 过压 */
    MCL_FAULT_UNDERVOLTAGE,     /**< 欠压 */
    MCL_FAULT_OVERTEMP,         /**< 过温 */
    MCL_FAULT_STALL,            /**< 堵转 */
    MCL_FAULT_SYNC_LOST,        /**< 失步 */
    MCL_FAULT_DRV,              /**< 门驱故障（nFAULT 引脚） */
} mcl_fault;

/* ============================ 反馈类型 ============================ */

typedef enum
{
    MCL_FEEDBACK_NONE = 0,      /**< 无反馈（无感） */
    MCL_FEEDBACK_ENCODER,       /**< ABI / SPI 编码器 */
    MCL_FEEDBACK_HALL,          /**< 霍尔 */
} mcl_feedback_type;

/* ============================ 错误码 ============================ */

typedef enum
{
    MCL_OK = 0,
    MCL_ERR_PARAM,              /**< 参数非法 */
    MCL_ERR_STATE,              /**< 状态不允许 */
    MCL_ERR_HAL,                /**< HAL 返回错误 */
    MCL_ERR_BUSY,               /**< 忙（校准进行中） */
} mcl_err;

/* ============================ 控制模式 ============================ */

typedef enum
{
    MCL_CTRL_CURRENT = 0,       /**< 电流环（直接设定 Iq） */
    MCL_CTRL_SPEED,             /**< 速度环（级联于电流环） */
    MCL_CTRL_POSITION,          /**< 位置环（级联于速度环） */
    MCL_CTRL_OPENLOOP_VF,       /**< 开环旋转电压矢量（V/F，全开环） */
    MCL_CTRL_OPENLOOP_IF,       /**< 开环旋转电流矢量（I/F，电流环闭环、相位开环） */
    MCL_CTRL_OPENLOOP_ALIGN,    /**< 开环固定电流矢量（转子预定位） */
} mcl_ctrl_mode;

/* ============================ PID 参数 ============================ */

typedef struct
{
    mcl_scalar kp;              /**< 比例系数 */
    mcl_scalar ki;              /**< 积分系数 */
    mcl_scalar kd;              /**< 微分系数 */
    mcl_scalar out_min;         /**< 输出下限 */
    mcl_scalar out_max;         /**< 输出上限 */
    mcl_scalar i_min;           /**< 积分项下限 */
    mcl_scalar i_max;           /**< 积分项上限 */
} mcl_pid_params;

/* ============================ 保护阈值 ============================ */

/* 保护使能位（mcl_protection_limits.enabled 位掩码，置位 = 启用） */
#define MCL_PROTECT_OVERCURRENT    (1u << 0)  /**< 过流 */
#define MCL_PROTECT_OVERVOLTAGE    (1u << 1)  /**< 过压 */
#define MCL_PROTECT_UNDERVOLTAGE   (1u << 2)  /**< 欠压 */
#define MCL_PROTECT_OVERTEMP       (1u << 3)  /**< 过温（含降额） */
#define MCL_PROTECT_STALL          (1u << 4)  /**< 堵转 */
#define MCL_PROTECT_ALL            (MCL_PROTECT_OVERCURRENT | MCL_PROTECT_OVERVOLTAGE | \
                                    MCL_PROTECT_UNDERVOLTAGE | MCL_PROTECT_OVERTEMP | \
                                    MCL_PROTECT_STALL)

typedef struct
{
    uint32_t   enabled;            /**< 保护使能位掩码（见 MCL_PROTECT_*） */
    mcl_scalar overcurrent;        /**< 过流阈值 A */
    mcl_scalar overvoltage;        /**< 过压阈值 V */
    mcl_scalar undervoltage;       /**< 欠压阈值 V */
    mcl_scalar temp_derate_start;  /**< 温度降额起始 ℃（低于此不降额） */
    mcl_scalar overtemp;           /**< 过温关断阈值 ℃ */
    mcl_scalar stall_speed;        /**< 堵转判定转速 rad/s */
    mcl_scalar stall_time;         /**< 堵转判定时间 s */
} mcl_protection_limits;

/* ============================ 反馈配置 ============================ */

typedef struct
{
    mcl_feedback_type type;     /**< 反馈类型 */
    mcl_scalar encoder_offset;  /**< 编码器电零位 rad */
    uint32_t   encoder_cpr;     /**< 编码器线数（ABI，每转计数） */
} mcl_feedback_cfg;

/* ============================ 遥测 ============================ */

typedef struct
{
    mcl_scalar speed_rpm;       /**< 转速 rpm */
    mcl_scalar position_rad;    /**< 位置 rad */
    mcl_scalar iq;              /**< q 轴电流 A */
    mcl_scalar id;              /**< d 轴电流 A */
    mcl_scalar vbus;            /**< 母线电压 V */
    mcl_scalar ibus;            /**< 母线电流 A */
    mcl_scalar duty;            /**< 当前占空比 */
    mcl_scalar temp_motor;      /**< 电机温度 ℃（宿主填入） */
    mcl_scalar temp_fet;        /**< 功率级温度 ℃（宿主填入） */
    mcl_scalar est_phase;       /**< 估计电气角 rad（无感） */
    mcl_scalar est_speed_rad_s; /**< 估计速度 rad/s（无感） */
} mcl_telemetry;

/* ============================ 故障现场快照 ============================ */

typedef struct
{
    mcl_fault  fault;           /**< 故障码 */
    mcl_scalar current;         /**< 故障时电流 A */
    mcl_scalar voltage;         /**< 故障时母线电压 V */
    mcl_scalar speed;           /**< 故障时转速 rad/s */
    mcl_scalar temp;            /**< 故障时温度 ℃ */
    uint32_t   tick;            /**< 故障时控制周期计数 */
} mcl_fault_info;

#ifdef __cplusplus
}
#endif

#endif /* MCL_TYPES_H */
