/**
 * @file    mcl_math.h
 * @brief   mcl 电机控制库：数学库抽象（可插拔 + 精度跟随 mcl_scalar）
 *
 * 电机控制与观测器高频使用三角函数 / 开方；不同芯片可能提供硬件数学 API
 * （CORDIC、CMSIS-DSP、厂商数学库）。本模块接口统一用 mcl_scalar，
 * 随编译精度（float / Q15 / Q31）自动切换。
 *
 * 实现选择（编译期宏，多选一，未定义时用默认查表）：
 *   MCL_MATH_USE_CUSTOM  用户自定义实现（extern，映射到芯片 API）
 *   MCL_MATH_USE_CMSIS   CMSIS-DSP（arm_sin_f32 等）
 *   MCL_MATH_USE_LIBM    标准 C 数学库 math.h（sinf 等）
 *   （默认）             内置查表 + 插值（自包含，ARM 通用）
 *
 * 角度约定（重要）：
 *   - float 模式：弧度，范围 [0, 2π)
 *   - Q15/Q31 模式：归一化角度，范围 [0, 1)，即 1.0 = 2π = 一圈
 *   （详见 docs/spec/mcl_fixed_point.md）
 */

#ifndef MCL_MATH_H
#define MCL_MATH_H

#include "mcl_types.h"

/* ============================ 常量（弧度，float 模式用） ============================ */

#define MCL_PI         3.14159265358979323846f
#define MCL_PI_2       1.57079632679489661923f
#define MCL_PI_4       0.78539816339744830962f
#define MCL_3PI_4      2.35619449019234492885f
#define MCL_TWO_PI     6.28318530717958647692f
#define MCL_INV_TWO_PI 0.15915494309189533577f

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 正弦
 * @param x 角度（float：弧度；定点：归一化 [0,1) = [0,2π)）
 * @return sin(x)
 */
mcl_scalar mcl_math_sin(mcl_scalar x);

/**
 * @brief 余弦
 * @param x 角度（float：弧度；定点：归一化）
 * @return cos(x)
 */
mcl_scalar mcl_math_cos(mcl_scalar x);

/**
 * @brief 同时计算正弦与余弦（省一次查表/调用）
 * @param x   角度（float：弧度；定点：归一化）
 * @param sin sin(x)（输出）
 * @param cos cos(x)（输出）
 */
void mcl_math_sincos(mcl_scalar x, mcl_scalar *sin, mcl_scalar *cos);

/**
 * @brief 反正切（全象限）
 * @param y 对边
 * @param x 邻边
 * @return 角度（float：[-π, π]；定点：归一化 [-0.5, 0.5]）
 */
mcl_scalar mcl_math_atan2(mcl_scalar y, mcl_scalar x);

/**
 * @brief 快速反正切近似（观测器性能对比用，精度略低于 atan2）
 * @param y 对边
 * @param x 邻边
 * @return 角度（同 atan2）
 */
mcl_scalar mcl_math_fast_atan2(mcl_scalar y, mcl_scalar x);

/**
 * @brief 平方根
 * @param x 非负数
 * @return sqrt(x)
 */
mcl_scalar mcl_math_sqrt(mcl_scalar x);

#ifdef __cplusplus
}
#endif

#endif /* MCL_MATH_H */
