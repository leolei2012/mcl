/**
 * @file    mcl_pll.h
 * @brief   mcl 电机控制库：PLL 锁相环（相位跟踪 + 速度估计）
 *
 * 对（观测器或编码器给出的）相位进行跟踪与滤波，输出平滑相位与速度。
 * 与观测器解耦，可独立替换（如换用滑模观测器时 PLL 保持不变）。
 */

#ifndef MCL_PLL_H
#define MCL_PLL_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PLL 运行时状态
 */
typedef struct
{
    mcl_scalar kp;                   /**< 比例系数 */
    mcl_scalar ki;                   /**< 积分系数 */
    mcl_scalar phase;                /**< 跟踪相位 rad */
    mcl_scalar speed;                /**< 估计速度 rad/s */
    mcl_scalar i_term;               /**< 积分项 */
} mcl_pll;

/**
 * @brief 初始化 PLL
 * @param self PLL 实例
 * @param kp   比例系数
 * @param ki   积分系数
 */
void mcl_pll_init(mcl_pll *self, mcl_scalar kp, mcl_scalar ki);

/**
 * @brief 复位 PLL
 * @param self PLL 实例
 */
void mcl_pll_reset(mcl_pll *self);

/**
 * @brief PLL 单步更新
 * @param self      PLL 实例
 * @param phase     输入相位 rad（来自观测器 / 编码器）
 * @param dt        控制周期 s
 * @param phase_out 平滑相位 rad（输出）
 * @param speed_out 估计速度 rad/s（输出）
 */
void mcl_pll_run(mcl_pll *self, mcl_scalar phase, mcl_scalar dt,
                 mcl_scalar *phase_out, mcl_scalar *speed_out);

#ifdef __cplusplus
}
#endif

#endif /* MCL_PLL_H */
