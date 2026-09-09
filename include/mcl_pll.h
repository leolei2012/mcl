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
    mcl_scalar kp;                   /**< 相位锁定比例增益（VESC 式：单位 1/s，作用于相位积分，
                                          加到速度项上；定点为 (电气速度 pu)/圈） */
    mcl_scalar ki;                   /**< 速度积分增益（单位 1/s²；定点为 (电气速度 pu)/(圈·dt_pu)） */
    mcl_scalar phase;                /**< 跟踪相位：float=rad，定点=归一化圈 [0,1) */
    mcl_scalar speed;                /**< 估计速度：float=rad/s，定点=电气速度 pu(=ω/W_BASE)。
                                          VESC 式下本字段即积分项，直接累加 ki·err·dt */
    mcl_scalar last_phase;           /**< 上一拍输入相位（相位差分数度估计 + wind-up 限幅用） */
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
 * @brief PLL 单步更新（VESC 式：kp 作用于相位积分，ki 作用于速度积分）
 * @param self      PLL 实例
 * @param phase     输入相位（来自观测器 / 编码器）：float=rad，定点=「圈」
 * @param dt        控制周期：float=秒，定点=dt_pu(=dt/T_BASE)
 * @param phase_out 平滑相位（输出，量纲同输入）
 * @param speed_out 估计速度（输出）：float=rad/s，定点=电气速度 pu
 *
 * 算法（对齐 VESC foc_pll_run）：
 *   phase += (speed + kp·err)·dt
 *   speed += ki·err·dt
 */
void mcl_pll_run(mcl_pll *self, mcl_scalar phase, mcl_scalar dt,
                 mcl_scalar *phase_out, mcl_scalar *speed_out);

#ifdef __cplusplus
}
#endif

#endif /* MCL_PLL_H */
