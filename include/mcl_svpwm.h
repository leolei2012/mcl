/**
 * @file    mcl_svpwm.h
 * @brief   mcl 电机控制库：空间矢量 PWM 调制
 */

#ifndef MCL_SVPWM_H
#define MCL_SVPWM_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SVPWM 调制：两相静止电压 → 三相占空比
 *
 * 采用七段式（含零矢量分配）SVPWM，等效于注入三次谐波的载波调制。
 * 当电压矢量超出线性区时进行过调制处理（限制在六边形内切圆边界）。
 *
 * @param v_alpha α 轴电压
 * @param v_beta  β 轴电压
 * @param max_duty 最大占空比（0~1，通常对应调制比上限）
 * @param da A 相占空比（输出，0~1）
 * @param db B 相占空比（输出，0~1）
 * @param dc C 相占空比（输出，0~1）
 */
void mcl_svpwm_run(mcl_scalar v_alpha, mcl_scalar v_beta, mcl_scalar max_duty,
                   mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc);

#ifdef __cplusplus
}
#endif

#endif /* MCL_SVPWM_H */
