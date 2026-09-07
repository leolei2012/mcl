/**
 * @file    mcl_observer_ortega.h
 * @brief   mcl 电机控制库：ORTEGA 型磁链观测器（带相位收敛的非线性观测器）
 *
 * 参考 Ortega et al. 的 PMSM 磁链观测器（VESC FOC_OBSERVER_ORTEGA_ORIGINAL）：
 *   1. 定子磁链积分：ψ_s += (v - R*i) * dt
 *   2. 转子磁链：λ_r = ψ_s - L*i
 *   3. 幅值误差反馈（非对称 clamp）：err = λ² - |λ_r|²，仅当 err<0（幅值偏大）
 *      时沿 λ_r 方向拉回，err>0 时置 0 让积分自然增长。
 *
 * 相比朴素磁链观测器（mcl_observer_flux），该非对称反馈保证估计磁链全局
 * 收敛到真实磁链（相位 + 幅值），对初始相位误差、参数误差更鲁棒。
 *
 * 用法同 mcl_observer_flux，在 mcl_init 注入 ops = &mcl_observer_ortega_ops。
 */

#ifndef MCL_OBSERVER_ORTEGA_H
#define MCL_OBSERVER_ORTEGA_H

#include "mcl_types.h"
#include "mcl_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ORTEGA 观测器参数
 */
typedef struct
{
    mcl_scalar lambda;               /**< 永磁磁链 Wb */
    mcl_scalar resistance;           /**< 相电阻 Ω */
    mcl_scalar inductance;           /**< 相电感 H */
    mcl_scalar gain;                 /**< 观测器增益 γ（越大收敛越快） */
} mcl_observer_ortega_params;

/**
 * @brief ORTEGA 观测器实例
 */
typedef struct
{
    mcl_observer_ortega_params params; /**< 参数 */
    mcl_scalar x1;                     /**< 定子磁链 α 分量估计 */
    mcl_scalar x2;                     /**< 定子磁链 β 分量估计 */
    mcl_scalar lambda_est;             /**< 转子磁链幅值估计 */
    mcl_scalar i_alpha_last;           /**< 上次 α 电流（seed 换算用） */
    mcl_scalar i_beta_last;            /**< 上次 β 电流（seed 换算用） */
} mcl_observer_ortega;

/**
 * @brief ORTEGA 观测器接口（注入 mcl_observer 载体用）
 */
extern const mcl_observer_ops mcl_observer_ortega_ops;

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_ORTEGA_H */
