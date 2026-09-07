/**
 * @file    mcl_observer_flux.h
 * @brief   mcl 电机控制库：磁链观测器（参考实现，观测器插槽模板）
 *
 * 基于电机电压方程的磁链观测器：估计转子磁链矢量（α/β 分量），
 * 由磁链矢量求相位。可作为实现新观测器时的模板参考。
 *
 * 用法：
 *   mcl_observer_flux obs;
 *   mcl_observer_flux_params params = { .lambda = 0.02f, .gain = 100.0f };
 *   // 在 mcl_init 时注入：ops = &mcl_observer_flux_ops, impl = &obs, params = &params
 */

#ifndef MCL_OBSERVER_FLUX_H
#define MCL_OBSERVER_FLUX_H

#include "mcl_types.h"
#include "mcl_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 磁链观测器参数
 */
typedef struct
{
    mcl_scalar lambda;               /**< 永磁磁链 Wb */
    mcl_scalar resistance;           /**< 相电阻 Ω */
    mcl_scalar inductance;           /**< 相电感 H */
    mcl_scalar gain;                 /**< 观测器增益（越大收敛越快） */
} mcl_observer_flux_params;

/**
 * @brief 磁链观测器实例
 */
typedef struct
{
    mcl_observer_flux_params params; /**< 参数 */
    mcl_scalar x1;                        /**< 磁链 α 分量估计 */
    mcl_scalar x2;                        /**< 磁链 β 分量估计 */
    mcl_scalar lambda_est;                /**< 磁链幅值估计 */
    mcl_scalar i_alpha_last;              /**< 上次 α 电流（离散化用） */
    mcl_scalar i_beta_last;               /**< 上次 β 电流（离散化用） */
} mcl_observer_flux;

/**
 * @brief 磁链观测器接口（注入 mcl_observer 载体用）
 */
extern const mcl_observer_ops mcl_observer_flux_ops;

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_FLUX_H */
