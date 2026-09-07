/**
 * @file    mcl_observer_smo.h
 * @brief   mcl 电机控制库：滑模观测器（SMO）
 *
 * 基于电流误差的滑模观测器，估计反电动势，再从中提取转子相位
 * （含反电动势低通滤波的相位补偿，见 mcl_observer_smo.c 头注释）。
 * 与磁链观测器（mcl_observer_flux / mcl_observer_ortega）实现同一
 * mcl_observer_ops 接口，可注入 mcl 载体对比性能。
 */

#ifndef MCL_OBSERVER_SMO_H
#define MCL_OBSERVER_SMO_H

#include "mcl_types.h"
#include "mcl_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 滑模观测器参数
 */
typedef struct
{
    mcl_scalar resistance;      /**< 相电阻 Ω */
    mcl_scalar inductance;      /**< 相电感 H */
    mcl_scalar flux;            /**< 永磁磁链 ψ_f Wb（速度估计 + 相位补偿用） */
    mcl_scalar gain;            /**< 滑模增益（需大于反电动势幅值 ω·λ） */
    mcl_scalar lpf;             /**< 反电动势低通滤波系数（越小过滤越平滑但相位延迟越大） */
    mcl_scalar boundary;        /**< 边界层厚度（饱和函数，抑制抖振） */
} mcl_observer_smo_params;

/**
 * @brief 滑模观测器实例
 */
typedef struct
{
    mcl_observer_smo_params params; /**< 参数 */
    mcl_scalar i_alpha_hat;         /**< 估计电流 α A */
    mcl_scalar i_beta_hat;          /**< 估计电流 β A */
    mcl_scalar e_alpha;             /**< 估计反电动势 α（滤波后） */
    mcl_scalar e_beta;              /**< 估计反电动势 β（滤波后） */
} mcl_observer_smo;

/**
 * @brief 滑模观测器接口（注入 mcl_observer 载体用）
 */
extern const mcl_observer_ops mcl_observer_smo_ops;

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_SMO_H */
