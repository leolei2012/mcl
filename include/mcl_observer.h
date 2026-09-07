/**
 * @file    mcl_observer.h
 * @brief   mcl 电机控制库：观测器抽象接口（可插拔观测器载体）
 *
 * mcl 作为观测器载体，通过统一的 ops 接口接入不同观测器算法，
 * 便于实现、切换与性能对比。
 *
 * 实现一个观测器的步骤：
 *  1. 定义自己的实例结构体（含算法状态与参数）；
 *  2. 实现 mcl_observer_ops 的 init / reset / update（/ get_confidence）；
 *  3. 在 mcl_init 时把 ops + 实例 + 参数注入到 mcl 载体。
 *
 * 参考实现见 mcl_observer_flux.h（磁链观测器）。
 */

#ifndef MCL_OBSERVER_H
#define MCL_OBSERVER_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 观测器统一接口（所有观测器算法实现此接口）
 */
typedef struct
{
    /**
     * @brief 初始化观测器
     * @param impl   观测器实例（实现方自定义结构体）
     * @param params 观测器参数（实现方自定义结构体，可为 NULL）
     */
    void (*init)(void *impl, const void *params);

    /**
     * @brief 复位观测器内部状态
     * @param impl 观测器实例
     */
    void (*reset)(void *impl);

    /**
     * @brief 单步更新，估计转子电气角与速度
     * @param impl        观测器实例
     * @param v_alpha     α 轴电压 V
     * @param v_beta      β 轴电压 V
     * @param i_alpha     α 轴电流 A
     * @param i_beta      β 轴电流 A
     * @param dt          控制周期 s
     * @param phase_rad   估计电气角 rad（输出，归一化到 [-π, π]）
     * @param speed_rad_s 估计速度 rad/s（输出，可为 NULL）
     */
    void (*update)(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                   mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                   mcl_scalar *phase_rad, mcl_scalar *speed_rad_s);

    /**
     * @brief 预置观测器内部磁链状态（可选，NULL 表示不支持）
     *
     * 用于无感开环→闭环平滑切换：开环拖动期间把观测器内部磁链状态
     * 预置到当前开环相位对应的转子磁链矢量，切闭环瞬间观测器从正确
     * 初值开始，避免跳变 / 失步（VESC 式做法）。
     *
     * @param impl        观测器实例
     * @param flux_alpha  转子磁链 α 分量 Wb
     * @param flux_beta   转子磁链 β 分量 Wb
     */
    void (*seed)(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta);

    /**
     * @brief 收敛可信度（可选，NULL 表示不支持）
     * @param impl 观测器实例
     * @return 0~1，越接近 1 表示估计越可靠
     */
    mcl_scalar (*get_confidence)(void *impl);
} mcl_observer_ops;

/**
 * @brief 观测器载体（mcl 持有，绑定具体算法）
 */
typedef struct
{
    const mcl_observer_ops *ops;    /**< 算法接口 */
    void                   *impl;   /**< 算法私有实例 */
    void                   *params; /**< 算法私有参数 */
} mcl_observer;

/**
 * @brief 绑定观测器载体
 * @param self   载体
 * @param ops    算法接口（不可为 NULL）
 * @param impl   算法实例（实现方分配）
 * @param params 算法参数（可为 NULL）
 */
void mcl_observer_init(mcl_observer *self, const mcl_observer_ops *ops,
                       void *impl, void *params);

/**
 * @brief 复位观测器
 * @param self 载体
 */
void mcl_observer_reset(mcl_observer *self);

/**
 * @brief 观测器单步更新
 * @param self        载体
 * @param v_alpha     α 轴电压 V
 * @param v_beta      β 轴电压 V
 * @param i_alpha     α 轴电流 A
 * @param i_beta      β 轴电流 A
 * @param dt          控制周期 s
 * @param phase_rad   估计电气角 rad（输出）
 * @param speed_rad_s 估计速度 rad/s（输出，可为 NULL）
 */
void mcl_observer_update(mcl_observer *self, mcl_scalar v_alpha, mcl_scalar v_beta,
                         mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                         mcl_scalar *phase_rad, mcl_scalar *speed_rad_s);

/**
 * @brief 获取观测器收敛可信度
 * @param self 载体
 * @return 0~1；接口不支持时返回 1.0f
 */
mcl_scalar mcl_observer_get_confidence(mcl_observer *self);

/**
 * @brief 预置观测器内部磁链状态（开环→闭环切换用）
 * @param self        载体
 * @param flux_alpha  转子磁链 α 分量 Wb
 * @param flux_beta   转子磁链 β 分量 Wb
 */
void mcl_observer_seed(mcl_observer *self, mcl_scalar flux_alpha, mcl_scalar flux_beta);

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_H */
