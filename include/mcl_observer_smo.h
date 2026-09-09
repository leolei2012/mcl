/**
 * @file    mcl_observer_smo.h
 * @brief   mcl 电机控制库：滑模观测器（SMO，AN1078 式）
 *
 * 基于电流误差的滑模观测器，估计反电动势，再从中提取转子相位与速度。
 * 结构对齐 Microchip AN1078（Sensorless FOC of PMSM Using SMO，smcpos.c）：
 *   1. 滑模电流观测器：î += G·(v − e − z) + F·î  （G=Ts/L，F=1−R·Ts/L）
 *   2. 滑模控制 z = Kslide·sat(i_err / MaxSMCError)
 *   3. 反电动势：两级「自适应」低通，滤波器系数 Kslf = ω_est·Ts（设下限）
 *   4. 角度：θ = atan2(−e_final_α, e_final_β) + 固定相移 atan(3)≈71.57°
 *      （=AN1078 CONSTANT_PHASE_SHIFT；补两级低通在 Kslf=ω·Ts 时的总相移）
 *   5. 速度：θ 差分 → ω·Ts，平滑后用于自适应滤波系数
 *
 * 与磁链观测器（mcl_observer_flux / mcl_observer_ortega）实现同一
 * mcl_observer_ops 接口，可注入 mcl 载体对比性能。
 *
 * 精度限制：Q15 变速（加速段）相位不准（±130° 级振荡，16 位量化极限），
 * 建议无感闭环用 Q31/float，或改用 ORTEGA（Q15 变速稳定）。详见 smcpos 实现头注释。
 */

#ifndef MCL_OBSERVER_SMO_H
#define MCL_OBSERVER_SMO_H

#include "mcl_types.h"
#include "mcl_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 滑模观测器参数（AN1078 语义）
 *
 * 与 smcpos.c 的逐项对照：
 *   resistance  → motorParm.qRs / NORM_RS           （相电阻 R，用于 F=1−R·Ts/L）
 *   inductance  → motorParm.qLsDt / NORM_LSDTBASE   （相电感 L，用于 G=Ts/L）
 *   flux        → 永磁磁链 ψ_f（AN1078 SMO 本身不用，保留供 seed/接口一致）
 *   gain        → Kslide = SMCGAIN = 0.85          （滑模增益，无量纲 0~1）
 *   lpf         → 最低电气转速（ENDSPEED_ELECTR 等价，rad/s 或 ω_pu）
 *                （用于反电动势滤波系数下限 Kslf_min = lpf·dt，防零速死锁）
 *   boundary    → MaxSMCError = 0.005              （线性滑模区最大电流误差）
 *
 * 反电动势自适应滤波系数（AN1078: Kslf = Ω·THETA_FILTER_CNST = ω·Ts）：
 *   mcl 里 w_est 直接存「每采样角增量 ω·Ts」（无量纲），故 Kslf = |w_est|；
 *   下限 Kslf_min = lpf·dt（lpf = 最低电气转速 ENDSPEED_ELECTR 等价）。
 *   mcl 反电动势约定 e_α=−ωλ·sinθ、e_β=+ωλ·cosθ，atan2(−e_α, e_β)=θ；
 *   两级低通在 Kslf=ω·Ts 时总相移 atan(3)≈71.57°，故角度补固定 +71.57°
 *   （=AN1078 CONSTANT_PHASE_SHIFT）。
 *
 * 量纲约定：float=物理量（rad/s、V、A、s）；定点 per-unit + 角度「圈」(1.0=2π)。
 */
typedef struct
{
    mcl_scalar resistance;      /**< 相电阻 Ω（float）/ pu（定点） */
    mcl_scalar inductance;      /**< 相电感 H（float）/ pu（定点） */
    mcl_scalar flux;            /**< 永磁磁链 ψ_f Wb（float）/ pu（定点），接口一致用 */
    mcl_scalar gain;            /**< 滑模增益 Kslide（AN1078 SMCGAIN=0.85，0~1 无量纲） */
    mcl_scalar lpf;             /**< 最低电气转速 rad/s（float）/ ω_pu（定点），滤波系数下限 */
    mcl_scalar boundary;        /**< 线性滑模区最大电流误差 MaxSMCError（AN1078=0.005） */
} mcl_observer_smo_params;

/**
 * @brief 滑模观测器实例
 */
typedef struct
{
    mcl_observer_smo_params params; /**< 参数 */
    mcl_scalar i_alpha_hat;         /**< 估计电流 α A */
    mcl_scalar i_beta_hat;          /**< 估计电流 β A */
    mcl_scalar e_alpha;             /**< 反电动势 α（一级滤波） */
    mcl_scalar e_beta;              /**< 反电动势 β（一级滤波） */
    mcl_scalar e_alpha_final;       /**< 反电动势 α（二级滤波，用于角度） */
    mcl_scalar e_beta_final;        /**< 反电动势 β（二级滤波，用于角度） */
    mcl_scalar w_est;               /**< 估计电气速度（带符号，用于自适应滤波系数） */
    mcl_scalar theta_prev;          /**< 上一拍角度（速度差分） */
} mcl_observer_smo;

/**
 * @brief 滑模观测器接口（注入 mcl_observer 载体用）
 */
extern const mcl_observer_ops mcl_observer_smo_ops;

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_SMO_H */
