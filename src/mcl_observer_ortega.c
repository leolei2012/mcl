/**
 * @file    mcl_observer_ortega.c
 * @brief   mcl 电机控制库：ORTEGA 型磁链观测器实现
 *
 * 参考 Ortega et al. 观测器（VESC FOC_OBSERVER_ORTEGA_ORIGINAL）：
 *
 *   err = λ² - |λ_r|²，λ_r = ψ_s - L*i
 *   if (err > 0) err = 0          （非对称 clamp，保证全局收敛）
 *   dψ_s/dt = v - R*i + (γ/2)·λ_r·err
 *   phase  = atan2(λ_r_β, λ_r_α)
 *
 * 相位由 PLL 估计，此处不直接输出；速度输出置 0。
 */

#include "mcl_observer_ortega.h"
#include "mcl_math.h"

#ifndef MCL_DISABLE_OBSERVER

static void ortega_reset(void *impl);

static void ortega_init(void *impl, const void *params)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    if (params != NULL)
    {
        self->params = *(const mcl_observer_ortega_params *)params;
    }

    ortega_reset(impl);
}

static void ortega_reset(void *impl)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    self->x1 = (mcl_scalar)0;
    self->x2 = (mcl_scalar)0;
    self->lambda_est = (mcl_scalar)0;
    self->i_alpha_last = (mcl_scalar)0;
    self->i_beta_last = (mcl_scalar)0;
}

static void ortega_update(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                          mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                          mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    mcl_scalar R = self->params.resistance;
    mcl_scalar L = self->params.inductance;
    mcl_scalar lambda = self->params.lambda;
    mcl_scalar gamma_half;
    mcl_scalar L_ia;
    mcl_scalar L_ib;
    mcl_scalar lambda_alpha;
    mcl_scalar lambda_beta;
    mcl_scalar err;

    if (self == NULL)
    {
        return;
    }

    gamma_half = MCL_MUL(self->params.gain, (mcl_scalar)0.5f);
    L_ia = MCL_MUL(L, i_alpha);
    L_ib = MCL_MUL(L, i_beta);

    /* 估计转子磁链（更新前） */
    lambda_alpha = MCL_SUB(self->x1, L_ia);
    lambda_beta = MCL_SUB(self->x2, L_ib);

    /* 幅值平方误差：err = λ² - |λ_r|²，非对称 clamp（err>0 置 0）保证收敛 */
    err = MCL_SUB(MCL_MUL(lambda, lambda),
                  MCL_ADD(MCL_MUL(lambda_alpha, lambda_alpha),
                          MCL_MUL(lambda_beta, lambda_beta)));
    if (err > (mcl_scalar)0)
    {
        err = (mcl_scalar)0;
    }

    /* 定子磁链积分 + 沿 λ_r 方向的幅值反馈 */
    self->x1 = MCL_ADD(self->x1,
        MCL_MUL(MCL_ADD(MCL_SUB(v_alpha, MCL_MUL(R, i_alpha)),
                        MCL_MUL(MCL_MUL(gamma_half, lambda_alpha), err)), dt));
    self->x2 = MCL_ADD(self->x2,
        MCL_MUL(MCL_ADD(MCL_SUB(v_beta, MCL_MUL(R, i_beta)),
                        MCL_MUL(MCL_MUL(gamma_half, lambda_beta), err)), dt));

    /* 更新后转子磁链 + 幅值 */
    lambda_alpha = MCL_SUB(self->x1, L_ia);
    lambda_beta = MCL_SUB(self->x2, L_ib);
    self->lambda_est = mcl_math_sqrt(MCL_ADD(MCL_MUL(lambda_alpha, lambda_alpha),
                                              MCL_MUL(lambda_beta, lambda_beta)));

    self->i_alpha_last = i_alpha;
    self->i_beta_last = i_beta;

    if (phase_rad != NULL)
    {
        *phase_rad = mcl_math_atan2(lambda_beta, lambda_alpha);
    }
    if (speed_rad_s != NULL)
    {
        *speed_rad_s = (mcl_scalar)0;
    }
}

static void ortega_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    /* 定子磁链 = 转子磁链 + L*i（用上一拍电流近似当前） */
    self->x1 = MCL_ADD(flux_alpha, MCL_MUL(self->params.inductance, self->i_alpha_last));
    self->x2 = MCL_ADD(flux_beta, MCL_MUL(self->params.inductance, self->i_beta_last));
    self->lambda_est = self->params.lambda;
}

const mcl_observer_ops mcl_observer_ortega_ops = {
    .init = ortega_init,
    .reset = ortega_reset,
    .update = ortega_update,
    .seed = ortega_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
