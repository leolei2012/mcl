/**
 * @file    mcl_observer_smo.c
 * @brief   mcl 电机控制库：滑模观测器（SMO）实现
 *
 * 电流滑模观测器（αβ，幅值不变）：
 *   1. 电流观测：i_hat += (v - R·i_hat + z) · dt / L
 *   2. 滑模项：z = k · sat(i - i_hat, boundary)（饱和函数抑制抖振）
 *   3. 反电动势提取：e += lpf · (-z - e) · dt（低通；滑动面上 z_eq = -e，故取 -z 得 +e）
 *   4. 相位：θ = atan2(-e_α, e_β) + atan(ω_est/lpf)（低通相位补偿）
 *
 * 符号约定说明：被控对象 di/dt = (v - R·i - e)/L，观测器 di_hat/dt = (v - R·i_hat + z)/L，
 * 滑动面上等价控制 z_eq = -e；反电动势取自 -z；相位 atan2(-e_α, e_β)（-e_α = ψ_f·ω·sinθ）。
 * 两处负号自洽，无符号错误。
 *
 * 低通相位补偿：反电动势经一阶低通（截止 lpf）在速度 ω 下滞后 φ = atan(ω/lpf)，
 * 用估计速度 ω_est = |e|/ψ_f 补偿，消除 lpf 引入的稳态相位误差。
 *
 * 已知限制（SMO 抖振抑制长尾）：闭环变速阶段仍有轻微转速振荡，源于滑模方波
 * 经低通的残留纹波，需自适应增益/二阶滤波进一步抑制。
 */

#include "mcl_observer_smo.h"
#include "mcl_math.h"

static void smo_reset(void *impl);

static void smo_init(void *impl, const void *params)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    if (params != NULL)
    {
        self->params = *(const mcl_observer_smo_params *)params;
    }

    smo_reset(impl);
}

static void smo_reset(void *impl)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    self->i_alpha_hat = (mcl_scalar)0;
    self->i_beta_hat = (mcl_scalar)0;
    self->e_alpha = (mcl_scalar)0;
    self->e_beta = (mcl_scalar)0;
}

static mcl_scalar smo_sign(mcl_scalar x)
{
    if (x > (mcl_scalar)0) { return MCL_FROM_FLOAT(1.0f); }
    if (x < (mcl_scalar)0) { return MCL_FROM_FLOAT(-1.0f); }
    return (mcl_scalar)0;
}

/* 饱和函数：边界层内线性，层外饱和（抑制滑模抖振） */
static mcl_scalar smo_sat(mcl_scalar x, mcl_scalar boundary)
{
    if (boundary <= (mcl_scalar)0)
    {
        return smo_sign(x);
    }
    if (x > boundary) { return MCL_FROM_FLOAT(1.0f); }
    if (x < MCL_NEG(boundary)) { return MCL_FROM_FLOAT(-1.0f); }
    return MCL_DIV(x, boundary);
}

static void smo_update(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                       mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                       mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    mcl_scalar err_a;
    mcl_scalar err_b;
    mcl_scalar z_a;
    mcl_scalar z_b;
    mcl_scalar dt_over_l;

    if (self == NULL)
    {
        return;
    }

    /* 电流误差 */
    err_a = MCL_SUB(i_alpha, self->i_alpha_hat);
    err_b = MCL_SUB(i_beta, self->i_beta_hat);

    /* 滑模项 z = k · sat(err, boundary) */
    z_a = MCL_MUL(self->params.gain, smo_sat(err_a, self->params.boundary));
    z_b = MCL_MUL(self->params.gain, smo_sat(err_b, self->params.boundary));

    /* 电流观测：i_hat += (v - R·i_hat + z) · dt / L（TODO 定点：除法） */
    dt_over_l = MCL_DIV(dt, self->params.inductance);
    self->i_alpha_hat = MCL_ADD(self->i_alpha_hat,
                                MCL_MUL(MCL_ADD(MCL_SUB(v_alpha,
                                    MCL_MUL(self->params.resistance, self->i_alpha_hat)), z_a), dt_over_l));
    self->i_beta_hat = MCL_ADD(self->i_beta_hat,
                               MCL_MUL(MCL_ADD(MCL_SUB(v_beta,
                                    MCL_MUL(self->params.resistance, self->i_beta_hat)), z_b), dt_over_l));

    /* 反电动势低通滤波：e_hat ≈ -z（z 稳态估计 -e，故取 -z 得 +e）
       e += lpf · (-z - e) · dt */
    self->e_alpha = MCL_ADD(self->e_alpha,
                            MCL_MUL(MCL_MUL(MCL_SUB(MCL_NEG(z_a), self->e_alpha), self->params.lpf), dt));
    self->e_beta = MCL_ADD(self->e_beta,
                           MCL_MUL(MCL_MUL(MCL_SUB(MCL_NEG(z_b), self->e_beta), self->params.lpf), dt));

    /* 相位：θ = atan2(-e_α, e_β) + 低通相位补偿
       反电动势经一阶低通（截止 lpf）在速度 ω 下有滞后 φ = atan(ω/lpf)，
       用估计速度 ω_est = |e|/ψ_f 补偿之，消除 lpf 引入的相位误差 */
    if (phase_rad != NULL)
    {
        mcl_scalar mag = mcl_math_sqrt(MCL_ADD(MCL_MUL(self->e_alpha, self->e_alpha),
                                               MCL_MUL(self->e_beta, self->e_beta)));
        mcl_scalar w_est = (self->params.flux > (mcl_scalar)0)
                         ? MCL_DIV(mag, self->params.flux) : (mcl_scalar)0;
        mcl_scalar theta_raw = mcl_math_atan2(MCL_NEG(self->e_alpha), self->e_beta);
        mcl_scalar comp = mcl_math_fast_atan2(w_est, self->params.lpf);
        *phase_rad = MCL_ADD(theta_raw, comp);
    }

    /* 速度由 PLL 估计，此处不直接输出 */
    if (speed_rad_s != NULL)
    {
        *speed_rad_s = (mcl_scalar)0;
    }
}

static void smo_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    /* 预置反电动势方向：e 与转子磁链垂直，e ∝ [-flux_β, flux_α]。
       幅值用 |flux|（≈ ψ_f）近似，切闭环后 update 会修正到 ψ_f·ω。
       电流估计 i_hat 无法从磁链推（缺实测电流），保持连续。 */
    self->e_alpha = MCL_NEG(flux_beta);
    self->e_beta = flux_alpha;
}

const mcl_observer_ops mcl_observer_smo_ops = {
    .init = smo_init,
    .reset = smo_reset,
    .update = smo_update,
    .seed = smo_seed,
    .get_confidence = NULL
};
