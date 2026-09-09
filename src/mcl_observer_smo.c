/**
 * @file    mcl_observer_smo.c
 * @brief   mcl 电机控制库：滑模观测器（SMO）实现 —— AN1078 式
 *
 * 忠实对齐 Microchip AN1078「Sensorless FOC of PMSM Using SMO」的 smcpos.c：
 *
 *   1. 滑模电流观测器（CalcEstI）：
 *        EstI = F·EstI + G·(V − E − Z)，其中 G = Ts/L、F = 1 − R·Ts/L
 *        电流误差 Ierr = EstI − I（估计值减实测值）。
 *   2. 滑模控制（CalcZalpha/CalcZbeta）：
 *        |Ierr| < MaxSMCError：Z = Kslide·Ierr/MaxSMCError（线性区）
 *        否则              ：Z = ±Kslide（饱和区）
 *        Kslide = SMCGAIN = 0.85（无量纲 0~1），MaxSMCError = 0.005。
 *   3. 反电动势（CalcBEMF）：两级「自适应」低通：
 *        E      += Kslf·(Z − E)        （一级，回喂电流观测器）
 *        Efinal += Kslf·(E − Efinal)   （二级，用于角度）
 *        Kslf = |ω_est|·Ts（= AN1078 的 Ω·THETA_FILTER_CNST），设下限 Kslf_min = lpf·Ts
 *   4. 角度：θ = atan2(−Efinal_α, Efinal_β)
 *        mcl 反电动势约定 e_α=−ωλ·sinθ、e_β=+ωλ·cosθ，故 atan2(−e_α, e_β)=θ，
 *        无需 AN1078 的固定 +90°（CONSTANT_PHASE_SHIFT）。AN1078 中该偏移补偿的是
 *        其内部 E 的 90° 相移，mcl 的 E 回喂/角度约定已等价消去，不额外加。
 *   5. 速度：θ 差分低通（用于自适应滤波系数 w_est）。
 *
 * 与旧实现的关键差异（根治恒速 90° 误差 / 变速 N-S 锁反）：
 *   - 反电动势滤波系数 Kslf 直接 = |ω_est|·Ts，不再额外乘 dt（消除系数过小导致 E 不收敛）；
 *   - Kslf 设下限（lpf·Ts），防 w_est=0 时系数归零 → E 永不更新 → 角度卡死；
 *   - 电流误差符号按 AN1078：Ierr = EstI − I（估计减实测）；
 *   - 两级滤波分工明确：一级 E 回喂电流观测器、二级 Efinal 进 atan2。
 *
 * 量纲约定：float=物理量（rad/s、V、A、s）；定点 per-unit + 角度「圈」(1.0=2π)。
 *
 * ── 精度限制备注 ─────────────────────────────────────────────
 * Q15 变速精度不足：SMO 含高频开关项（±Kslide 饱和滑模）和两级自适应低通，
 * 对量化噪声敏感。16 位 Q15（LSB≈3e-5）在「变速加速段」（反电动势 |e| 快速
 * 从 2V 升至 4V、电流 iq 剧变）下，量化误差累积使估计相位在 ±130° 级振荡
 * （转速仍收敛，但相位不准）。Q31（LSB≈4.7e-10）和 float 位宽足够，变速
 * 闭环正常；恒速下 Q15 也能到 ~1°。
 * 结论：无感闭环优先 Q31/float 或 ORTEGA（ORTEGA 在 Q15 变速下稳定）；
 * SMO 的 Q15 变速相位仅作参考、不保证精度（16 位量化极限，AN1078 用
 * dsPIC 40 位累加器 + 硬件饱和规避，纯 Q15 无法等价）。
 */

#include "mcl_observer_smo.h"
#include "mcl_math.h"

#ifndef MCL_DISABLE_OBSERVER

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
    self->e_alpha_final = (mcl_scalar)0;
    self->e_beta_final = (mcl_scalar)0;
    self->w_est = (mcl_scalar)0;
    self->theta_prev = (mcl_scalar)0;
}

/* AN1078 滑模控制：线性区 Z = Kslide·Ierr/MaxSMCError；饱和区 Z = ±Kslide。
   输入 x = 电流误差，boundary = MaxSMCError，输出 = Z/Kslide ∈ [-1,1]。
   （与 AN1078 CalcZalpha 的「比例项再乘 Kslide」等价，此处把除以 boundary 与
   乘以 Kslide 拆开，避免 MCL_DIV 的定点精度损失。） */
static mcl_scalar smo_slide_component(mcl_scalar err, mcl_scalar boundary)
{
    mcl_scalar r;
    if (boundary <= (mcl_scalar)0)
    {
        /* 退化：无线性区，纯开关 */
        if (err > (mcl_scalar)0) { return MCL_FROM_FLOAT(1.0f); }
        if (err < (mcl_scalar)0) { return MCL_FROM_FLOAT(-1.0f); }
        return (mcl_scalar)0;
    }
    if (err > boundary) { return MCL_FROM_FLOAT(1.0f); }
    if (err < MCL_NEG(boundary)) { return MCL_FROM_FLOAT(-1.0f); }
    r = MCL_DIV(err, boundary);
    if (r > MCL_FROM_FLOAT(1.0f)) { return MCL_FROM_FLOAT(1.0f); }
    if (r < MCL_FROM_FLOAT(-1.0f)) { return MCL_FROM_FLOAT(-1.0f); }
    return r;
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
    mcl_scalar G;   /* Ts/L */
    mcl_scalar F;   /* 1 − R·Ts/L */
    mcl_scalar kslf;
    mcl_scalar kslf_min;

    if (self == NULL)
    {
        return;
    }

    /* 1. 电流误差（AN1078 约定：Ierr = EstI − I，估计值减实测值） */
    err_a = MCL_SUB(self->i_alpha_hat, i_alpha);
    err_b = MCL_SUB(self->i_beta_hat, i_beta);

    /* 2. 滑模控制：z = Kslide·sat(err/MaxSMCError) */
    z_a = MCL_MUL(self->params.gain, smo_slide_component(err_a, self->params.boundary));
    z_b = MCL_MUL(self->params.gain, smo_slide_component(err_b, self->params.boundary));

    /* 3. 电流观测（CalcEstI）：EstI = F·EstI + G·(V − E − Z)
          G = Ts/L、F = 1 − R·Ts/L */
    G = MCL_DIV(dt, self->params.inductance);
    F = MCL_SUB(MCL_FROM_FLOAT(1.0f),
                MCL_MUL(MCL_DIV(self->params.resistance, self->params.inductance), dt));
    self->i_alpha_hat = MCL_ADD(MCL_MUL(F, self->i_alpha_hat),
        MCL_MUL(G, MCL_SUB(MCL_SUB(v_alpha, self->e_alpha), z_a)));
    self->i_beta_hat = MCL_ADD(MCL_MUL(F, self->i_beta_hat),
        MCL_MUL(G, MCL_SUB(MCL_SUB(v_beta, self->e_beta), z_b)));

    /* 4. 反电动势自适应滤波系数（AN1078）：Kslf = Ω·THETA_FILTER_CNST = ω_est·Ts
          下限 Kslf_min = ENDSPEED_ELECTR·THETA_FILTER_CNST = lpf·Ts
          （防 w_est 初始 0 → Kslf=0 → 反电动势永不更新 → 角度卡死）

          速度估计：w_est 存「每采样角增量 ω·Ts」（无量纲、远小于 1）。
          float 下角度为弧度，Δθ 已是 ω·Ts；定点下角度为「圈」，Δθ = ω·Ts/(2π)，
          故乘 2π（浮点常数）还原为 ω·Ts。这样 Kslf = |w_est| 直接成立，
          且避开「圈/dt 与 ω_pu 差 2π、2π 定点不可表达」的坑。 */
    kslf = MCL_ABS(self->w_est);
    kslf_min = MCL_MUL(self->params.lpf, dt);
    if (kslf < kslf_min)
    {
        kslf = kslf_min;
    }

    /* 5. 一级低通（CalcBEMF 前两行）：E += Kslf·(Z − E)
          该 E 回喂到上面的电流观测器（CalcEstI 的 e 项用一级 E） */
    self->e_alpha = MCL_ADD(self->e_alpha, MCL_MUL(kslf, MCL_SUB(z_a, self->e_alpha)));
    self->e_beta = MCL_ADD(self->e_beta, MCL_MUL(kslf, MCL_SUB(z_b, self->e_beta)));

    /* 6. 二级低通（CalcBEMF 后两行）：Efinal += Kslf·(E − Efinal)，用于角度 */
    self->e_alpha_final = MCL_ADD(self->e_alpha_final,
        MCL_MUL(kslf, MCL_SUB(self->e_alpha, self->e_alpha_final)));
    self->e_beta_final = MCL_ADD(self->e_beta_final,
        MCL_MUL(kslf, MCL_SUB(self->e_beta, self->e_beta_final)));

    /* 7. 角度：θ = atan2(−Efinal_α, Efinal_β) + 固定相移
          mcl 反电动势约定 e_α=−ωλ·sinθ、e_β=+ωλ·cosθ，故 atan2(−e_α, e_β)=θ；
          两级低通在截止频率处的相移固定为 ~90°，故补 AN1078 的 CONSTANT_PHASE_SHIFT
          = +90°（float +π/2，定点 +0.25 圈）。 */
    if (phase_rad != NULL)
    {
        mcl_scalar theta = mcl_math_atan2(MCL_NEG(self->e_alpha_final), self->e_beta_final);
        /* 固定相移补偿（AN1078 CONSTANT_PHASE_SHIFT 等价）：
           两级反电动势低通在 Kslf=ω·Ts 时，一级（含 e−E 回喂）滞后 atan(1/2)≈26.57°、
           二级滞后 atan(1)=45°，合计 atan(3)≈71.57°。该滞后与转速无关（自适应 Kslf），
           故补偿固定 +71.57°。float +1.24905 rad；定点 +0.19880 圈。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
        theta = MCL_ADD(theta, MCL_FROM_FLOAT(0.19880f));
        if (theta > MCL_FROM_FLOAT(0.5f)) { theta = MCL_SUB(theta, MCL_FROM_FLOAT(1.0f)); }
        if (theta < MCL_FROM_FLOAT(-0.5f)) { theta = MCL_ADD(theta, MCL_FROM_FLOAT(1.0f)); }
#else
        theta += MCL_FROM_FLOAT(1.24905f);
        if (theta > MCL_PI) { theta -= MCL_TWO_PI; }
        if (theta < -MCL_PI) { theta += MCL_TWO_PI; }
#endif
        *phase_rad = theta;

        /* 8. 速度：w_est = ω·Ts（每采样角增量，无量纲）。角度差分给出 Δθ：
              float=弧度（=ω·Ts 已正确）；定点=圈（=ω·Ts/(2π)，乘 2π 还原）。
              一阶平滑降低噪（w_est 用于下一步的 Kslf）。 */
        {
            mcl_scalar dtheta = MCL_SUB(theta, self->theta_prev);
            mcl_scalar w_step;
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
            if (dtheta > MCL_FROM_FLOAT(0.5f)) { dtheta = MCL_SUB(dtheta, MCL_FROM_FLOAT(1.0f)); }
            if (dtheta < MCL_FROM_FLOAT(-0.5f)) { dtheta = MCL_ADD(dtheta, MCL_FROM_FLOAT(1.0f)); }
            w_step = MCL_FROM_FLOAT(MCL_TO_FLOAT(dtheta) * 6.28318530718f); /* 圈→弧度 ω·Ts */
#else
            if (dtheta > MCL_FROM_FLOAT(3.14159265f)) { dtheta = MCL_SUB(dtheta, MCL_FROM_FLOAT(6.2831853f)); }
            if (dtheta < MCL_FROM_FLOAT(-3.14159265f)) { dtheta = MCL_ADD(dtheta, MCL_FROM_FLOAT(6.2831853f)); }
            w_step = dtheta;   /* 弧度，已是 ω·Ts */
#endif
            self->theta_prev = theta;
            /* 重平滑：w_est ← 0.95·w_est + 0.05·w_step（抗角度差分噪声） */
            self->w_est = MCL_ADD(MCL_MUL(self->w_est, MCL_FROM_FLOAT(0.95f)),
                                  MCL_MUL(w_step, MCL_FROM_FLOAT(0.05f)));
        }
    }

    if (speed_rad_s != NULL)
    {
        /* w_est = ω·Ts → 除以 dt 还原为归一化速度（ω/W_BASE = ω_pu）。
           实际闭环中 mcl 用 PLL 估速，此输出当前未使用（mcl.c 传 NULL）。 */
        *speed_rad_s = (dt > (mcl_scalar)0) ? MCL_DIV(self->w_est, dt) : (mcl_scalar)0;
    }
}

static void smo_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    /* 预置反电动势方向：e 与转子磁链垂直（e = ωe·J·λ）。
       mcl 反电动势约定 e_α=−ωλ·sinθ、e_β=+ωλ·cosθ，磁链 λ=(flux_alpha, flux_beta)
       对应 (λcosθ, λsinθ)，故 e = ωe·(−λ_β, λ_α)：
         e_α = −λ_β、e_β = +λ_α（幅值 −ωλ，方向由滤波收敛到实际幅值）。 */
    self->e_alpha = MCL_NEG(flux_beta);
    self->e_beta = flux_alpha;
    self->e_alpha_final = self->e_alpha;
    self->e_beta_final = self->e_beta;
    self->w_est = (mcl_scalar)0;
}

const mcl_observer_ops mcl_observer_smo_ops = {
    .init = smo_init,
    .reset = smo_reset,
    .update = smo_update,
    .seed = smo_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
