/**
 * @file    fixed_point_observer_test.c
 * @brief   mcl 定点观测器验证（per-unit 归一化，float/Q15/Q31 三精度共用）
 *
 * 用归一化（per-unit）的 PMSM 数据验证三个观测器（flux/SMO/ORTEGA）在定点下的
 * 相位估计精度，对比 float 精度结果，证明观测器层已可定点运行。
 *
 * 归一化基值（满足 V_BASE = W_BASE·λ_BASE，使电压方程 per-unit 形式不变）：
 *   W_BASE = 500 rad/s（电角速度基值）
 *   I_BASE = 10 A
 *   λ_BASE = 0.02 Wb  →  V_BASE = W_BASE·λ_BASE = 10 V
 *   R_BASE = V_BASE/I_BASE = 1 Ω
 *   L_BASE = V_BASE/(W_BASE·I_BASE) = 0.002 H
 *   T_BASE = 1/W_BASE = 0.002 s
 *
 * SMO 参数（AN1078 语义）：gain=Kslide=0.85（无量纲）、boundary=MaxSMCError=0.005
 * （电流误差，已归一化）、lpf=最低电气转速（rad/s 或 ω_pu，反电动势滤波系数下限）。
 */

#include "mcl.h"
#include "mcl_observer_flux.h"
#include "mcl_observer_smo.h"
#include "mcl_observer_ortega.h"
#include <stdio.h>
#include <math.h>

#define W_BASE        500.0f
#define I_BASE        10.0f
#define LAMBDA_BASE   0.02f
#define V_BASE        (W_BASE * LAMBDA_BASE)
#define R_BASE        (V_BASE / I_BASE)
#define L_BASE        (V_BASE / (W_BASE * I_BASE))

#define N 2000            /* 采样点 */
#define SKIP 200          /* 跳过收敛瞬态 */

/* 观测器相位输出 → 圈数（float 弧度÷2π；定点已是归一化圈数），统一量纲对比 */
static float phase_to_turns(mcl_scalar phase)
{
    float v = MCL_TO_FLOAT(phase);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    return v;
#else
    return v / 6.283185307f;
#endif
}

/* 用给定观测器跑 per-unit 数据，返回平均/max 相位误差（圈） */
static void run_one(const mcl_observer_ops *ops, void *impl, void *params,
                    mcl_scalar R_pu, mcl_scalar L_pu, mcl_scalar lam_pu,
                    mcl_scalar w_pu, mcl_scalar iq_pu, mcl_scalar dt_pu,
                    float *err_mean, float *err_max)
{
    mcl_observer carrier;
    float w = 100.0f, iq = 1.0f, dt = 0.0001f;   /* 物理运行点（生成数据用） */
    float err_sum = 0.0f, err_maxv = 0.0f;
    int i;

    (void)R_pu; (void)L_pu; (void)lam_pu; (void)w_pu; (void)iq_pu; (void)dt_pu;

    mcl_observer_init(&carrier, ops, impl, params);

    for (i = 0; i < N; i++)
    {
        float t = (float)i * dt;
        float th = w * t;
        float th_turns = th / 6.283185307f;

        /* 电流（物理）：id=0, iq 恒定 */
        float i_alpha = -iq * sinf(th);
        float i_beta  = iq * cosf(th);
        float di_alpha = -iq * w * cosf(th);
        float di_beta  = -iq * w * sinf(th);

        /* 电压（物理）：v = R·i + L·di/dt + e（R=1Ω, L=1mH, λ=0.02Wb） */
        float v_alpha = 1.0f * i_alpha + 0.001f * di_alpha - w * 0.02f * sinf(th);
        float v_beta  = 1.0f * i_beta  + 0.001f * di_beta  + w * 0.02f * cosf(th);

        mcl_scalar i_alpha_pu = MCL_FROM_FLOAT(i_alpha / I_BASE);
        mcl_scalar i_beta_pu  = MCL_FROM_FLOAT(i_beta / I_BASE);
        mcl_scalar v_alpha_pu = MCL_FROM_FLOAT(v_alpha / V_BASE);
        mcl_scalar v_beta_pu  = MCL_FROM_FLOAT(v_beta / V_BASE);
        mcl_scalar phase_est;

        mcl_observer_update(&carrier, v_alpha_pu, v_beta_pu, i_alpha_pu, i_beta_pu,
                            dt_pu, &phase_est, NULL);

        if (i > SKIP)
        {
            float pe = phase_to_turns(phase_est);
            float err = th_turns - pe;
            while (err > 0.5f) { err -= 1.0f; }
            while (err < -0.5f) { err += 1.0f; }
            float ae = err < 0 ? -err : err;
            err_sum += ae;
            if (ae > err_maxv) { err_maxv = ae; }
        }
    }

    *err_mean = err_sum / (float)(N - SKIP);
    *err_max = err_maxv;
}

int main(void)
{
    mcl_scalar R_pu   = MCL_FROM_FLOAT(1.0f / R_BASE);       /* 1.0  */
    mcl_scalar L_pu   = MCL_FROM_FLOAT(0.001f / L_BASE);     /* 0.5  */
    mcl_scalar lam_pu = MCL_FROM_FLOAT(0.02f / LAMBDA_BASE); /* 1.0  */
    mcl_scalar dt_pu  = MCL_FROM_FLOAT(0.0001f * W_BASE);    /* 0.05 */

    float m_flux, x_flux, m_smo, x_smo, m_ort, x_ort;

#if defined(MCL_USE_Q15)
    printf("===== 三观测器定点验证（Q15）=====\n");
#elif defined(MCL_USE_Q31)
    printf("===== 三观测器定点验证（Q31）=====\n");
#else
    printf("===== 三观测器定点验证（float）=====\n");
#endif

    /* flux */
    {
        mcl_observer_flux obs;
        mcl_observer_flux_params op;
        op.lambda = lam_pu; op.resistance = R_pu; op.inductance = L_pu;
        /* 幅值校正增益量纲 1/time：gain_pu = gain/W_BASE = 200/500 = 0.4 */
        op.gain = MCL_FROM_FLOAT(200.0f / W_BASE);
        run_one(&mcl_observer_flux_ops, &obs, &op, R_pu, L_pu, lam_pu,
                (mcl_scalar)0, (mcl_scalar)0, dt_pu, &m_flux, &x_flux);
        printf("  flux   平均 %.4f°  最大 %.2f°\n", m_flux * 360.0f, x_flux * 360.0f);
    }

    /* SMO */
    {
        mcl_observer_smo obs;
        mcl_observer_smo_params op;
        op.resistance = R_pu;
        op.inductance = L_pu;
        op.flux = lam_pu;
        op.gain = MCL_FROM_FLOAT(0.85f);              /* AN1078 SMCGAIN，无量纲 */
        op.lpf = MCL_FROM_FLOAT(100.0f / W_BASE);     /* 最低电气速度 100 rad/s（=运行点，ω_pu=0.2） */
        op.boundary = MCL_FROM_FLOAT(0.1f);           /* 滑模边界层（电流误差，归一化） */
        run_one(&mcl_observer_smo_ops, &obs, &op, R_pu, L_pu, lam_pu,
                (mcl_scalar)0, (mcl_scalar)0, dt_pu, &m_smo, &x_smo);
        printf("  SMO    平均 %.4f°  最大 %.2f°\n", m_smo * 360.0f, x_smo * 360.0f);
    }

    /* ORTEGA */
    {
        mcl_observer_ortega obs;
        mcl_observer_ortega_params op;
        op.lambda = lam_pu; op.resistance = R_pu; op.inductance = L_pu;
        op.gain = MCL_FROM_FLOAT(600000.0f * 0.02f * 0.02f * 0.002f);  /* 0.48 */
        run_one(&mcl_observer_ortega_ops, &obs, &op, R_pu, L_pu, lam_pu,
                (mcl_scalar)0, (mcl_scalar)0, dt_pu, &m_ort, &x_ort);
        printf("  ORTEGA 平均 %.4f°  最大 %.2f°\n", m_ort * 360.0f, x_ort * 360.0f);
    }

    return 0;
}
