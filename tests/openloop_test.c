/**
 * @file    openloop_test.c
 * @brief   mcl 开环路径仿真验证（float/Q15/Q31 三精度）
 *
 * 覆盖三条开环代码路径（这些路径此前只在 float 的 term_sim.c 中测试，
 * 定点下无人验证，且涉及 sign/电压钳位/角度回绕/seed 角等定点常量，容易出错）：
 *   1) 手动 VF（mcl_set_openloop_vf）——旋转电压矢量拖动转子；
 *   2) 预定位 ALIGN（mcl_set_openloop_align）——固定电流矢量吸到目标角；
 *   3) 自动开环→闭环切换（openloop_rpm + openloop_seed_angle）——零速启动收敛。
 *
 * 电机模型用 float 物理量（含机械方程），HAL 边界做 归一化/反归一化（per-unit）。
 * per-unit 基值：V_BASE = W_BASE·λ_BASE，V_BASE = V_BUS = 8V，W_BASE = 400 rad/s，
 * λ_BASE = 0.02 Wb，I_BASE = 10 A，RPM_BASE = 954.9 rpm。
 */

#include "mcl.h"
#include "mcl_observer_ortega.h"
#include <stdio.h>
#include <math.h>

#define I_BASE        10.0f
#define V_BASE        8.0f
#define LAMBDA_BASE   0.02f
#define W_BASE        (V_BASE / LAMBDA_BASE)          /* 400 */
#define RPM_BASE      (W_BASE / 4.0f * 60.0f / (2.0f * 3.14159265358979f))

#define TWO_PI_F      (2.0f * 3.14159265358979f)

static int g_fail = 0;
#define CHECK(name, cond) \
    do { if (cond) { printf("  [OK]   %s\n", name); } \
         else { printf("  [FAIL] %s\n", name); g_fail++; } } while (0)

/* ---- 归一化因子（float=恒等 / 定点=基值换算），集中在 HAL 边界用 ---- */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    #define K_I      (1.0f / I_BASE)
    #define K_V      (V_BASE)
    #define K_W      (1.0f / W_BASE)
    #define K_T      (1.0f / W_BASE)      /* time_base */
    #define K_ANG    (1.0f / TWO_PI_F)    /* 角度 ÷2π → 归一化圈 */
    #define K_SPD    (1.0f / RPM_BASE)
    #define K_R      (1.0f / (V_BASE / I_BASE))
    #define K_L      (1.0f / (V_BASE / (W_BASE * I_BASE)))
    #define K_LAM    (1.0f / LAMBDA_BASE)
#else
    #define K_I      (1.0f)
    #define K_V      (1.0f)
    #define K_W      (1.0f)
    #define K_T      (1.0f)
    #define K_ANG    (1.0f)
    #define K_SPD    (1.0f)
    #define K_R      (1.0f)
    #define K_L      (1.0f)
    #define K_LAM    (1.0f)
#endif

/* ---- 电机模型（float 物理量，含机械方程） ---- */
typedef struct { float R, L, lambda, J, pole_pairs; float i_alpha, i_beta; float theta_e, omega_e; } motor_t;
typedef struct { motor_t motor; float v_alpha, v_beta; } hal_t;

static void motor_step(motor_t *m, float v_alpha, float v_beta, float dt)
{
    float e_alpha = -m->omega_e * m->lambda * sinf(m->theta_e);
    float e_beta  =  m->omega_e * m->lambda * cosf(m->theta_e);
    float iq, te, omega_m;

    m->i_alpha += (v_alpha - m->R * m->i_alpha - e_alpha) * dt / m->L;
    m->i_beta  += (v_beta - m->R * m->i_beta - e_beta) * dt / m->L;

    iq = -m->i_alpha * sinf(m->theta_e) + m->i_beta * cosf(m->theta_e);
    te = 1.5f * m->pole_pairs * m->lambda * iq;
    omega_m = m->omega_e / m->pole_pairs;
    omega_m += te / m->J * dt;
    m->omega_e = omega_m * m->pole_pairs;
    m->theta_e += m->omega_e * dt;
    while (m->theta_e > TWO_PI_F) { m->theta_e -= TWO_PI_F; }
}

static void hal_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    hal_t *h = (hal_t *)ctx;
    float va = (2.0f * MCL_TO_FLOAT(da) - 1.0f) * K_V;
    float vb = (2.0f * MCL_TO_FLOAT(db) - 1.0f) * K_V;
    float vc = (2.0f * MCL_TO_FLOAT(dc) - 1.0f) * K_V;
    h->v_alpha = va;
    h->v_beta = (vb - vc) / 1.7320508f;
}

static int hal_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    hal_t *h = (hal_t *)ctx;
    motor_step(&h->motor, h->v_alpha, h->v_beta, 0.0001f);
    *ia = MCL_FROM_FLOAT(h->motor.i_alpha * K_I);
    *ib = MCL_FROM_FLOAT((-0.5f * h->motor.i_alpha + 0.8660254f * h->motor.i_beta) * K_I);
    *ic = MCL_FROM_FLOAT((-0.5f * h->motor.i_alpha - 0.8660254f * h->motor.i_beta) * K_I);
    return MCL_OK;
}

static int hal_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;
    *vbus = MCL_FROM_FLOAT(1.0f);   /* V_BUS/V_BASE = 1 */
    *ibus = (mcl_scalar)0;
    return MCL_OK;
}

static void model_setup(hal_t *h, mcl_hal_ops *hal)
{
    h->motor.R = 0.1f;
    h->motor.L = 0.001f;
    h->motor.lambda = 0.02f;
    h->motor.J = 0.0005f;
    h->motor.pole_pairs = 4.0f;
    h->motor.i_alpha = 0.0f;
    h->motor.i_beta = 0.0f;
    h->motor.theta_e = 0.0f;
    h->motor.omega_e = 0.0f;
    h->v_alpha = 0.0f;
    h->v_beta = 0.0f;

    hal->pwm_set_duty = hal_pwm;
    hal->adc_read_phase = hal_adc;
    hal->adc_read_bus = hal_bus;
    hal->enc_read_angle = NULL;
    hal->enc_read_speed = NULL;
    hal->read_temp = NULL;
    hal->micros = NULL;
}

static void cfg_setup(mcl_config *cfg)
{
    mcl_config_default(cfg);
    cfg->pole_pairs = 4;
    cfg->current_loop_freq_hz = 10000;
    cfg->max_duty = MCL_FROM_FLOAT(1.0f);
    cfg->bus_voltage = MCL_FROM_FLOAT(1.0f);
    cfg->time_base = MCL_FROM_FLOAT(K_T);
    cfg->phase_resistance = MCL_FROM_FLOAT(0.1f * K_R);
    cfg->phase_inductance = MCL_FROM_FLOAT(0.001f * K_L);
    cfg->bemf_const = MCL_FROM_FLOAT(0.02f * K_LAM);
    cfg->rated_current = MCL_FROM_FLOAT(1.0f);
    cfg->rated_speed_rpm = MCL_FROM_FLOAT(1.0f);
    cfg->current_pid.kp = MCL_FROM_FLOAT(0.5f);
    cfg->current_pid.ki = MCL_FROM_FLOAT(0.3f);
    cfg->current_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg->current_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg->current_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg->current_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg->speed_pid.kp = MCL_FROM_FLOAT(0.3f);
    cfg->speed_pid.ki = MCL_FROM_FLOAT(0.1f);
    cfg->speed_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg->speed_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg->speed_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg->speed_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg->limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg->limits.overvoltage = MCL_FROM_FLOAT(10.0f);
    cfg->limits.undervoltage = MCL_FROM_FLOAT(-10.0f);
    cfg->limits.overtemp = MCL_FROM_FLOAT(1000.0f);
    cfg->limits.stall_speed = (mcl_scalar)0;
    cfg->limits.stall_time = MCL_FROM_FLOAT(100000.0f);
}

static float rpm_of(hal_t *h)
{
    return h->motor.omega_e / h->motor.pole_pairs * 9.5492966f;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    hal_t hctx;
    mcl_config cfg;
    int i;
    float rpm_final, theta_final;

    printf("===== mcl 开环路径仿真（%s）=====\n",
#if defined(MCL_USE_Q15)
           "Q15"
#elif defined(MCL_USE_Q31)
           "Q31"
#else
           "float"
#endif
    );

    /* ---------- 场景 1：手动 VF（旋转电压矢量拖动） ---------- */
    model_setup(&hctx, &hal);
    cfg_setup(&cfg);
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_openloop_vf(&motor, MCL_FROM_FLOAT(1.0f), MCL_FROM_FLOAT(100.0f * K_SPD));
    mcl_start(&motor);
    for (i = 0; i < 6000; i++) { mcl_control_tick(&motor); }
    rpm_final = rpm_of(&hctx);
    CHECK("VF 拖动转子转动（稳态转速 > 50 rpm）", rpm_final > 50.0f);
    printf("        VF 稳态转速 = %.1f rpm（目标 100）\n", rpm_final);

    /* ---------- 场景 2：预定位 ALIGN（吸到目标电气角） ---------- */
    model_setup(&hctx, &hal);
    cfg_setup(&cfg);
    hctx.motor.theta_e = 1.0f;   /* 从 57° 开始，远离 π（180°）歧义点，验证被拉到 0 */
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_openloop_align(&motor, MCL_FROM_FLOAT(1.0f),
                           MCL_FROM_FLOAT(0.0f * K_ANG));  /* 拉到 0 rad / 0 圈 */
    mcl_start(&motor);
    for (i = 0; i < 4000; i++) { mcl_control_tick(&motor); }
    theta_final = hctx.motor.theta_e;
    /* ALIGN 把转子从 57° 拉向 0 角：稳态角不为精确 0（电流环稳态 + 采样），
       判据是「显著偏离初始 1.0 rad、向 0 收敛」，即落在 [0, 0.6) 或 (2π-0.6, 2π)。 */
    while (theta_final >= TWO_PI_F) { theta_final -= TWO_PI_F; }
    while (theta_final < 0.0f) { theta_final += TWO_PI_F; }
    CHECK("ALIGN 转子被拉到目标角附近（≈0 rad）",
          theta_final < 0.6f || theta_final > (TWO_PI_F - 0.6f));
    printf("        最终电气角 = %.4f rad（期望 0）\n", theta_final);

    /* ---------- 场景 3：自动开环→闭环切换（零速启动） ---------- */
    model_setup(&hctx, &hal);
    cfg_setup(&cfg);
    cfg.openloop_rpm = MCL_FROM_FLOAT(150.0f * K_SPD);
    cfg.openloop_hyst = MCL_FROM_FLOAT(0.05f * K_T);      /* 时间阈值随 time_base 归一化 */
    cfg.openloop_time_lock = MCL_FROM_FLOAT(0.05f * K_T);
    cfg.openloop_time_ramp = MCL_FROM_FLOAT(0.15f * K_T);
    cfg.openloop_time = MCL_FROM_FLOAT(0.10f * K_T);
    {
        mcl_observer_ortega obs;
        mcl_observer_ortega_params op;
        op.lambda = MCL_FROM_FLOAT(0.02f * K_LAM);
        op.resistance = MCL_FROM_FLOAT(0.1f * K_R);
        op.inductance = MCL_FROM_FLOAT(0.001f * K_L);
        /* ORTEGA gain 归一化：定点下 gain_pu = gain·λ_BASE²·L_BASE（量纲 1/(λ·L·time)
           的 per-unit 形式）＝600000×0.02×0.02×0.002≈0.48；float 传物理值 600000。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
        op.gain = MCL_FROM_FLOAT(600000.0f * 0.02f * 0.02f * 0.002f);
#else
        op.gain = MCL_FROM_FLOAT(600000.0f);
#endif
        mcl_init(&motor, &cfg, &hal, &hctx, &mcl_observer_ortega_ops, &obs, &op);
    }
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_speed(&motor, MCL_FROM_FLOAT(500.0f * K_SPD));
    mcl_start(&motor);
    for (i = 0; i < 20000; i++) { mcl_control_tick(&motor); }
    rpm_final = rpm_of(&hctx);
    CHECK("自动开环切闭环并收敛（稳态转速 > 200 rpm）", rpm_final > 200.0f);
    printf("        零速启动稳态转速 = %.1f rpm（目标 500）\n", rpm_final);

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
