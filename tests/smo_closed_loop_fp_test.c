/**
 * @file    smo_closed_loop_fp_test.c
 * @brief   mcl 滑模观测器（SMO）无感速度环闭环验证（float/Q15/Q31 三精度）
 *
 * 把 SMO 放进无感 FOC 速度环闭环，从有初始转速起步，验证能否稳定跟踪目标。
 * 基值：V_BASE = W_BASE·λ_BASE，V_BASE=V_BUS=8V，W_BASE=400 rad/s（电气），
 * λ_BASE=0.02 Wb，I_BASE=10 A，L_BASE=V_BASE/(W_BASE·I_BASE)=0.002 H。
 *
 * SMO 参数（AN1078 语义）：gain=Kslide=0.85（无量纲）、boundary=MaxSMCError=0.005
 * （电流误差，归一化）、lpf=最低电气转速（rad/s 或 ω_pu，滤波系数下限）。
 */

#include "mcl.h"
#include "mcl_observer_smo.h"
#include <stdio.h>
#include <math.h>

#define I_BASE        10.0f
#define V_BASE        8.0f
#define LAMBDA_BASE   0.02f
#define W_BASE        (V_BASE / LAMBDA_BASE)          /* 400 */
#define RPM_BASE      (W_BASE / 4.0f * 60.0f / (2.0f * 3.14159265358979f))
#define TWO_PI_F      (2.0f * 3.14159265358979f)

#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    #define K_I      (1.0f / I_BASE)
    #define K_V      (V_BASE)
    #define K_W      (1.0f / W_BASE)
    #define K_T      (1.0f / W_BASE)
    #define K_ANG    (1.0f / TWO_PI_F)
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

/* ---- 电机模型（float 物理，含机械方程） ---- */
typedef struct { float R, L, lambda, J, pole_pairs, i_alpha, i_beta, theta_e, omega_e; } motor_t;
typedef struct { motor_t motor; float v_alpha, v_beta; } hal_t;

static void motor_step(motor_t *m, float va, float vb, float dt)
{
    float ea = -m->omega_e * m->lambda * sinf(m->theta_e);
    float eb =  m->omega_e * m->lambda * cosf(m->theta_e);
    float iq, te, wm;
    m->i_alpha += (va - m->R * m->i_alpha - ea) * dt / m->L;
    m->i_beta  += (vb - m->R * m->i_beta  - eb) * dt / m->L;
    iq = -m->i_alpha * sinf(m->theta_e) + m->i_beta * cosf(m->theta_e);
    te = 1.5f * m->pole_pairs * m->lambda * iq;
    wm = m->omega_e / m->pole_pairs;
    wm += te / m->J * dt;
    m->omega_e = wm * m->pole_pairs;
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
    (void)ctx; *vbus = MCL_FROM_FLOAT(1.0f); *ibus = (mcl_scalar)0; return MCL_OK;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    hal_t hctx;
    mcl_config cfg;
    mcl_observer_smo obs;
    mcl_observer_smo_params op;
    float rpm_final, phase_err;
    int i;

    hctx.motor.R = 0.1f;  hctx.motor.L = 0.001f;  hctx.motor.lambda = 0.02f;
    hctx.motor.J = 0.0005f;  hctx.motor.pole_pairs = 4.0f;
    hctx.motor.i_alpha = 0;  hctx.motor.i_beta = 0;
    hctx.motor.theta_e = 0;
    hctx.motor.omega_e = 100.0f;   /* 初始 100 rad/s 电气 ≈ 238.7 rpm 机械 */
    hctx.v_alpha = 0;  hctx.v_beta = 0;

    hal.pwm_set_duty = hal_pwm;
    hal.adc_read_phase = hal_adc;
    hal.adc_read_bus = hal_bus;
    hal.enc_read_angle = NULL;
    hal.enc_read_speed = NULL;
    hal.read_temp = NULL;  hal.micros = NULL;

    /* SMO 参数 per-unit（归一化到 <1） */
    op.resistance = MCL_FROM_FLOAT(0.1f * K_R);
    op.inductance = MCL_FROM_FLOAT(0.001f * K_L);
    op.flux = MCL_FROM_FLOAT(0.02f * K_LAM);
    op.gain = MCL_FROM_FLOAT(0.85f);                 /* AN1078 SMCGAIN，无量纲 */
    op.lpf = MCL_FROM_FLOAT(100.0f * K_W);            /* 最低电气转速 100 rad/s（ω_pu） */
    op.boundary = MCL_FROM_FLOAT(1.0f * K_I);         /* 滑模边界层 1 A（float）/ 0.1 pu（定点） */

    mcl_config_default(&cfg);
    cfg.pole_pairs = 4;
    cfg.current_loop_freq_hz = 10000;
    cfg.max_duty = MCL_FROM_FLOAT(1.0f);
    cfg.bus_voltage = MCL_FROM_FLOAT(1.0f);
    cfg.time_base = MCL_FROM_FLOAT(K_T);
    cfg.phase_resistance = MCL_FROM_FLOAT(0.1f * K_R);
    cfg.phase_inductance = MCL_FROM_FLOAT(0.001f * K_L);
    cfg.bemf_const = MCL_FROM_FLOAT(0.02f * K_LAM);
    cfg.rated_current = MCL_FROM_FLOAT(1.0f);
    cfg.rated_speed_rpm = MCL_FROM_FLOAT(1.0f);
    cfg.current_pid.kp = MCL_FROM_FLOAT(0.5f);
    cfg.current_pid.ki = MCL_FROM_FLOAT(0.3f);
    cfg.current_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg.current_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg.current_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg.current_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg.speed_pid.kp = MCL_FROM_FLOAT(0.3f);
    cfg.speed_pid.ki = MCL_FROM_FLOAT(0.05f);
    cfg.speed_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg.speed_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg.speed_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg.speed_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg.openloop_rpm = (mcl_scalar)0;   /* 禁用自动开环，纯闭环测 SMO */
    cfg.limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg.limits.overvoltage = MCL_FROM_FLOAT(10.0f);
    cfg.limits.undervoltage = MCL_FROM_FLOAT(-10.0f);
    cfg.limits.overtemp = MCL_FROM_FLOAT(1000.0f);
    cfg.limits.stall_speed = (mcl_scalar)0;
    cfg.limits.stall_time = MCL_FROM_FLOAT(100000.0f);

    mcl_init(&motor, &cfg, &hal, &hctx, &mcl_observer_smo_ops, &obs, &op);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);

    /* 阶段 1：维持 ~238.7 rpm，让 SMO + PLL 收敛 */
    mcl_set_speed(&motor, MCL_FROM_FLOAT(238.7f * K_SPD));
    mcl_start(&motor);
    for (i = 0; i < 3000; i++) { mcl_control_tick(&motor); }
    rpm_final = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;
    phase_err = hctx.motor.theta_e - MCL_TO_FLOAT(motor.phase_rad) * (K_ANG > 1.0f ? TWO_PI_F : 1.0f);
    if (K_ANG > 1.0f) { phase_err = hctx.motor.theta_e - MCL_TO_FLOAT(motor.phase_rad) * TWO_PI_F; }
    else { phase_err = hctx.motor.theta_e - MCL_TO_FLOAT(motor.phase_rad); }
    while (phase_err > 3.14159265f) { phase_err -= 6.283185f; }
    while (phase_err < -3.14159265f) { phase_err += 6.283185f; }
    printf("  [阶段1后] rpm=%.1f 相位误差=%.3f rad\n", rpm_final, phase_err);

    /* 阶段 2：加速到 500 rpm */
    mcl_set_speed(&motor, MCL_FROM_FLOAT(500.0f * K_SPD));
    for (i = 0; i < 10000; i++) { mcl_control_tick(&motor); }

    rpm_final = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;
    if (K_ANG > 1.0f) { phase_err = hctx.motor.theta_e - MCL_TO_FLOAT(motor.phase_rad) * TWO_PI_F; }
    else { phase_err = hctx.motor.theta_e - MCL_TO_FLOAT(motor.phase_rad); }
    while (phase_err > 3.14159265f) { phase_err -= 6.283185f; }
    while (phase_err < -3.14159265f) { phase_err += 6.283185f; }

    printf("===== SMO 无感速度环闭环（%s）=====\n",
#if defined(MCL_USE_Q15)
           "Q15"
#elif defined(MCL_USE_Q31)
           "Q31"
#else
           "float"
#endif
    );
    printf("  目标 500 rpm，稳态转速 = %.1f rpm，相位误差 = %.3f rad\n", rpm_final, phase_err);
    printf("  状态 state=%d fault=%d\n", (int)motor.state, (int)motor.fault);

    return 0;
}
