/**
 * @file    smo_closed_loop_test.c
 * @brief   mcl 滑模观测器（SMO）无感速度环闭环验证（float）
 *
 * 回答「SMO 能不能用」：把 SMO 放进真实 FOC 无感速度环闭环，
 * 从有初始速度（反电动势足够大）起步，看能否稳定跟踪目标转速。
 * 采用与 term_sim 场景 5 相同的两阶段法（先维持当前转速让观测器收敛，再加速）。
 */

#include "mcl.h"
#include "mcl_observer_smo.h"
#include <stdio.h>
#include <math.h>

/* ---- 电机模型（float 物理，含机械方程） ---- */
typedef struct
{
    float R, L, lambda, J, pole_pairs;
    float i_alpha, i_beta, theta_e, omega_e;
} motor_t;

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
    while (m->theta_e > 6.283185307f) { m->theta_e -= 6.283185307f; }
}

static void hal_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    hal_t *h = (hal_t *)ctx;
    float va = 2.0f * MCL_TO_FLOAT(da) - 1.0f;   /* 满母线归一化 = 2V */
    float vb = 2.0f * MCL_TO_FLOAT(db) - 1.0f;
    float vc = 2.0f * MCL_TO_FLOAT(dc) - 1.0f;
    h->v_alpha = va;   /* 母线 2V，1.0 = 2V */
    h->v_beta = (vb - vc) / 1.7320508f;
}

static int hal_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    hal_t *h = (hal_t *)ctx;
    motor_step(&h->motor, h->v_alpha, h->v_beta, 0.0001f);
    *ia = h->motor.i_alpha;
    *ib = -0.5f * h->motor.i_alpha + 0.8660254f * h->motor.i_beta;
    *ic = -0.5f * h->motor.i_alpha - 0.8660254f * h->motor.i_beta;
    return MCL_OK;
}

static int hal_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx; *vbus = 2.0f; *ibus = 0.0f; return MCL_OK;
}

static int hal_enc(void *ctx, mcl_scalar *angle)
{
    hal_t *h = (hal_t *)ctx; *angle = h->motor.theta_e; return MCL_OK;
}

static int hal_spd(void *ctx, mcl_scalar *speed)
{
    hal_t *h = (hal_t *)ctx; *speed = h->motor.omega_e; return MCL_OK;
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

    hctx.motor.R = 1.0f;  hctx.motor.L = 0.001f;  hctx.motor.lambda = 0.02f;
    hctx.motor.J = 0.0001f;  hctx.motor.pole_pairs = 4.0f;
    hctx.motor.i_alpha = 0;  hctx.motor.i_beta = 0;
    hctx.motor.theta_e = 0;  hctx.motor.omega_e = 100.0f;  /* 初始 100 rad/s ≈ 238 rpm */
    hctx.v_alpha = 0;  hctx.v_beta = 0;

    hal.pwm_set_duty = hal_pwm;
    hal.adc_read_phase = hal_adc;
    hal.adc_read_bus = hal_bus;
    hal.enc_read_angle = hal_enc;
    hal.enc_read_speed = hal_spd;
    hal.read_temp = NULL;  hal.micros = NULL;

    op.resistance = 1.0f;
    op.inductance = 0.001f;
    op.flux = 0.02f;        /* 磁链 ψ_f */
    op.gain = 0.85f;        /* AN1078 SMCGAIN，无量纲 */
    op.lpf = 100.0f;        /* 最低电气转速 rad/s（反电动势滤波系数下限） */
    op.boundary = 1.0f;     /* 滑模边界层电流误差 A */

    mcl_config_default(&cfg);
    cfg.current_loop_freq_hz = 10000;
    cfg.max_duty = 1.0f;
    cfg.bus_voltage = 2.0f;
    cfg.current_pid.ki = 200.0f;
    cfg.current_pid.out_min = -2.0f;  cfg.current_pid.out_max = 2.0f;
    cfg.current_pid.i_min = -2.0f;    cfg.current_pid.i_max = 2.0f;
    cfg.speed_pid.kp = 0.3f;    /* 适中速度环增益 */
    cfg.speed_pid.ki = 10.0f;
    cfg.openloop_rpm = 0.0f;  /* 禁用自动开环，纯闭环测 SMO */
    cfg.limits.overcurrent = 100.0f;  cfg.limits.overvoltage = 100.0f;
    cfg.limits.undervoltage = -100.0f; cfg.limits.overtemp = 1000.0f;
    cfg.limits.stall_speed = 0.0f;    cfg.limits.stall_time = 100.0f;

    mcl_init(&motor, &cfg, &hal, &hctx, &mcl_observer_smo_ops, &obs, &op);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);

    /* 阶段 1：维持 238.7 rpm 0.3s，让 SMO + PLL 收敛 */
    mcl_set_speed(&motor, 238.7f);
    mcl_start(&motor);
    for (i = 0; i < 3000; i++) { mcl_control_tick(&motor); }
    rpm_final = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;
    phase_err = hctx.motor.theta_e - motor.phase_rad;
    while (phase_err > 3.14159265f) { phase_err -= 6.283185f; }
    while (phase_err < -3.14159265f) { phase_err += 6.283185f; }
    printf("  阶段1(恒速238.7rpm)后：rpm=%.1f 相位误差=%.3f rad\n", rpm_final, phase_err);

    /* 阶段 2：加速到 500 rpm */
    mcl_set_speed(&motor, 500.0f);
    for (i = 0; i < 10000; i++) { mcl_control_tick(&motor); }

    rpm_final = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;
    phase_err = hctx.motor.theta_e - motor.phase_rad;
    while (phase_err > 3.14159265f) { phase_err -= 6.283185f; }
    while (phase_err < -3.14159265f) { phase_err += 6.283185f; }

    printf("===== SMO 无感速度环闭环（float）=====\n");
    printf("  目标 500 rpm，稳态转速 = %.1f rpm，相位误差 = %.3f rad\n", rpm_final, phase_err);
    printf("  状态 state=%d fault=%d\n", (int)motor.state, (int)motor.fault);

    return 0;
}
