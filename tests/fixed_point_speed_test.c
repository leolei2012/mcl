/**
 * @file    fixed_point_speed_test.c
 * @brief   mcl 定点 FOC 有感速度环闭环验证（float/Q15/Q31 三精度）
 *
 * 电机模型用 float 物理量（含机械方程），HAL 边界做归一化/反归一化。
 * float 模式传物理量、定点模式传 per-unit（用条件编译分离），验证完整
 * 速度环（电流环+速度环+机械方程）在定点下三精度一致收敛到目标转速。
 *
 * per-unit 基值（V_BASE = W_BASE·λ_BASE，且 V_BASE = V_BUS）：
 *   W_BASE = 400 rad/s（电气）、V_BASE = 8 V（=母线）、I_BASE = 10 A、λ_BASE = 0.02 Wb
 *   RPM_BASE = W_BASE/p·60/(2π) = 954.9 rpm → 500 rpm = 0.5235 pu
 */

#include "mcl.h"
#include <stdio.h>
#include <math.h>

#define I_BASE        10.0f
#define V_BASE        8.0f
#define LAMBDA_BASE   0.02f
#define W_BASE        (V_BASE / LAMBDA_BASE)          /* 400 */
#define RPM_BASE      (W_BASE / 4.0f * 60.0f / (2.0f * 3.14159265358979f))

#define TARGET_RPM     500.0f
#define N              20000

/* ---- 归一化因子（float=1 / 定点=基值），集中在 HAL 边界用 ---- */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    #define K_I      (1.0f / I_BASE)      /* 电流 ÷I_BASE */
    #define K_V      (V_BASE)             /* 电压 ×V_BASE（反归一化） */
    #define K_W      (1.0f / W_BASE)      /* 速度 ÷W_BASE */
    #define K_T      (1.0f / W_BASE)      /* time_base = 1/W_BASE */
    #define K_ANG    (0.15915494309f)     /* 角度 ÷2π → 归一化圈 */
    #define K_SPDREF (1.0f / RPM_BASE)    /* rpm 参考 ÷RPM_BASE */
    #define K_R      (1.0f / (V_BASE / I_BASE))          /* R_pu 分母 */
    #define K_L      (1.0f / (V_BASE / (W_BASE * I_BASE)))/* L_pu 分母 */
    #define K_LAM    (1.0f / LAMBDA_BASE)                 /* λ_pu 分母 */
#else
    #define K_I      (1.0f)
    #define K_V      (1.0f)
    #define K_W      (1.0f)
    #define K_T      (1.0f)
    #define K_ANG    (1.0f)
    #define K_SPDREF (1.0f)
    #define K_R      (1.0f)
    #define K_L      (1.0f)
    #define K_LAM    (1.0f)
#endif

/* ---- 电机模型（float 物理量，含机械方程） ---- */
typedef struct
{
    float R, L, lambda, J, pole_pairs;
    float i_alpha, i_beta;
    float theta_e, omega_e;
} motor_t;

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
    while (m->theta_e > (2.0f * 3.14159265358979f)) { m->theta_e -= (2.0f * 3.14159265358979f); }
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
    *vbus = MCL_FROM_FLOAT(V_BASE * K_V / V_BASE);   /* V_BUS/V_BASE = 1（因 V_BASE=V_BUS） */
    *ibus = (mcl_scalar)0;
    return MCL_OK;
}

static int hal_angle(void *ctx, mcl_scalar *angle)
{
    hal_t *h = (hal_t *)ctx;
    *angle = MCL_FROM_FLOAT(h->motor.theta_e * K_ANG);
    return MCL_OK;
}

static int hal_speed(void *ctx, mcl_scalar *speed)
{
    hal_t *h = (hal_t *)ctx;
    *speed = MCL_FROM_FLOAT(h->motor.omega_e * K_W);
    return MCL_OK;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    hal_t hctx;
    mcl_config cfg;
    float rpm_final;
    int i;

    hctx.motor.R = 0.1f;
    hctx.motor.L = 0.001f;
    hctx.motor.lambda = 0.02f;
    hctx.motor.J = 0.0005f;
    hctx.motor.pole_pairs = 4.0f;
    hctx.motor.i_alpha = 0.0f;
    hctx.motor.i_beta = 0.0f;
    hctx.motor.theta_e = 0.0f;
    hctx.motor.omega_e = 0.0f;
    hctx.v_alpha = 0.0f;
    hctx.v_beta = 0.0f;

    hal.pwm_set_duty = hal_pwm;
    hal.adc_read_phase = hal_adc;
    hal.adc_read_bus = hal_bus;
    hal.enc_read_angle = hal_angle;
    hal.enc_read_speed = hal_speed;
    hal.read_temp = NULL;
    hal.micros = NULL;

    mcl_config_default(&cfg);
    cfg.pole_pairs = 4;
    cfg.current_loop_freq_hz = 10000;
    cfg.max_duty = MCL_FROM_FLOAT(1.0f);
    cfg.bus_voltage = MCL_FROM_FLOAT(V_BASE * K_V / V_BASE); /* V_BUS/V_BASE = 1 */
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
    cfg.speed_pid.ki = MCL_FROM_FLOAT(0.1f);
    cfg.speed_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg.speed_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg.speed_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg.speed_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg.limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg.limits.overvoltage = MCL_FROM_FLOAT(10.0f);
    cfg.limits.undervoltage = MCL_FROM_FLOAT(-10.0f);
    cfg.limits.overtemp = MCL_FROM_FLOAT(1000.0f);
    cfg.limits.stall_speed = (mcl_scalar)0;
    cfg.limits.stall_time = MCL_FROM_FLOAT(100000.0f);

    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_speed(&motor, MCL_FROM_FLOAT(TARGET_RPM * K_SPDREF));
    mcl_start(&motor);

    for (i = 0; i < N; i++) { mcl_control_tick(&motor); }

    rpm_final = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;

#if defined(MCL_USE_Q15)
    printf("===== 定点 FOC 有感速度环闭环（Q15）=====\n");
#elif defined(MCL_USE_Q31)
    printf("===== 定点 FOC 有感速度环闭环（Q31）=====\n");
#else
    printf("===== 定点 FOC 有感速度环闭环（float）=====\n");
#endif
    printf("  目标 = %.0f rpm，稳态转速 = %.1f rpm（应趋近 %.0f）\n",
           TARGET_RPM, rpm_final, TARGET_RPM);

    return 0;
}
