/**
 * @file    fixed_point_position_test.c
 * @brief   mcl 定点有感位置环闭环验证（float/Q15/Q31 三精度）
 *
 * 有感 FOC 位置环：编码器反馈机械角，级联速度环 + 电流环，跟踪目标机械角。
 * 之前位置环只在 float 的 sim_test 里测过，此处补三精度 per-unit 验证。
 *
 * 归一化基值（同 fixed_point_speed_test）：
 *   W_BASE = 400 rad/s、V_BASE = 8V、λ_BASE = 0.02 Wb、I_BASE = 10 A、RPM_BASE = 954.9 rpm。
 * 角度约定：float=rad，定点=「圈」(1.0=2π)。目标机械角 1 rad → 定点 1/(2π) 圈。
 */

#include "mcl.h"
#include <stdio.h>
#include <math.h>

#define I_BASE        10.0f
#define V_BASE        8.0f
#define LAMBDA_BASE   0.02f
#define W_BASE        (V_BASE / LAMBDA_BASE)
#define RPM_BASE      (W_BASE / 4.0f * 60.0f / (2.0f * 3.14159265358979f))
#define TWO_PI_F      (2.0f * 3.14159265358979f)

#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    #define K_I      (1.0f / I_BASE)
    #define K_V      (V_BASE)
    #define K_W      (1.0f / W_BASE)
    #define K_T      (1.0f / W_BASE)
    #define K_ANG    (1.0f / TWO_PI_F)
    #define K_R      (1.0f / (V_BASE / I_BASE))
    #define K_L      (1.0f / (V_BASE / (W_BASE * I_BASE)))
    #define K_LAM    (1.0f / LAMBDA_BASE)
#else
    #define K_I      (1.0f)
    #define K_V      (1.0f)
    #define K_W      (1.0f)
    #define K_T      (1.0f)
    #define K_ANG    (1.0f)
    #define K_R      (1.0f)
    #define K_L      (1.0f)
    #define K_LAM    (1.0f)
#endif

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
    wm = m->omega_e / m->pole_pairs; wm += te / m->J * dt;
    m->omega_e = wm * m->pole_pairs; m->theta_e += m->omega_e * dt;
    while (m->theta_e > TWO_PI_F) { m->theta_e -= TWO_PI_F; }
}

static void hal_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    hal_t *h = (hal_t *)ctx;
    float va = (2.0f * MCL_TO_FLOAT(da) - 1.0f) * K_V;
    float vb = (2.0f * MCL_TO_FLOAT(db) - 1.0f) * K_V;
    float vc = (2.0f * MCL_TO_FLOAT(dc) - 1.0f) * K_V;
    h->v_alpha = va; h->v_beta = (vb - vc) / 1.7320508f;
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

static int hal_angle(void *ctx, mcl_scalar *angle)
{
    hal_t *h = (hal_t *)ctx;
    /* 编码器返回电气角：float=rad，定点=圈 */
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
    float pos_final, pos_target;
    int i;

    hctx.motor.R = 0.1f; hctx.motor.L = 0.001f; hctx.motor.lambda = 0.02f;
    hctx.motor.J = 0.0005f; hctx.motor.pole_pairs = 4.0f;
    hctx.motor.i_alpha = 0; hctx.motor.i_beta = 0; hctx.motor.theta_e = 0; hctx.motor.omega_e = 0;
    hctx.v_alpha = 0; hctx.v_beta = 0;

    hal.pwm_set_duty = hal_pwm;
    hal.adc_read_phase = hal_adc;
    hal.adc_read_bus = hal_bus;
    hal.enc_read_angle = hal_angle;
    hal.enc_read_speed = hal_speed;
    hal.read_temp = NULL; hal.micros = NULL;

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
    /* 位置环 PID：输出速度参考 rpm。
       float 输出物理 rpm（可达数百）；定点输出 rpm_pu（≤1），参数 per-unit。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    cfg.pos_pid.kp = MCL_FROM_FLOAT(20.0f);
    cfg.pos_pid.ki = MCL_FROM_FLOAT(0.5f);
    cfg.pos_pid.kd = (mcl_scalar)0;
    cfg.pos_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg.pos_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg.pos_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg.pos_pid.i_max = MCL_FROM_FLOAT(1.0f);
#else
    cfg.pos_pid.kp = MCL_FROM_FLOAT(100.0f);
    cfg.pos_pid.ki = MCL_FROM_FLOAT(20.0f);
    cfg.pos_pid.kd = (mcl_scalar)0;
    cfg.pos_pid.out_min = MCL_FROM_FLOAT(-500.0f);
    cfg.pos_pid.out_max = MCL_FROM_FLOAT(500.0f);
    cfg.pos_pid.i_min = MCL_FROM_FLOAT(-500.0f);
    cfg.pos_pid.i_max = MCL_FROM_FLOAT(500.0f);
#endif
    cfg.limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg.limits.overvoltage = MCL_FROM_FLOAT(10.0f);
    cfg.limits.undervoltage = MCL_FROM_FLOAT(-10.0f);
    cfg.limits.overtemp = MCL_FROM_FLOAT(1000.0f);
    cfg.limits.stall_speed = (mcl_scalar)0;
    cfg.limits.stall_time = MCL_FROM_FLOAT(100000.0f);

    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);

    /* 目标机械角 1 rad；定点下机械角用「圈」：1 rad = 1/(2π) 圈 */
    pos_target = 1.0f;
    mcl_set_position(&motor, MCL_FROM_FLOAT(pos_target * K_ANG));
    mcl_start(&motor);

    for (i = 0; i < 20000; i++) { mcl_control_tick(&motor); }

    /* 机械角（float=rad；定点=圈×2π 折回 rad 对比） */
    pos_final = hctx.motor.theta_e / hctx.motor.pole_pairs;   /* rad */
    /* 归一化到 [0, 2π) 再取近 1 rad 的最小角差 */
    while (pos_final >= TWO_PI_F) { pos_final -= TWO_PI_F; }
    while (pos_final < 0.0f) { pos_final += TWO_PI_F; }

    printf("===== 定点有感位置环闭环（%s）=====\n",
#if defined(MCL_USE_Q15)
           "Q15"
#elif defined(MCL_USE_Q31)
           "Q31"
#else
           "float"
#endif
    );
    printf("  目标机械角 = %.2f rad，稳态机械角 = %.3f rad\n", pos_target, pos_final);

    return 0;
}
