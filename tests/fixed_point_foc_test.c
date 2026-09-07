/**
 * @file    fixed_point_foc_test.c
 * @brief   mcl 定点 FOC 电流环闭环验证（per-unit 归一化，float/Q15/Q31 三精度）
 *
 * 验证完整电流环闭环在定点下的正确性：采样归一化 → Clarke/Park →
 * 电流环 PI → 反 Park → SVPWM → 电机电流。固定编码器相位（θ=0）且 speed=0，
 * 避开速度环 rpm 换算与机械方程 J 归一化，聚焦 FOC 核心。
 *
 * per-unit 基值（严格满足 V_BASE = W_BASE·λ_BASE = R_BASE·I_BASE，且 V_BASE = V_BUS
 * 使 SVPWM 母线归一化 == 电压 per-unit，无需额外 scale）：
 *   V_BASE = V_BUS = 2 V、I_BASE = 5 A、λ_BASE = 0.02 Wb → W_BASE = 100 rad/s
 *   电机 R=0.1Ω→R_pu=0.25、L=1mH→L_pu=0.25、λ=0.02→λ_pu=1.0
 *   T_BASE = 1/W_BASE = 0.01 s → dt_pu = 0.0001/0.01 = 0.01
 */

#include "mcl.h"
#include <stdio.h>

#define I_BASE        5.0f
#define V_BASE        2.0f           /* = 母线电压 */
#define LAMBDA_BASE   0.02f
#define W_BASE        (V_BASE / LAMBDA_BASE)   /* 100 */
#define R_BASE        (V_BASE / I_BASE)        /* 0.4 */
#define L_BASE        (V_BASE / (W_BASE * I_BASE)) /* 0.004 */

#define R_PU          (0.1f / R_BASE)          /* 0.25 */
#define L_PU          (0.001f / L_BASE)        /* 0.25 */

#define N 15000

typedef struct { mcl_scalar i_alpha, i_beta; } pu_motor_t;
typedef struct { pu_motor_t motor; mcl_scalar v_alpha, v_beta; } pu_hal_t;

static mcl_scalar pu_R(void) { return MCL_FROM_FLOAT(R_PU); }
static mcl_scalar pu_L(void) { return MCL_FROM_FLOAT(L_PU); }

/* per-unit 电气方程 di/dt = (v - R·i)/L（θ=0 → 反电势=0） */
static void pu_motor_step(pu_motor_t *m, pu_hal_t *h)
{
    mcl_scalar dt_pu = MCL_FROM_FLOAT(0.0001f / (1.0f / W_BASE));  /* 0.01 */
    m->i_alpha = MCL_ADD(m->i_alpha,
        MCL_MUL(MCL_DIV(MCL_SUB(h->v_alpha, MCL_MUL(pu_R(), m->i_alpha)), pu_L()), dt_pu));
    m->i_beta = MCL_ADD(m->i_beta,
        MCL_MUL(MCL_DIV(MCL_SUB(h->v_beta, MCL_MUL(pu_R(), m->i_beta)), pu_L()), dt_pu));
}

/* SVPWM duty [0,1] → per-unit 相电压（[-1,1]，1=满母线=V_BASE）。
   映射 2·duty-1 需 ×2（定点下移位），HAL 边界用 float 中转（M4F/M7 有 FPU 零成本） */
static void pu_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    pu_hal_t *h = (pu_hal_t *)ctx;
    float va = 2.0f * MCL_TO_FLOAT(da) - 1.0f;
    float vb = 2.0f * MCL_TO_FLOAT(db) - 1.0f;
    float vc = 2.0f * MCL_TO_FLOAT(dc) - 1.0f;
    h->v_alpha = MCL_FROM_FLOAT(va);
    h->v_beta = MCL_FROM_FLOAT((vb - vc) * 0.5773503f);
}

static int pu_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    pu_hal_t *h = (pu_hal_t *)ctx;
    pu_motor_step(&h->motor, h);
    *ia = h->motor.i_alpha;
    *ib = MCL_ADD(MCL_MUL(MCL_FROM_FLOAT(-0.5f), h->motor.i_alpha),
                  MCL_MUL(MCL_FROM_FLOAT(0.8660254f), h->motor.i_beta));
    *ic = MCL_SUB(MCL_MUL(MCL_FROM_FLOAT(-0.5f), h->motor.i_alpha),
                  MCL_MUL(MCL_FROM_FLOAT(0.8660254f), h->motor.i_beta));
    return MCL_OK;
}

static int pu_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;
    *vbus = MCL_FROM_FLOAT(1.0f);   /* V_BUS/V_BASE = 1 */
    *ibus = (mcl_scalar)0;
    return MCL_OK;
}

static int pu_angle(void *ctx, mcl_scalar *angle)
{
    (void)ctx; *angle = (mcl_scalar)0;
    return MCL_OK;
}

static int pu_speed(void *ctx, mcl_scalar *speed)
{
    (void)ctx; *speed = (mcl_scalar)0;
    return MCL_OK;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    pu_hal_t hctx;
    mcl_config cfg;
    mcl_scalar iq_final;
    int i;

    hctx.motor.i_alpha = (mcl_scalar)0;
    hctx.motor.i_beta = (mcl_scalar)0;
    hctx.v_alpha = (mcl_scalar)0;
    hctx.v_beta = (mcl_scalar)0;

    hal.pwm_set_duty = pu_pwm;
    hal.adc_read_phase = pu_adc;
    hal.adc_read_bus = pu_bus;
    hal.enc_read_angle = pu_angle;
    hal.enc_read_speed = pu_speed;
    hal.read_temp = NULL;
    hal.micros = NULL;

    mcl_config_default(&cfg);
    cfg.current_loop_freq_hz = 10000;
    cfg.max_duty = MCL_FROM_FLOAT(1.0f);
    cfg.bus_voltage = MCL_FROM_FLOAT(1.0f);          /* V_BUS/V_BASE = 1 */
    cfg.time_base = MCL_FROM_FLOAT(1.0f / W_BASE);   /* 0.01 */
    cfg.phase_resistance = MCL_FROM_FLOAT(R_PU);     /* 0.25 */
    cfg.phase_inductance = MCL_FROM_FLOAT(L_PU);     /* 0.25 */
    cfg.rated_current = MCL_FROM_FLOAT(1.0f);        /* 5A/5A */
    cfg.current_pid.kp = MCL_FROM_FLOAT(0.5f);
    cfg.current_pid.ki = MCL_FROM_FLOAT(0.3f);      /* <1（Q15 不饱和），ki·dt=0.003 */
    cfg.current_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg.current_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg.current_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg.current_pid.i_max = MCL_FROM_FLOAT(1.0f);
    cfg.limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg.limits.overvoltage = MCL_FROM_FLOAT(10.0f);
    cfg.limits.undervoltage = MCL_FROM_FLOAT(-10.0f);
    cfg.limits.overtemp = MCL_FROM_FLOAT(1000.0f);
    /* 禁用堵转保护：本测试固定相位 speed=0（测电流环，转子不转），否则被判堵转关断 */
    cfg.limits.stall_speed = (mcl_scalar)0;
    cfg.limits.stall_time = MCL_FROM_FLOAT(100000.0f);

    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_current(&motor, MCL_FROM_FLOAT(0.5f));   /* iq 目标 0.5 pu */
    mcl_start(&motor);

    for (i = 0; i < N; i++) { mcl_control_tick(&motor); }
    iq_final = motor.iq_now;

#if defined(MCL_USE_Q15)
    printf("===== FOC 电流环定点闭环（Q15）=====\n");
#elif defined(MCL_USE_Q31)
    printf("===== FOC 电流环定点闭环（Q31）=====\n");
#else
    printf("===== FOC 电流环定点闭环（float）=====\n");
#endif
    printf("  iq 目标 = 0.500 pu，稳态 iq = %.4f pu，稳态 id = %.4f pu（iq 应趋近 0.5，id≈0）\n",
           MCL_TO_FLOAT(iq_final), MCL_TO_FLOAT(motor.id_now));

    return 0;
}
