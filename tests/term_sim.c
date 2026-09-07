/**
 * @file    term_sim.c
 * @brief   mcl 电机控制库终端可视化仿真（ASCII 波形）
 *
 * 有感 FOC 速度环：目标 500 rpm，终端直接画转速/电流波形。
 * 编译（工程根目录）：
 *   gcc -std=c99 -Iinclude tests/term_sim.c src -lm -o tests/term_sim.exe
 */

#include "mcl.h"
#include "mcl_observer_flux.h"
#include "mcl_observer_ortega.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============================ 终端 ASCII 波形 ============================ */

#define TW 76     /* 宽度（列） */
#define TH 20     /* 高度（行） */

static void term_plot(const char *title, const float *data, int n,
                      float ymin, float ymax)
{
    char grid[TH][TW];
    int r, c;

    for (r = 0; r < TH; r++)
        for (c = 0; c < TW; c++)
            grid[r][c] = ' ';

    for (c = 0; c < TW; c++)
    {
        int i0 = c * n / TW;
        int i1 = (c + 1) * n / TW;
        float lo, hi;
        int i, r_lo, r_hi, r_top, r_bot;

        if (i1 <= i0) { i1 = i0 + 1; }
        if (i1 > n) { i1 = n; }

        lo = hi = data[i0];
        for (i = i0; i < i1; i++)
        {
            if (data[i] < lo) { lo = data[i]; }
            if (data[i] > hi) { hi = data[i]; }
        }

        r_lo = TH - 1 - (int)((lo - ymin) / (ymax - ymin) * (TH - 1));
        r_hi = TH - 1 - (int)((hi - ymin) / (ymax - ymin) * (TH - 1));
        if (r_lo < 0) { r_lo = 0; }
        if (r_lo >= TH) { r_lo = TH - 1; }
        if (r_hi < 0) { r_hi = 0; }
        if (r_hi >= TH) { r_hi = TH - 1; }

        r_top = (r_hi < r_lo) ? r_hi : r_lo;
        r_bot = (r_hi > r_lo) ? r_hi : r_lo;
        for (r = r_top; r <= r_bot; r++) { grid[r][c] = '#'; }
    }

    printf("\n  %s  [%.2f .. %.2f]\n", title, ymin, ymax);
    for (r = 0; r < TH; r++)
    {
        printf("  |");
        for (c = 0; c < TW; c++) { putchar(grid[r][c]); }
        printf("|\n");
    }
    printf("  +");
    for (c = 0; c < TW; c++) { putchar('-'); }
    printf("+\n");
}

/* ============================ PMSM 电机模型（有感） ============================ */

typedef struct
{
    float R, L, lambda, J, pole_pairs;
    float i_alpha, i_beta;
    float theta_e, omega_e;
} pmotor_t;

typedef struct
{
    pmotor_t motor;
    float v_alpha, v_beta;
    float dt;
} hal_ctx_t;

static void motor_step(pmotor_t *m, float v_alpha, float v_beta, float dt)
{
    float e_alpha = -m->omega_e * m->lambda * sinf(m->theta_e);
    float e_beta = m->omega_e * m->lambda * cosf(m->theta_e);
    float iq, te, tl, omega_m;

    m->i_alpha += (v_alpha - m->R * m->i_alpha - e_alpha) * dt / m->L;
    m->i_beta += (v_beta - m->R * m->i_beta - e_beta) * dt / m->L;

    iq = -m->i_alpha * sinf(m->theta_e) + m->i_beta * cosf(m->theta_e);
    te = 1.5f * m->pole_pairs * m->lambda * iq;
    tl = 0.0f;

    omega_m = m->omega_e / m->pole_pairs;
    omega_m += (te - tl) / m->J * dt;
    m->omega_e = omega_m * m->pole_pairs;
    m->theta_e += m->omega_e * dt;
    while (m->theta_e > MCL_TWO_PI) { m->theta_e -= MCL_TWO_PI; }
}

static void hal_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    hal_ctx_t *h = (hal_ctx_t *)ctx;
    float va = 2.0f * da - 1.0f;
    float vb = 2.0f * db - 1.0f;
    float vc = 2.0f * dc - 1.0f;
    h->v_alpha = va;
    h->v_beta = (vb - vc) / 1.7320508f;
}

static int hal_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    hal_ctx_t *h = (hal_ctx_t *)ctx;
    motor_step(&h->motor, h->v_alpha, h->v_beta, h->dt);
    *ia = h->motor.i_alpha;
    *ib = (-h->motor.i_alpha + 1.7320508f * h->motor.i_beta) / 2.0f;
    *ic = (-h->motor.i_alpha - 1.7320508f * h->motor.i_beta) / 2.0f;
    return MCL_OK;
}

static int hal_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;
    *vbus = 2.0f;
    *ibus = 0.0f;
    return MCL_OK;
}

static int hal_angle(void *ctx, mcl_scalar *angle)
{
    hal_ctx_t *h = (hal_ctx_t *)ctx;
    *angle = h->motor.theta_e;
    return MCL_OK;
}

static int hal_speed(void *ctx, mcl_scalar *speed)
{
    hal_ctx_t *h = (hal_ctx_t *)ctx;
    *speed = h->motor.omega_e;
    return MCL_OK;
}

/* ============================ main ============================ */

#define SIM_N  6000   /* 采样点数（0.6s @ 10kHz） */

/* 电机模型 + mock HAL 初始化（有感/无感通用，角度/速度回调真实读出） */
static void model_setup(hal_ctx_t *hctx, mcl_hal_ops *hal)
{
    hctx->motor.R = 1.0f;
    hctx->motor.L = 0.001f;
    hctx->motor.lambda = 0.02f;
    hctx->motor.J = 0.0001f;
    hctx->motor.pole_pairs = 4.0f;
    hctx->motor.i_alpha = 0.0f;
    hctx->motor.i_beta = 0.0f;
    hctx->motor.theta_e = 0.0f;
    hctx->motor.omega_e = 0.0f;
    hctx->v_alpha = 0.0f;
    hctx->v_beta = 0.0f;
    hctx->dt = 0.0001f;

    hal->pwm_set_duty = hal_pwm;
    hal->adc_read_phase = hal_adc;
    hal->adc_read_bus = hal_bus;
    hal->enc_read_angle = hal_angle;
    hal->enc_read_speed = hal_speed;
    hal->read_temp = NULL;
    hal->micros = NULL;
}

static void cfg_relax(mcl_config *cfg)
{
    mcl_config_default(cfg);
    cfg->current_loop_freq_hz = 10000;
    cfg->max_duty = 1.0f;
    cfg->bus_voltage = 2.0f;
    cfg->current_pid.ki = 200.0f;
    cfg->current_pid.out_min = -2.0f;
    cfg->current_pid.out_max = 2.0f;
    cfg->current_pid.i_min = -2.0f;
    cfg->current_pid.i_max = 2.0f;
    /* 仿真放宽保护（避免 2V 母线触发欠压/过流） */
    cfg->limits.overcurrent = 100.0f;
    cfg->limits.overvoltage = 100.0f;
    cfg->limits.undervoltage = -100.0f;
    cfg->limits.overtemp = 1000.0f;
    cfg->limits.stall_speed = 0.0f;
    cfg->limits.stall_time = 100.0f;
}

static float rpm_of(hal_ctx_t *h)
{
    return h->motor.omega_e / h->motor.pole_pairs * 9.5492966f;
}

static float iq_of(hal_ctx_t *h)
{
    return -h->motor.i_alpha * sinf(h->motor.theta_e)
         + h->motor.i_beta * cosf(h->motor.theta_e);
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    hal_ctx_t hctx;
    mcl_config cfg;

    static float rpm[SIM_N];
    static float iq[SIM_N];
    int i;

    model_setup(&hctx, &hal);
    cfg_relax(&cfg);

    /* ============ 场景 1：有感 FOC 速度环（闭环） ============ */
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_speed(&motor, 500.0f);
    mcl_start(&motor);
    for (i = 0; i < SIM_N; i++)
    {
        mcl_control_tick(&motor);
        rpm[i] = rpm_of(&hctx);
        iq[i] = iq_of(&hctx);
    }
    printf("======== 场景 1：有感 FOC 速度环（闭环） ========\n");
    printf("目标 500 rpm，稳态转速 = %.1f rpm，稳态 iq = %.3f A\n",
           rpm[SIM_N - 1], iq[SIM_N - 1]);
    term_plot("转速响应 rpm（目标 500）", rpm, SIM_N, 0.0f, 600.0f);
    term_plot("q 轴电流 iq A（加速段为正转矩）", iq, SIM_N, -0.5f, 2.5f);

    /* ============ 场景 2：开环旋转电压矢量（V/F，全开环） ============ */
    model_setup(&hctx, &hal);
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_openloop_vf(&motor, 1.0f, 100.0f);   /* 满电压幅值，目标 100 rpm */
    mcl_start(&motor);
    for (i = 0; i < SIM_N; i++)
    {
        mcl_control_tick(&motor);
        rpm[i] = rpm_of(&hctx);
        iq[i] = iq_of(&hctx);
    }
    printf("\n======== 场景 2：开环旋转电压矢量 V/F（全开环） ========\n");
    printf("电压幅值 1.0（满母线），目标 100 rpm，稳态转速 = %.1f rpm\n",
           rpm[SIM_N - 1]);
    term_plot("开环 VF 转速 rpm（相位斜坡 100 rpm，转子拖到反电势平衡）",
              rpm, SIM_N, 0.0f, 150.0f);

    /* ============ 场景 3：开环固定电流矢量（转子预定位） ============ */
    model_setup(&hctx, &hal);
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
    mcl_set_openloop_align(&motor, 2.0f, 0.0f);  /* 2A 拉到 0 电气角 */
    mcl_start(&motor);
    for (i = 0; i < 1000; i++)
    {
        mcl_control_tick(&motor);
    }
    printf("\n======== 场景 3：开环固定电流矢量（预定位） ========\n");
    printf("2A 拉到 0 rad，转子最终电气角 = %.4f rad（应趋近 0）\n",
           hctx.motor.theta_e);

    /* ============ 场景 4：无感零速启动（自动开环→闭环切换） ============ */
    {
        mcl_observer_ortega obs;
        mcl_observer_ortega_params op;
        float phase_err;

        op.lambda = 0.02f;
        op.resistance = 1.0f;
        op.inductance = 0.001f;
        op.gain = 600000.0f;   /* ORTEGA 增益 γ（≈ 600/L） */

        model_setup(&hctx, &hal);   /* 从零转速、零相位启动 */
        cfg.openloop_rpm = 150.0f;  /* 调低开环上限，让"切闭环"更明显 */
        cfg.openloop_time_lock = 0.05f;  /* 预定位 */
        cfg.openloop_time_ramp = 0.15f;  /* 慢斜坡，避免转子冲过头 */
        cfg.openloop_time = 0.10f;       /* 匀速保持，让转子稳定跟上磁场 */
        mcl_init(&motor, &cfg, &hal, &hctx, &mcl_observer_ortega_ops, &obs, &op);
        mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);
        mcl_set_speed(&motor, 500.0f);
        mcl_start(&motor);
        for (i = 0; i < SIM_N; i++)
        {
            mcl_control_tick(&motor);
            rpm[i] = rpm_of(&hctx);
        }
        phase_err = hctx.motor.theta_e - motor.phase_rad;
        while (phase_err > MCL_PI) { phase_err -= MCL_TWO_PI; }
        while (phase_err < -MCL_PI) { phase_err += MCL_TWO_PI; }

        /* 再跑 0.4s 观察是否收敛（不采样） */
        for (i = 0; i < 4000; i++) { mcl_control_tick(&motor); }
        {
            float pe2 = hctx.motor.theta_e - motor.phase_rad;
            while (pe2 > MCL_PI) { pe2 -= MCL_TWO_PI; }
            while (pe2 < -MCL_PI) { pe2 += MCL_TWO_PI; }
            printf("\n======== 场景 4：无感零速启动（自动开环→闭环切换） ========\n");
            printf("从 0 启动到 500 rpm，0.6s 时转速 = %.1f rpm，1.0s 时转速 = %.1f rpm\n",
                   rpm[SIM_N - 1], rpm_of(&hctx));
            printf("相位误差：0.6s = %.3f rad，1.0s = %.3f rad\n", phase_err, pe2);
        }
        term_plot("无感零速启动 rpm（自动开环拖动 → 切闭环，openloop_rpm=150）",
                  rpm, SIM_N, 0.0f, 600.0f);
    }

    /* ============ 场景 5：ORTEGA 无感速度环（稳态跟踪，不经开环） ============ */
    {
        mcl_observer_ortega obs;
        mcl_observer_ortega_params op;
        float pe;

        op.lambda = 0.02f;
        op.resistance = 1.0f;
        op.inductance = 0.001f;
        op.gain = 600000.0f;

        model_setup(&hctx, &hal);
        hctx.motor.omega_e = 100.0f;   /* 初始电气速度 100 rad/s ≈ 238.7 rpm */
        cfg.openloop_rpm = 0.0f;       /* 禁用自动开环，测纯闭环稳态跟踪 */
        mcl_init(&motor, &cfg, &hal, &hctx, &mcl_observer_ortega_ops, &obs, &op);
        mcl_set_mode(&motor, MCL_MODE_FOC_SENSORLESS);

        /* 阶段 1：先维持当前转速 0.3s，让观测器/PLL 收敛（避免初始相位瞬态） */
        mcl_set_speed(&motor, 238.7f);
        mcl_start(&motor);
        for (i = 0; i < 3000; i++) { mcl_control_tick(&motor); }

        /* 阶段 2：加速到 500 rpm */
        mcl_set_speed(&motor, 500.0f);
        for (i = 0; i < SIM_N; i++)
        {
            mcl_control_tick(&motor);
            rpm[i] = rpm_of(&hctx);
        }
        pe = hctx.motor.theta_e - motor.phase_rad;
        while (pe > MCL_PI) { pe -= MCL_TWO_PI; }
        while (pe < -MCL_PI) { pe += MCL_TWO_PI; }

        /* 再跑 0.4s 观察收敛 */
        for (i = 0; i < 4000; i++) { mcl_control_tick(&motor); }
        {
            float pe2 = hctx.motor.theta_e - motor.phase_rad;
            while (pe2 > MCL_PI) { pe2 -= MCL_TWO_PI; }
            while (pe2 < -MCL_PI) { pe2 += MCL_TWO_PI; }
            printf("\n======== 场景 5：ORTEGA 无感速度环（稳态跟踪） ========\n");
            printf("从 238.7 rpm 拉到 500 rpm：0.6s 转速 = %.1f rpm，1.0s 转速 = %.1f rpm\n",
                   rpm[SIM_N - 1], rpm_of(&hctx));
            printf("相位误差：0.6s = %.3f rad，1.0s = %.3f rad\n", pe, pe2);
        }
        term_plot("ORTEGA 无感速度环 rpm（目标 500，观测器相位持续收敛）",
                  rpm, SIM_N, 0.0f, 600.0f);
    }

    return 0;
}
