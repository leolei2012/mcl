/**
 * @file    sim_test.c
 * @brief   mcl 电机控制库 PC 端仿真测试（float 精度）
 *
 * 编译（在工程根目录）：
 *   gcc -std=c99 -Iinclude tests/sim_test.c \
 *       src/mcl_math.c src/mcl_transform.c src/mcl_pid.c src/mcl_svpwm.c \
 *       src/mcl_pll.c src/mcl_observer.c src/mcl_observer_flux.c \
 *       -lm -o tests/sim_test
 *
 * 运行：./tests/sim_test
 * 产物：tests/sim_output.html（浏览器打开查看波形）
 */

#include "mcl.h"
#include "mcl_observer_flux.h"
#include "mcl_observer_smo.h"
#include "mcl_observer_ortega.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#define PLOT_W   640
#define PLOT_H   200
#define PLOT_N   1024

/* ============================ SVG 绘图辅助 ============================ */

static void svg_begin(FILE *f, const char *title)
{
    fprintf(f, "<h3>%s</h3>\n", title);
    fprintf(f, "<svg width=\"%d\" height=\"%d\" xmlns=\"http://www.w3.org/2000/svg\" "
               "style=\"background:#0d1117;border:1px solid #30363d;\">\n", PLOT_W, PLOT_H);
    /* 零线 */
    fprintf(f, "<line x1=\"0\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#555\" stroke-dasharray=\"4 4\"/>\n",
            PLOT_H / 2, PLOT_W, PLOT_H / 2);
}

static void svg_curve(FILE *f, const float *y, int n, float ymin, float ymax, const char *color)
{
    int i;
    fprintf(f, "<polyline points=\"");
    for (i = 0; i < n; i++)
    {
        float v = y[i];
        if (v < ymin) { v = ymin; }
        if (v > ymax) { v = ymax; }
        float px = (float)i / (float)(n - 1) * (float)PLOT_W;
        float py = (ymax - v) / (ymax - ymin) * (float)PLOT_H;
        fprintf(f, "%.1f,%.1f ", px, py);
    }
    fprintf(f, "\" fill=\"none\" stroke=\"%s\" stroke-width=\"1.5\"/>\n", color);
}

static void svg_end(FILE *f)
{
    fprintf(f, "</svg>\n");
}

/* ============================ 场景 1：Clark/Park 变换 ============================ */

static void test_transform(FILE *html)
{
    float id_avg = 0.0f;
    float iq_avg = 0.0f;
    float id_arr[PLOT_N];
    float iq_arr[PLOT_N];
    int i;

    printf("[1] Clark/Park 变换：三相平衡电流经变换后 dq 应为直流\n");
    for (i = 0; i < PLOT_N; i++)
    {
        float t = (float)i * 0.0001f;              /* 0~0.1024s */
        float phase = 100.0f * t;                  /* ω=100 rad/s */
        float ia = sinf(phase);
        float ib = sinf(phase - 2.0943951f);       /* -120° */
        float ic = sinf(phase + 2.0943951f);       /* +120° */
        mcl_scalar alpha, beta, id, iq;
        mcl_transform_clarke(ia, ib, ic, &alpha, &beta);
        mcl_transform_park(alpha, beta, phase, &id, &iq);
        id_arr[i] = id;
        iq_arr[i] = iq;
        id_avg += id;
        iq_avg += iq;
    }
    id_avg /= PLOT_N;
    iq_avg /= PLOT_N;

    printf("   id 均值 = %.4f（期望 0）\n", id_avg);
    printf("   iq 均值 = %.4f（期望 -1，取决于相序）\n", iq_avg);

    svg_begin(html, "场景 1：Clark/Park 变换后的 d/q 轴电流（应为直流）");
    svg_curve(html, id_arr, PLOT_N, -1.5f, 1.5f, "#58a6ff");
    svg_curve(html, iq_arr, PLOT_N, -1.5f, 1.5f, "#3fb950");
    svg_end(html);
}

/* ============================ 场景 2：PID 阶跃响应 ============================ */

static void test_pid(FILE *html)
{
    mcl_pid pid;
    mcl_pid_params p;
    float out[PLOT_N];
    float ref[PLOT_N];
    int i;

    p.kp = 1.0f;
    p.ki = 20.0f;
    p.kd = 0.0f;
    p.out_min = -1.0f;
    p.out_max = 1.0f;
    p.i_min = -1.0f;
    p.i_max = 1.0f;
    mcl_pid_init(&pid, &p);

    printf("[2] PID 阶跃响应（kp=1, ki=20）\n");
    for (i = 0; i < PLOT_N; i++)
    {
        float setpoint = 1.0f;
        float fb = (i == 0) ? 0.0f : out[i - 1];   /* 单位反馈 */
        float err = setpoint - fb;
        out[i] = mcl_pid_run(&pid, err, 0.001f);
        ref[i] = setpoint;
    }
    printf("   终值 = %.4f（期望趋近 1.0）\n", out[PLOT_N - 1]);

    svg_begin(html, "场景 2：PID 阶跃响应（绿=参考，蓝=输出）");
    svg_curve(html, ref, PLOT_N, -0.2f, 1.4f, "#3fb950");
    svg_curve(html, out, PLOT_N, -0.2f, 1.4f, "#58a6ff");
    svg_end(html);
}

/* ============================ 场景 3：SVPWM 波形 ============================ */

static void test_svpwm(FILE *html)
{
    float da[PLOT_N];
    float db[PLOT_N];
    float dc[PLOT_N];
    int i;

    printf("[3] SVPWM 调制（旋转电压矢量 → 三相马鞍波占空比）\n");
    for (i = 0; i < PLOT_N; i++)
    {
        float phase = (float)i / (float)PLOT_N * 2.0f * (float)MCL_PI;
        mcl_scalar va = 0.8f * cosf(phase);
        mcl_scalar vb = 0.8f * sinf(phase);
        mcl_scalar a, b, c;
        mcl_svpwm_run(va, vb, 1.0f, &a, &b, &c);
        da[i] = a;
        db[i] = b;
        dc[i] = c;
    }
    {
        float dmin = 1.0f;
        float dmax = 0.0f;
        for (i = 0; i < PLOT_N; i++)
        {
            if (da[i] < dmin) { dmin = da[i]; }
            if (db[i] < dmin) { dmin = db[i]; }
            if (dc[i] < dmin) { dmin = dc[i]; }
            if (da[i] > dmax) { dmax = da[i]; }
            if (db[i] > dmax) { dmax = db[i]; }
            if (dc[i] > dmax) { dmax = dc[i]; }
        }
        printf("   占空比范围检查：min=%.3f max=%.3f（应落在 [0,1]）\n", dmin, dmax);
    }

    svg_begin(html, "场景 3：SVPWM 三相占空比（马鞍波，蓝=A 绿=B 红=C）");
    svg_curve(html, da, PLOT_N, -0.05f, 1.05f, "#58a6ff");
    svg_curve(html, db, PLOT_N, -0.05f, 1.05f, "#3fb950");
    svg_curve(html, dc, PLOT_N, -0.05f, 1.05f, "#f85149");
    svg_end(html);
}

/* ============================ 场景 4：PLL 相位跟踪 ============================ */

static void test_pll(FILE *html)
{
    mcl_pll pll;
    float err_arr[PLOT_N];
    int i;

    mcl_pll_init(&pll, 200.0f, 40000.0f);

    printf("[4] PLL 相位跟踪（输入斜坡相位，看误差收敛）\n");
    for (i = 0; i < PLOT_N; i++)
    {
        float t = (float)i * 0.0001f;
        float true_phase = 100.0f * t;
        float ph_out, spd_out;
        mcl_pll_run(&pll, true_phase, 0.0001f, &ph_out, &spd_out);
        /* 误差 wrap 到 [-π,π] */
        float err = true_phase - ph_out;
        while (err > (float)MCL_PI) { err -= (float)MCL_TWO_PI; }
        while (err < -(float)MCL_PI) { err += (float)MCL_TWO_PI; }
        err_arr[i] = err;
    }
    printf("   稳态相位误差 = %.5f rad（应趋近 0）\n", err_arr[PLOT_N - 1]);
    printf("   估计速度 = 待稳定后应趋近 100 rad/s\n");

    svg_begin(html, "场景 4：PLL 相位误差收敛（应趋近 0）");
    svg_curve(html, err_arr, PLOT_N, -0.2f, 0.2f, "#58a6ff");
    svg_end(html);
}

/* ============================ 场景 5：多观测器对比 ============================ */

/* 用给定观测器跑简化 PMSM 数据，输出估计相位与误差 */
static void run_observer(const mcl_observer_ops *ops, void *impl, void *params,
                         float lambda, float R, float L, float w, float iq, float dt,
                         float *est_arr, float *err_arr)
{
    mcl_observer carrier;
    int i;

    mcl_observer_init(&carrier, ops, impl, params);

    for (i = 0; i < PLOT_N; i++)
    {
        float t = (float)i * dt;
        float th = w * t;

        /* 电流：id=0, iq 恒定 */
        float i_alpha = -iq * sinf(th);
        float i_beta = iq * cosf(th);
        float di_alpha = -iq * w * cosf(th);
        float di_beta = -iq * w * sinf(th);

        /* 电压：v = R*i + L*di/dt + dλ/dt */
        float v_alpha = R * i_alpha + L * di_alpha - w * lambda * sinf(th);
        float v_beta = R * i_beta + L * di_beta + w * lambda * cosf(th);

        mcl_scalar phase_est;
        mcl_observer_update(&carrier, v_alpha, v_beta, i_alpha, i_beta, dt,
                            &phase_est, NULL);

        float pe = (float)phase_est;
        while (pe > (float)MCL_PI) { pe -= (float)MCL_TWO_PI; }
        while (pe < -(float)MCL_PI) { pe += (float)MCL_TWO_PI; }

        float err = th - pe;
        while (err > (float)MCL_PI) { err -= (float)MCL_TWO_PI; }
        while (err < -(float)MCL_PI) { err += (float)MCL_TWO_PI; }

        est_arr[i] = pe;
        err_arr[i] = err;
    }
}

static void test_observer(FILE *html)
{
    mcl_observer_flux obs_flux;
    mcl_observer_flux_params op_flux;
    mcl_observer_smo obs_smo;
    mcl_observer_smo_params op_smo;
    mcl_observer_ortega obs_ortega;
    mcl_observer_ortega_params op_ortega;

    float est_flux[PLOT_N];
    float err_flux[PLOT_N];
    float est_smo[PLOT_N];
    float err_smo[PLOT_N];
    float est_ortega[PLOT_N];
    float err_ortega[PLOT_N];
    float true_phase[PLOT_N];
    int i;

    float lambda = 0.02f;
    float R = 1.0f;
    float L = 0.001f;
    float w = 100.0f;
    float iq = 1.0f;
    float dt = 0.0001f;

    op_flux.lambda = lambda;
    op_flux.resistance = R;
    op_flux.inductance = L;
    op_flux.gain = 500.0f;   /* 幅值误差反馈增益 */

    op_smo.resistance = R;
    op_smo.inductance = L;
    op_smo.flux = lambda;      /* 磁链 ψ_f，速度估计 + 相位补偿 */
    op_smo.gain = 5.0f;       /* 滑模增益，需 > ω·λ = 2.0 */
    op_smo.lpf = 2000.0f;     /* 反电动势低通滤波 */
    op_smo.boundary = 0.05f;  /* 边界层厚度，抑制抖振 */

    op_ortega.lambda = lambda;
    op_ortega.resistance = R;
    op_ortega.inductance = L;
    op_ortega.gain = 600000.0f; /* ORTEGA 增益 γ（≈ 600/L，参考 VESC） */

    printf("[5] 多观测器对比：磁链 vs 滑模 vs ORTEGA\n");

    run_observer(&mcl_observer_flux_ops, &obs_flux, &op_flux,
                 lambda, R, L, w, iq, dt, est_flux, err_flux);
    run_observer(&mcl_observer_smo_ops, &obs_smo, &op_smo,
                 lambda, R, L, w, iq, dt, est_smo, err_smo);
    run_observer(&mcl_observer_ortega_ops, &obs_ortega, &op_ortega,
                 lambda, R, L, w, iq, dt, est_ortega, err_ortega);

    printf("   磁链观测器稳态误差 = %.5f rad\n", err_flux[PLOT_N - 1]);
    printf("   滑模观测器稳态误差 = %.5f rad\n", err_smo[PLOT_N - 1]);
    printf("   ORTEGA 观测器稳态误差 = %.5f rad\n", err_ortega[PLOT_N - 1]);

    for (i = 0; i < PLOT_N; i++)
    {
        true_phase[i] = w * (float)i * dt;
    }

    svg_begin(html, "场景 5：观测器估计相位（蓝=磁链，橙=滑模，红=ORTEGA，绿=真实）");
    svg_curve(html, est_flux, PLOT_N, -4.0f, 4.0f, "#58a6ff");
    svg_curve(html, est_smo, PLOT_N, -4.0f, 4.0f, "#d29922");
    svg_curve(html, est_ortega, PLOT_N, -4.0f, 4.0f, "#f85149");
    svg_curve(html, true_phase, PLOT_N, -4.0f, 4.0f, "#3fb950");
    svg_end(html);

    svg_begin(html, "场景 5b：观测器相位误差（蓝=磁链，橙=滑模，红=ORTEGA）");
    svg_curve(html, err_flux, PLOT_N, -0.5f, 0.5f, "#58a6ff");
    svg_curve(html, err_smo, PLOT_N, -0.5f, 0.5f, "#d29922");
    svg_curve(html, err_ortega, PLOT_N, -0.5f, 0.5f, "#f85149");
    svg_end(html);
}

/* ============================ 场景 6/7：FOC 无感闭环（速度环 / 位置环） ============================ */

/* PMSM 电机模型（αβ 域，含机械方程） */
typedef struct
{
    float R;
    float L;
    float lambda;
    float J;
    float pole_pairs;
    float i_alpha;
    float i_beta;
    float theta_e;      /* 电气角 */
    float omega_e;      /* 电角速度 rad/s */
} foc_motor_t;

typedef struct
{
    foc_motor_t motor;
    float v_alpha;
    float v_beta;
    float dt;
} foc_hal_t;

static void foc_motor_step(foc_motor_t *m, float v_alpha, float v_beta, float dt)
{
    float e_alpha = -m->omega_e * m->lambda * sinf(m->theta_e);
    float e_beta = m->omega_e * m->lambda * cosf(m->theta_e);
    m->i_alpha += (v_alpha - m->R * m->i_alpha - e_alpha) * dt / m->L;
    m->i_beta += (v_beta - m->R * m->i_beta - e_beta) * dt / m->L;

    /* 电磁转矩 Te = 1.5 · p · λ · iq */
    float iq = -m->i_alpha * sinf(m->theta_e) + m->i_beta * cosf(m->theta_e);
    float Te = 1.5f * m->pole_pairs * m->lambda * iq;
    float Tl = 0.0f;   /* 负载转矩 */

    /* 机械方程：dωm/dt = (Te - Tl) / J */
    float omega_m = m->omega_e / m->pole_pairs;
    omega_m += (Te - Tl) / m->J * dt;
    m->omega_e = omega_m * m->pole_pairs;
    m->theta_e += m->omega_e * dt;
    while (m->theta_e > MCL_TWO_PI) { m->theta_e -= MCL_TWO_PI; }
}

static void foc_pwm_set_duty(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    foc_hal_t *h = (foc_hal_t *)ctx;
    float va = 2.0f * da - 1.0f;
    float vb = 2.0f * db - 1.0f;
    float vc = 2.0f * dc - 1.0f;
    h->v_alpha = va;
    h->v_beta = (vb - vc) / 1.7320508f;
}

static int foc_adc_read_phase(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    foc_hal_t *h = (foc_hal_t *)ctx;
    foc_motor_step(&h->motor, h->v_alpha, h->v_beta, h->dt);
    *ia = h->motor.i_alpha;
    *ib = (-h->motor.i_alpha + 1.7320508f * h->motor.i_beta) / 2.0f;
    *ic = (-h->motor.i_alpha - 1.7320508f * h->motor.i_beta) / 2.0f;
    return MCL_OK;
}

static int foc_adc_read_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;
    *vbus = 2.0f;
    *ibus = 0.0f;
    return MCL_OK;
}

static int foc_enc_read_angle(void *ctx, mcl_scalar *angle)
{
    foc_hal_t *h = (foc_hal_t *)ctx;
    *angle = h->motor.theta_e;
    return MCL_OK;
}

static int foc_enc_read_speed(void *ctx, mcl_scalar *speed)
{
    foc_hal_t *h = (foc_hal_t *)ctx;
    *speed = h->motor.omega_e;
    return MCL_OK;
}

static int foc_read_temp(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet)
{
    (void)ctx;
    *temp_motor = 25.0f;
    *temp_fet = 30.0f;
    return MCL_OK;
}

/* 统一初始化：电机模型 + mock HAL + 门面配置 + 磁链观测器（有感/无感） */
static void foc_setup(mcl *motor, mcl_hal_ops *hal, foc_hal_t *hctx,
                      mcl_observer_flux *obs, mcl_observer_flux_params *op,
                      float omega_init, mcl_mode mode)
{
    mcl_config cfg;

    hctx->motor.R = 1.0f;
    hctx->motor.L = 0.001f;
    hctx->motor.lambda = 0.02f;
    hctx->motor.J = 0.0001f;
    hctx->motor.pole_pairs = 4.0f;
    hctx->motor.i_alpha = 0.0f;
    hctx->motor.i_beta = 0.0f;
    hctx->motor.theta_e = 0.0f;
    hctx->motor.omega_e = omega_init;
    hctx->v_alpha = 0.0f;
    hctx->v_beta = 0.0f;
    hctx->dt = 0.0001f;

    hal->pwm_set_duty = foc_pwm_set_duty;
    hal->adc_read_phase = foc_adc_read_phase;
    hal->adc_read_bus = foc_adc_read_bus;
    hal->enc_read_angle = (mode == MCL_MODE_FOC_SENSORED) ? foc_enc_read_angle : NULL;
    hal->enc_read_speed = (mode == MCL_MODE_FOC_SENSORED) ? foc_enc_read_speed : NULL;
    hal->micros = NULL;
    hal->read_temp = foc_read_temp;

    mcl_config_default(&cfg);
    cfg.current_loop_freq_hz = 10000;
    cfg.max_duty = 1.0f;
    cfg.bus_voltage = 2.0f;
    cfg.current_pid.ki = 200.0f;
    cfg.current_pid.out_min = -2.0f;
    cfg.current_pid.out_max = 2.0f;
    cfg.current_pid.i_min = -2.0f;
    cfg.current_pid.i_max = 2.0f;
    /* 仿真放宽保护 */
    cfg.limits.overcurrent = 100.0f;
    cfg.limits.overvoltage = 100.0f;
    cfg.limits.undervoltage = -100.0f;
    cfg.limits.overtemp = 1000.0f;
    cfg.limits.stall_speed = 0.0f;
    cfg.limits.stall_time = 100.0f;
    /* 场景 6/7 测闭环跟踪（非启动），禁用自动开环；
       自动开环零速启动由 tests/term_sim.c 场景 4 覆盖 */
    cfg.openloop_rpm = 0.0f;

    op->lambda = 0.02f;
    op->resistance = 1.0f;
    op->inductance = 0.001f;
    op->gain = 500.0f;

    mcl_init(motor, &cfg, hal, hctx,
             (mode == MCL_MODE_FOC_SENSORLESS) ? &mcl_observer_flux_ops : NULL,
             obs, op);
    mcl_set_mode(motor, mode);
    mcl_start(motor);
}

static float foc_phase_err(foc_hal_t *hctx, mcl *motor)
{
    float err = hctx->motor.theta_e - motor->phase_rad;
    while (err > MCL_PI) { err -= MCL_TWO_PI; }
    while (err < -MCL_PI) { err += MCL_TWO_PI; }
    return err;
}

static void test_foc_closed_loop(FILE *html)
{
    mcl motor;
    mcl_hal_ops hal;
    foc_hal_t hctx;
    mcl_observer_flux obs;
    mcl_observer_flux_params op;

    float rpm_arr[PLOT_N];
    float rpm_ref_arr[PLOT_N];
    float err_arr[PLOT_N];
    int i;

    foc_setup(&motor, &hal, &hctx, &obs, &op, 100.0f, MCL_MODE_FOC_SENSORLESS);
    mcl_set_speed(&motor, 500.0f);   /* 目标 500 rpm（机械） */

    printf("[6] FOC 无感速度环：目标 500 rpm\n");
    for (i = 0; i < PLOT_N; i++)
    {
        mcl_control_tick(&motor);
        rpm_arr[i] = hctx.motor.omega_e / hctx.motor.pole_pairs * 9.5492966f;
        rpm_ref_arr[i] = 500.0f;
        err_arr[i] = foc_phase_err(&hctx, &motor);
    }
    printf("   稳态转速 = %.1f rpm（期望 500）\n", rpm_arr[PLOT_N - 1]);
    printf("   稳态相位误差 = %.5f rad\n", err_arr[PLOT_N - 1]);

    svg_begin(html, "场景 6：FOC 无感速度环（蓝=转速 rpm，绿=参考 500）");
    svg_curve(html, rpm_arr, PLOT_N, 0.0f, 600.0f, "#58a6ff");
    svg_curve(html, rpm_ref_arr, PLOT_N, 0.0f, 600.0f, "#3fb950");
    svg_end(html);

    svg_begin(html, "场景 6b：速度环相位误差");
    svg_curve(html, err_arr, PLOT_N, -0.5f, 0.5f, "#f85149");
    svg_end(html);
}

static void test_pos_closed_loop(FILE *html)
{
    mcl motor;
    mcl_hal_ops hal;
    foc_hal_t hctx;
    mcl_observer_flux obs;
    mcl_observer_flux_params op;

    enum { POS_N = PLOT_N * 3 };   /* 位置环用更长时间（0.3s）收敛 */
    float pos_arr[POS_N];
    float pos_ref_arr[POS_N];
    int i;

    foc_setup(&motor, &hal, &hctx, &obs, &op, 0.0f, MCL_MODE_FOC_SENSORED);
    mcl_set_position(&motor, 1.0f);   /* 目标机械角 1 rad */

    printf("[7] FOC 有感位置环：目标机械角 1 rad（编码器反馈）\n");
    for (i = 0; i < POS_N; i++)
    {
        mcl_control_tick(&motor);
        pos_arr[i] = hctx.motor.theta_e / hctx.motor.pole_pairs;
        pos_ref_arr[i] = 1.0f;
    }
    printf("   稳态机械角 = %.4f rad（期望 1.0）\n", pos_arr[POS_N - 1]);

    svg_begin(html, "场景 7：FOC 有感位置环（蓝=机械角 rad，绿=参考 1.0）");
    svg_curve(html, pos_arr, POS_N, 0.0f, 1.5f, "#58a6ff");
    svg_curve(html, pos_ref_arr, POS_N, 0.0f, 1.5f, "#3fb950");
    svg_end(html);
}

/* ============================ main ============================ */

int main(void)
{
    FILE *html = fopen("tests/sim_output.html", "w");
    if (html == NULL)
    {
        printf("无法创建 tests/sim_output.html\n");
        return 1;
    }

    fprintf(html, "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
                  "<title>mcl 仿真测试</title></head>"
                  "<body style=\"background:#0d1117;color:#c9d1d9;"
                  "font-family:monospace;\">\n");
    fprintf(html, "<h2>mcl 电机控制库仿真测试（float 精度）</h2>\n");

    printf("========== mcl 仿真测试 ==========\n");
    test_transform(html);
    test_pid(html);
    test_svpwm(html);
    test_pll(html);
    test_observer(html);
    test_foc_closed_loop(html);
    test_pos_closed_loop(html);

    fprintf(html, "</body></html>\n");
    fclose(html);

    printf("=================================\n");
    printf("波形已生成：tests/sim_output.html（用浏览器打开查看）\n");
    return 0;
}
