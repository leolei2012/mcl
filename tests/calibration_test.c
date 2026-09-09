/**
 * @file    calibration_test.c
 * @brief   mcl 校准流程仿真验证（float）
 *
 * 用 mock HAL + 电机模型验证校准能测出正确的相电阻 R 和相电感 L。
 * 校准为开环电压注入（沿 α 轴锁 d 轴，转子不动、无反电势），电机方程简化为：
 *   di_α/dt = (v_α - R·i_α)/L
 *
 * mock HAL：micros() 步进模拟时间（100μs）并推进电机一步，使 cal_delay_us 的
 * 忙等待期间电流随时间演化，模拟真实"延时等稳定/测上升率"。
 */

#include "mcl.h"
#include <stdio.h>
#include <math.h>

#define R_TRUE   1.0f
#define L_TRUE   0.001f
#define VBUS     2.0f

typedef struct
{
    float i_alpha;     /* α 轴电流 A（锁 d 轴，β=0） */
    float v_alpha;     /* 施加 α 轴电压 V */
    uint32_t time_us;  /* 模拟时间 μs */
} hal_t;

#define STEP_US 100u

static void motor_step(hal_t *h)
{
    /* di/dt = (v - R·i)/L，前向欧拉 dt=100μs */
    h->i_alpha += (h->v_alpha - R_TRUE * h->i_alpha) / L_TRUE * (STEP_US * 1.0e-6f);
}

static uint32_t mock_micros(void *ctx)
{
    hal_t *h = (hal_t *)ctx;
    h->time_us += STEP_US;
    motor_step(h);
    return h->time_us;
}

static void mock_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    hal_t *h = (hal_t *)ctx;
    (void)db; (void)dc;
    /* 沿 α 轴相电压 = da · vbus（da ∈ [-1,1]） */
    h->v_alpha = MCL_TO_FLOAT(da) * VBUS;
}

static int mock_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    hal_t *h = (hal_t *)ctx;
    *ia = h->i_alpha;
    *ib = -0.5f * h->i_alpha;
    *ic = -0.5f * h->i_alpha;
    return MCL_OK;
}

static int mock_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;
    *vbus = VBUS;
    *ibus = 0.0f;
    return MCL_OK;
}

int main(void)
{
    mcl_hal_ops hal;
    hal_t hctx;
    mcl_config cfg;
    mcl_scalar R_meas, L_meas;

    hctx.i_alpha = 0.0f;
    hctx.v_alpha = 0.0f;
    hctx.time_us = 0u;

    hal.pwm_set_duty = mock_pwm;
    hal.adc_read_phase = mock_adc;
    hal.adc_read_bus = mock_bus;
    hal.enc_read_angle = NULL;
    hal.enc_read_speed = NULL;
    hal.read_temp = NULL;
    hal.micros = mock_micros;

    mcl_config_default(&cfg);
    cfg.bus_voltage = VBUS;

    printf("===== mcl 校准仿真验证（float）=====\n");

    /* 电阻测量：注入 duty=0.5 → v_α=1V，稳态 i=1A → R=1Ω */
    if (mcl_cal_resistance(&hal, &hctx, &cfg, MCL_FROM_FLOAT(0.5f), &R_meas) == MCL_OK)
    {
        printf("  相电阻 R：测得 %.4f Ω，真实 %.4f Ω\n", MCL_TO_FLOAT(R_meas), R_TRUE);
    }
    else
    {
        printf("  相电阻 R：测量失败\n");
    }

    /* 电感测量：脉冲 duty=0.5 → v_α=1V，di/dt=1000A/s，Δt=100μs → Δi=0.1A → L=1mH */
    if (mcl_cal_inductance(&hal, &hctx, &cfg, MCL_FROM_FLOAT(0.5f), &L_meas) == MCL_OK)
    {
        printf("  相电感 L：测得 %.6f H，真实 %.5f H\n", MCL_TO_FLOAT(L_meas), L_TRUE);
    }
    else
    {
        printf("  相电感 L：测量失败\n");
    }

    return 0;
}
