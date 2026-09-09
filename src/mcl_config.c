/**
 * @file    mcl_config.c
 * @brief   mcl 电机控制库：配置实现（默认配置 + 校验）
 */

#include "mcl_config.h"
#include <string.h>

void mcl_config_default(mcl_config *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    /* 电机参数 */
    cfg->pole_pairs = 4;
    cfg->phase_resistance = MCL_FROM_FLOAT(1.0f);
    cfg->phase_inductance = MCL_FROM_FLOAT(0.001f);
    cfg->ld_lq_diff = (mcl_scalar)0;   /* SPMSM：Ld=Lq */
    cfg->bemf_const = MCL_FROM_FLOAT(0.02f);
    cfg->rated_current = MCL_FROM_FLOAT(5.0f);
    cfg->rated_speed_rpm = MCL_FROM_FLOAT(3000.0f);

    /* 运行配置 */
    cfg->pwm_freq_hz = 20000u;
    cfg->current_loop_freq_hz = 20000u;
    cfg->speed_loop_divider = 10u;
    cfg->pos_loop_divider = 10u;
    cfg->max_duty = MCL_FROM_FLOAT(0.95f);
    cfg->bus_voltage = MCL_FROM_FLOAT(24.0f);
    cfg->time_base = MCL_FROM_FLOAT(1.0f);   /* float 默认 1.0 = dt 不归一化 */

    /* 电流环 PID */
    cfg->current_pid.kp = MCL_FROM_FLOAT(1.0f);
    cfg->current_pid.ki = MCL_FROM_FLOAT(100.0f);
    cfg->current_pid.kd = (mcl_scalar)0;
    cfg->current_pid.out_min = MCL_FROM_FLOAT(-1.0f);
    cfg->current_pid.out_max = MCL_FROM_FLOAT(1.0f);
    cfg->current_pid.i_min = MCL_FROM_FLOAT(-1.0f);
    cfg->current_pid.i_max = MCL_FROM_FLOAT(1.0f);

    /* 速度环 PID */
    cfg->speed_pid.kp = MCL_FROM_FLOAT(0.5f);
    cfg->speed_pid.ki = MCL_FROM_FLOAT(20.0f);
    cfg->speed_pid.kd = (mcl_scalar)0;
    cfg->speed_pid.out_min = MCL_FROM_FLOAT(-2.0f);
    cfg->speed_pid.out_max = MCL_FROM_FLOAT(2.0f);
    cfg->speed_pid.i_min = MCL_FROM_FLOAT(-2.0f);
    cfg->speed_pid.i_max = MCL_FROM_FLOAT(2.0f);

    /* 位置环 PID */
    cfg->pos_pid.kp = MCL_FROM_FLOAT(100.0f);
    cfg->pos_pid.ki = MCL_FROM_FLOAT(20.0f);
    cfg->pos_pid.kd = (mcl_scalar)0;
    cfg->pos_pid.out_min = MCL_FROM_FLOAT(-500.0f);
    cfg->pos_pid.out_max = MCL_FROM_FLOAT(500.0f);
    cfg->pos_pid.i_min = MCL_FROM_FLOAT(-500.0f);
    cfg->pos_pid.i_max = MCL_FROM_FLOAT(500.0f);

    /* 反馈 */
    cfg->feedback.type = MCL_FEEDBACK_NONE;
    cfg->feedback.encoder_offset = (mcl_scalar)0;
    cfg->feedback.encoder_cpr = 4096u;

    /* PLL（VESC 式：kp 作用于相位积分、ki 作用于速度积分）
       float 用物理值（VESC 参考 kp=2000 1/s、ki=30000 1/s²）；
       定点用 per-unit 默认（归一化到 <1，见 mcl_fixed_point.md §9.2）。 */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    cfg->pll_kp = MCL_FROM_FLOAT(0.3f);
    cfg->pll_ki = MCL_FROM_FLOAT(0.01f);
#else
    cfg->pll_kp = MCL_FROM_FLOAT(2000.0f);
    cfg->pll_ki = MCL_FROM_FLOAT(30000.0f);
#endif

    /* 无感自动开环启动（VESC 式） */
    cfg->openloop_rpm = MCL_FROM_FLOAT(200.0f);
    cfg->openloop_rpm_low = (mcl_scalar)0;
    cfg->openloop_hyst = MCL_FROM_FLOAT(0.1f);
    cfg->openloop_time_lock = MCL_FROM_FLOAT(0.05f);
    cfg->openloop_time_ramp = MCL_FROM_FLOAT(0.1f);
    cfg->openloop_time = MCL_FROM_FLOAT(0.05f);
    cfg->openloop_boost_q = (mcl_scalar)0;
    cfg->openloop_max_q = MCL_FROM_FLOAT(-1.0f);
    cfg->openloop_drag_q = MCL_FROM_FLOAT(1.0f);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    cfg->openloop_seed_angle = MCL_FROM_FLOAT(0.125f);      /* 45°/360°（VESC M_PI/4） */
#else
    cfg->openloop_seed_angle = MCL_FROM_FLOAT(0.78539816f); /* π/4（45°） */
#endif

    /* 保护阈值 */
    cfg->limits.enabled = MCL_PROTECT_ALL;
    cfg->limits.overcurrent = MCL_FROM_FLOAT(10.0f);
    cfg->limits.overvoltage = MCL_FROM_FLOAT(30.0f);
    cfg->limits.undervoltage = MCL_FROM_FLOAT(8.0f);
    cfg->limits.temp_derate_start = MCL_FROM_FLOAT(80.0f);
    cfg->limits.overtemp = MCL_FROM_FLOAT(100.0f);
    cfg->limits.stall_speed = MCL_FROM_FLOAT(1.0f);
    cfg->limits.stall_time = MCL_FROM_FLOAT(0.5f);
    cfg->fault_stop_time = MCL_FROM_FLOAT(1.0f);

    /* 校准 */
    cfg->current_offset[0] = (mcl_scalar)0;
    cfg->current_offset[1] = (mcl_scalar)0;
    cfg->current_offset[2] = (mcl_scalar)0;
}

int mcl_config_validate(const mcl_config *cfg)
{
    if (cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    if (cfg->pole_pairs == 0u)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->pwm_freq_hz == 0u || cfg->current_loop_freq_hz == 0u)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->speed_loop_divider == 0u || cfg->pos_loop_divider == 0u)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->max_duty <= (mcl_scalar)0 || cfg->max_duty > MCL_FROM_FLOAT(1.0f))
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->phase_inductance <= (mcl_scalar)0)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->rated_current <= (mcl_scalar)0)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg->limits.overvoltage <= cfg->limits.undervoltage)
    {
        return MCL_ERR_PARAM;
    }

    return MCL_OK;
}
