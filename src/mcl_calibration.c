/**
 * @file    mcl_calibration.c
 * @brief   mcl 电机控制库：校准实现（阻塞式，宿主在电机停转时调用）
 *
 * 注意：当前为 float 语义实现；平均 / 除法等在定点下需定点除法（TODO）。
 */

#include "mcl_calibration.h"

int mcl_cal_current_offset(const mcl_hal_ops *hal, void *ctx,
                                 uint32_t samples, mcl_scalar offset[3])
{
    uint32_t i;
    mcl_scalar sum_a = (mcl_scalar)0;
    mcl_scalar sum_b = (mcl_scalar)0;
    mcl_scalar sum_c = (mcl_scalar)0;

    if (hal == NULL || hal->adc_read_phase == NULL || offset == NULL)
    {
        return MCL_ERR_PARAM;
    }

    for (i = 0; i < samples; i++)
    {
        mcl_scalar ia;
        mcl_scalar ib;
        mcl_scalar ic;
        if (hal->adc_read_phase(ctx, &ia, &ib, &ic) != MCL_OK)
        {
            return MCL_ERR_HAL;
        }
        sum_a = MCL_ADD(sum_a, ia);
        sum_b = MCL_ADD(sum_b, ib);
        sum_c = MCL_ADD(sum_c, ic);
    }

    /* 平均（TODO 定点：除法与样本数缩放） */
    offset[0] = sum_a / (mcl_scalar)samples;
    offset[1] = sum_b / (mcl_scalar)samples;
    offset[2] = sum_c / (mcl_scalar)samples;
    return MCL_OK;
}

int mcl_cal_encoder_align(const mcl_hal_ops *hal, void *ctx,
                                const mcl_config *cfg, mcl_scalar align_current,
                                mcl_scalar *offset)
{
    mcl_scalar angle;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->enc_read_angle == NULL || offset == NULL)
    {
        return MCL_ERR_PARAM;
    }
    (void)cfg;

    /* 施加直流把转子吸到 d 轴（简化：A 正、BC 负） */
    hal->pwm_set_duty(ctx, align_current, MCL_NEG(align_current), MCL_NEG(align_current));

    /* TODO：延时等待转子对齐（阻塞延时 / 宿主轮询） */

    if (hal->enc_read_angle(ctx, &angle) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }

    *offset = angle;

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    return MCL_OK;
}

int mcl_cal_resistance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar current,
                             mcl_scalar *resistance)
{
    mcl_scalar ia;
    mcl_scalar ib;
    mcl_scalar ic;
    mcl_scalar vbus = (mcl_scalar)0;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->adc_read_phase == NULL || resistance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    (void)cfg;

    /* 注入直流占空比 */
    hal->pwm_set_duty(ctx, current, (mcl_scalar)0, (mcl_scalar)0);

    /* TODO：延时等待电流稳定 */

    if (hal->adc_read_bus != NULL)
    {
        (void)hal->adc_read_bus(ctx, &vbus, &ic);
    }
    if (hal->adc_read_phase(ctx, &ia, &ib, &ic) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }

    /* R = duty * vbus / i（简化；TODO 定点除法 + 完整测量流程） */
    *resistance = MCL_MUL(current, vbus) / ia;

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    return MCL_OK;
}

int mcl_cal_inductance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *inductance)
{
    mcl_scalar ia;
    mcl_scalar ib;
    mcl_scalar ic;
    mcl_scalar vbus = (mcl_scalar)0;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->adc_read_phase == NULL || inductance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    (void)cfg;

    /* 施加 PWM 电压 */
    hal->pwm_set_duty(ctx, duty, (mcl_scalar)0, (mcl_scalar)0);

    /* TODO：延时 + 测量电流上升率 di/dt */

    if (hal->adc_read_bus != NULL)
    {
        (void)hal->adc_read_bus(ctx, &vbus, &ic);
    }
    if (hal->adc_read_phase(ctx, &ia, &ib, &ic) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }

    /* L = V * dt / di（简化；TODO 完整测量流程 + 定点除法） */
    *inductance = MCL_MUL(duty, vbus) / ia;

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    return MCL_OK;
}
