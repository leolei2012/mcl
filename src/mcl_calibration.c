/**
 * @file    mcl_calibration.c
 * @brief   mcl 电机控制库：校准实现（阻塞式，宿主在电机停转时调用）
 *
 * 参考 VESC 的测量方法（mcpwm_foc_measure_resistance / measure_inductance）：
 *   - 电流零漂：无 PWM 输出下采样平均
 *   - 编码器对齐：注入固定电压矢量把转子吸到 d 轴（α 轴），延时稳定后读编码器
 *   - 相电阻 R：注入直流电压（锁 d 轴，不产生转矩），斜坡上升 + 延时稳定 + 多次平均，
 *     R = V_avg / I_avg（稳态 di/dt=0，电感压降为 0）
 *   - 相电感 L：电压脉冲测电流上升率，L = V · Δt / Δi（短脉冲，R·i 项可忽略）
 *
 * 校准为开环电压注入（mcl 无校准用电流环），注入量统一为「占空比 duty ∈ [0,1]」。
 * 稳定性依赖 HAL 的 micros 时间基准做忙等待延时。
 */

#include "mcl_calibration.h"
#include "mcl_math.h"

#ifndef MCL_DISABLE_CALIBRATION

/* ============================ 内部延时（基于 micros 忙等待） ============================ */

static void cal_delay_us(const mcl_hal_ops *hal, void *ctx, uint32_t us)
{
    uint32_t start;
    uint32_t now;
    if (hal == NULL || hal->micros == NULL)
    {
        return;
    }
    start = hal->micros(ctx);
    do
    {
        now = hal->micros(ctx);
    } while ((uint32_t)(now - start) < us);
}

/* ============================ 电流零漂 ============================ */

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
    if (samples == 0u)
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

    /* 平均：除以样本数（用 MCL_DIV，定点正确；样本数可能 >1 需保证分母 <1）
       TODO 定点：样本数 >1 时 1/samples 是 <1 的倒数，此处用 MCL_DIV 需样本数在 [-1,1)，
       实际样本数为整数 >1 会溢出。故此处仍用物理语义除法（float）；定点下由宿主在
       offset 语义层处理（样本数归一化）。 */
    offset[0] = sum_a / (mcl_scalar)samples;
    offset[1] = sum_b / (mcl_scalar)samples;
    offset[2] = sum_c / (mcl_scalar)samples;
    return MCL_OK;
}

/* ============================ 编码器对齐 ============================ */

int mcl_cal_encoder_align(const mcl_hal_ops *hal, void *ctx,
                                const mcl_config *cfg, mcl_scalar align_duty,
                                mcl_scalar *offset)
{
    mcl_scalar angle;
    mcl_scalar duty = (mcl_scalar)0;
    int i;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->enc_read_angle == NULL || offset == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 斜坡上升占空比（VESC 式 utils_step_towards），避免电流突变导致转子乱跳/过流 */
    for (i = 0; i < 50; i++)
    {
        duty = MCL_ADD(duty, MCL_DIV(align_duty, MCL_FROM_FLOAT(50.0f)));
        if (duty > align_duty)
        {
            duty = align_duty;
        }
        /* 沿 α 轴注入（A 正、BC 负），把转子吸到电气角 0° */
        hal->pwm_set_duty(ctx, duty, MCL_NEG(duty), MCL_NEG(duty));
        cal_delay_us(hal, ctx, 1000u);
    }

    /* 延时等待转子稳定对齐（电气时间常数量级，取 50ms 足够） */
    cal_delay_us(hal, ctx, 50000u);

    if (hal->enc_read_angle(ctx, &angle) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }

    /* 对齐后编码器读到的角度即零位偏移 */
    *offset = angle;

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    return MCL_OK;
}

/* ============================ 相电阻测量 ============================ */

int mcl_cal_resistance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *resistance)
{
    mcl_scalar sum_i = (mcl_scalar)0;
    mcl_scalar sum_v = (mcl_scalar)0;
    mcl_scalar vbus = (mcl_scalar)0;
    mcl_scalar d = (mcl_scalar)0;
    uint32_t n = 0u;
    int i;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->adc_read_phase == NULL || resistance == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    if (hal->adc_read_bus != NULL)
    {
        mcl_scalar ibus;
        (void)hal->adc_read_bus(ctx, &vbus, &ibus);
    }
    else
    {
        vbus = cfg->bus_voltage;
    }

    /* 斜坡上升占空比，把转子锁到 d 轴（沿 α 轴注入，电流不产生转矩） */
    for (i = 0; i < 50; i++)
    {
        d = MCL_ADD(d, MCL_DIV(duty, MCL_FROM_FLOAT(50.0f)));
        if (d > duty)
        {
            d = duty;
        }
        hal->pwm_set_duty(ctx, d, MCL_NEG(d), MCL_NEG(d));
        cal_delay_us(hal, ctx, 1000u);
    }

    /* 延时等电流稳定（di/dt → 0，只剩电阻压降） */
    cal_delay_us(hal, ctx, 50000u);

    /* 多次采样平均，抑制噪声 */
    for (i = 0; i < 100; i++)
    {
        mcl_scalar ia;
        mcl_scalar ib;
        mcl_scalar ic;
        if (hal->adc_read_phase(ctx, &ia, &ib, &ic) != MCL_OK)
        {
            hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
            return MCL_ERR_HAL;
        }
        /* 稳态 α 轴电流 = ia（锁 d 轴，β 分量≈0），电压 = d · vbus */
        sum_i = MCL_ADD(sum_i, ia);
        sum_v = MCL_ADD(sum_v, MCL_MUL(d, vbus));
        n++;
        cal_delay_us(hal, ctx, 200u);
    }

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);

    if (n == 0u)
    {
        return MCL_ERR_HAL;
    }

    /* R = 平均电压 / 平均电流（TODO 定点：除以样本数 n 需归一化，当前为 float 语义） */
    {
        mcl_scalar i_avg = sum_i / (mcl_scalar)n;
        mcl_scalar v_avg = sum_v / (mcl_scalar)n;
        if (MCL_ABS(i_avg) < MCL_FROM_FLOAT(1.0e-6f))
        {
            return MCL_ERR_HAL;   /* 电流近乎为 0，测量无效 */
        }
        *resistance = MCL_DIV(v_avg, i_avg);
    }
    return MCL_OK;
}

/* ============================ 相电感测量 ============================ */

int mcl_cal_inductance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *inductance)
{
    mcl_scalar ia0;
    mcl_scalar ib0;
    mcl_scalar ic0;
    mcl_scalar ia1;
    mcl_scalar ib1;
    mcl_scalar ic1;
    mcl_scalar vbus = (mcl_scalar)0;
    mcl_scalar v_alpha;
    mcl_scalar di;
    mcl_scalar dt;
    uint32_t t0;
    uint32_t t1;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->adc_read_phase == NULL || hal->micros == NULL ||
        inductance == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    if (hal->adc_read_bus != NULL)
    {
        mcl_scalar ibus;
        (void)hal->adc_read_bus(ctx, &vbus, &ibus);
    }
    else
    {
        vbus = cfg->bus_voltage;
    }

    /* 1. 消磁：PWM 置零，等电流衰减到 0 */
    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    cal_delay_us(hal, ctx, 20000u);

    /* 2. 施加电压脉冲（沿 α 轴），立即采样初始电流 */
    hal->pwm_set_duty(ctx, duty, MCL_NEG(duty), MCL_NEG(duty));
    if (hal->adc_read_phase(ctx, &ia0, &ib0, &ic0) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }
    t0 = hal->micros(ctx);

    /* 3. 短脉冲 Δt（电流上升、未到稳态，R·i 项可忽略），再采样 */
    cal_delay_us(hal, ctx, 100u);
    if (hal->adc_read_phase(ctx, &ia1, &ib1, &ic1) != MCL_OK)
    {
        hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        return MCL_ERR_HAL;
    }
    t1 = hal->micros(ctx);

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);

    /* 4. L = V·Δt/Δi（α 轴分量）
       v_α = duty·vbus（沿 α 轴的相电压），Δi = ia1 - ia0，Δt = (t1-t0) 微秒 → 秒 */
    v_alpha = MCL_MUL(duty, vbus);
    di = MCL_SUB(ia1, ia0);
    dt = MCL_FROM_FLOAT((float)(t1 - t0) * 1.0e-6f);

    if (MCL_ABS(di) < MCL_FROM_FLOAT(1.0e-6f))
    {
        return MCL_ERR_HAL;   /* 电流未上升，测量无效 */
    }

    *inductance = MCL_DIV(MCL_MUL(v_alpha, dt), di);
    return MCL_OK;
}

/* ============================ 霍尔相序检测 ============================ */

/* 6 个换相步的预定义电压矢量（对应 bldc_apply：每相 +duty / -duty / 0） */
static void cal_bldc_step_duty(uint8_t step, mcl_scalar duty,
                               mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc)
{
    mcl_scalar neg = MCL_NEG(duty);
    *da = (mcl_scalar)0; *db = (mcl_scalar)0; *dc = (mcl_scalar)0;
    switch (step)
    {
    case 0: *da = duty; *db = neg; break;   /* A+ B- */
    case 1: *da = duty; *dc = neg; break;   /* A+ C- */
    case 2: *db = duty; *dc = neg; break;   /* B+ C- */
    case 3: *db = duty; *da = neg; break;   /* B+ A- */
    case 4: *dc = duty; *da = neg; break;   /* C+ A- */
    case 5: *dc = duty; *db = neg; break;   /* C+ B- */
    default: break;
    }
}

int mcl_cal_hall_detect(const mcl_hal_ops *hal, void *ctx,
                              const mcl_config *cfg, mcl_scalar duty,
                              uint8_t hall_map[8])
{
    uint8_t step;
    int i;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->read_hall == NULL || hall_map == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 每个换相步：施加直流对齐转子，等稳定后读 hall，记录 hall→step 映射 */
    for (step = 0; step < 6; step++)
    {
        mcl_scalar da, db, dc;
        uint8_t hall = 0u;

        cal_bldc_step_duty(step, duty, &da, &db, &dc);
        hal->pwm_set_duty(ctx, da, db, dc);
        cal_delay_us(hal, ctx, 50000u);   /* 等转子对齐 */

        if (hal->read_hall(ctx, &hall) != MCL_OK)
        {
            hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
            return MCL_ERR_HAL;
        }
        hall_map[hall & 0x07u] = step;
    }

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);

    /* 未覆盖的 hall 组合（111/000 等无效）填 0，避免野值 */
    for (i = 0; i < 8; i++)
    {
        if (hall_map[i] > 5u) { hall_map[i] = 0u; }
    }
    return MCL_OK;
}

/* ============================ 磁链 λ 测量 ============================ */

int mcl_cal_flux_linkage(const mcl_hal_ops *hal, void *ctx,
                               const mcl_config *cfg, mcl_scalar duty,
                               mcl_scalar speed, mcl_scalar *linkage)
{
    mcl_scalar theta = (mcl_scalar)0;
    mcl_scalar dt = (mcl_scalar)0;
    mcl_scalar v_peak = (mcl_scalar)0;
    int i;

    if (hal == NULL || hal->pwm_set_duty == NULL ||
        hal->adc_read_phase_voltage == NULL || hal->micros == NULL ||
        linkage == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (speed <= (mcl_scalar)0)
    {
        return MCL_ERR_PARAM;
    }

    /* 开环旋转电压矢量（V/F）拖动电机到 speed，空载稳态 v ≈ e = ω·λ */
    dt = MCL_FROM_FLOAT(1.0f / (float)cfg->current_loop_freq_hz);
    for (i = 0; i < 200; i++)   /* 拖 200 个周期（或更少，看电流环频率） */
    {
        mcl_scalar s, c;
        mcl_scalar v_alpha, v_beta;
        mcl_scalar da, db, dc;

        theta = MCL_ADD(theta, MCL_MUL(speed, dt));
        /* 角度回绕到 [0,2π)（float 语义，定点需归一化） */
        while (theta > MCL_FROM_FLOAT(6.2831853f)) { theta = MCL_SUB(theta, MCL_FROM_FLOAT(6.2831853f)); }

        mcl_math_sincos(theta, &s, &c);
        v_alpha = MCL_MUL(duty, c);
        v_beta = MCL_MUL(duty, s);

        /* 反 Clark（幅值不变）→ 三相占空比（相对中性点，映射到 [0,1]） */
        da = MCL_ADD(v_alpha, MCL_FROM_FLOAT(0.5f));
        db = MCL_ADD(MCL_NEG(MCL_MUL(v_alpha, MCL_FROM_FLOAT(0.5f))),
                     MCL_MUL(v_beta, MCL_FROM_FLOAT(0.8660254f)));
        db = MCL_ADD(db, MCL_FROM_FLOAT(0.5f));
        dc = MCL_SUB(MCL_NEG(MCL_MUL(v_alpha, MCL_FROM_FLOAT(0.5f))),
                     MCL_MUL(v_beta, MCL_FROM_FLOAT(0.8660254f)));
        dc = MCL_ADD(dc, MCL_FROM_FLOAT(0.5f));

        hal->pwm_set_duty(ctx, da, db, dc);

        /* 测端电压 αβ 幅值（空载 ≈ 反电动势幅值 = ω·λ） */
        {
            mcl_scalar va, vb, vc;
            mcl_scalar u_alpha, u_beta, mag;
            if (hal->adc_read_phase_voltage(ctx, &va, &vb, &vc) != MCL_OK)
            {
                hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
                return MCL_ERR_HAL;
            }
            /* 三相端电压 → αβ（幅值不变） */
            u_alpha = va;
            u_beta = MCL_MUL(MCL_ADD(MCL_ADD(vb, vb), va), MCL_FROM_FLOAT(0.5773503f));
            mag = mcl_math_sqrt(MCL_ADD(MCL_MUL(u_alpha, u_alpha), MCL_MUL(u_beta, u_beta)));
            if (mag > v_peak) { v_peak = mag; }
        }
        cal_delay_us(hal, ctx, 1000u);
    }

    hal->pwm_set_duty(ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);

    if (v_peak <= (mcl_scalar)0)
    {
        return MCL_ERR_HAL;
    }

    /* λ = |e| / ω */
    *linkage = MCL_DIV(v_peak, speed);
    return MCL_OK;
}

#endif /* MCL_DISABLE_CALIBRATION */
