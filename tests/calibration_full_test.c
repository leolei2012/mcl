/**
 * @file    calibration_full_test.c
 * @brief   mcl 校准子功能测试（float）：电流零漂、编码器对齐、相序、磁链
 *
 * 补齐 calibration_test.c 未覆盖的 4 个校准子项：
 *   1. 电流零漂（mcl_cal_current_offset）
 *   2. 编码器电气零位对齐（mcl_cal_encoder_align）
 *   3. 霍尔相序检测（mcl_cal_hall_detect）
 *   4. 磁链 λ 测量（mcl_cal_flux_linkage）
 *
 * 均为阻塞式流程，mock HAL 用 micros 步进做忙等待延时。
 */

#include "mcl.h"
#include <stdio.h>
#include <math.h>

static int g_fail = 0;
#define CHECK(name, cond) do { if (cond) { printf("  [OK]   %s\n", name); } \
    else { printf("  [FAIL] %s\n", name); g_fail++; } } while (0)

/* ---- mock HAL 上下文 ---- */
typedef struct
{
    float i_offset_a, i_offset_b, i_offset_c;   /* 电流零漂（ADC 返回） */
    float enc_angle;                             /* 编码器角度 */
    uint8_t hall;                                /* 霍尔状态 */
    float phase_v_a, phase_v_b, phase_v_c;       /* 三相端电压 */
    uint32_t time_us;
} hal_t;

static uint32_t mock_micros(void *ctx)
{
    hal_t *h = (hal_t *)ctx;
    h->time_us += 100u;   /* 每次 micros 快进 100us，模拟时间流逝 */
    return h->time_us;
}

static void mock_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    (void)ctx; (void)da; (void)db; (void)dc;
}

static int mock_adc(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    hal_t *h = (hal_t *)ctx;
    *ia = h->i_offset_a;
    *ib = h->i_offset_b;
    *ic = h->i_offset_c;
    return MCL_OK;
}

static int mock_enc(void *ctx, mcl_scalar *angle)
{
    hal_t *h = (hal_t *)ctx;
    *angle = h->enc_angle;
    return MCL_OK;
}

static int mock_hall(void *ctx, uint8_t *hall)
{
    hal_t *h = (hal_t *)ctx;
    *hall = h->hall;
    return MCL_OK;
}

static int mock_phase_voltage(void *ctx, mcl_scalar *va, mcl_scalar *vb, mcl_scalar *vc)
{
    hal_t *h = (hal_t *)ctx;
    *va = h->phase_v_a;
    *vb = h->phase_v_b;
    *vc = h->phase_v_c;
    return MCL_OK;
}

int main(void)
{
    mcl_hal_ops hal;
    hal_t h;
    mcl_config cfg;
    mcl_scalar offset[3];
    mcl_scalar linkage;
    uint8_t hall_map[8];
    mcl_scalar enc_offset;
    int i, ok;

    hal.pwm_set_duty = mock_pwm;
    hal.adc_read_phase = mock_adc;
    hal.adc_read_bus = NULL;
    hal.enc_read_angle = mock_enc;
    hal.enc_read_speed = NULL;
    hal.read_hall = mock_hall;
    hal.adc_read_phase_voltage = mock_phase_voltage;
    hal.micros = mock_micros;
    hal.read_temp = NULL;

    mcl_config_default(&cfg);
    cfg.current_loop_freq_hz = 10000;
    cfg.bus_voltage = 2.0f;

    printf("===== 校准子功能测试（float）=====\n");

    /* ---- 1. 电流零漂 ---- */
    h.i_offset_a = 0.15f; h.i_offset_b = -0.10f; h.i_offset_c = 0.05f;
    ok = mcl_cal_current_offset(&hal, &h, 256u, offset);
    CHECK("电流零漂：返回 OK", ok == MCL_OK);
    CHECK("电流零漂：A 相平均正确", fabsf(MCL_TO_FLOAT(offset[0]) - 0.15f) < 1e-3f);
    CHECK("电流零漂：B 相平均正确", fabsf(MCL_TO_FLOAT(offset[1]) + 0.10f) < 1e-3f);
    CHECK("电流零漂：C 相平均正确", fabsf(MCL_TO_FLOAT(offset[2]) - 0.05f) < 1e-3f);

    /* ---- 2. 编码器电气零位对齐 ---- */
    h.enc_angle = 1.234f;   /* 编码器读到 1.234 rad */
    ok = mcl_cal_encoder_align(&hal, &h, &cfg, MCL_FROM_FLOAT(0.5f), &enc_offset);
    CHECK("编码器对齐：返回 OK", ok == MCL_OK);
    CHECK("编码器对齐：偏移 = 读到的角度", fabsf(MCL_TO_FLOAT(enc_offset) - 1.234f) < 1e-3f);

    /* ---- 3. 霍尔相序检测 ---- */
    h.hall = 0x01u;   /* 固定霍尔状态 001 */
    for (i = 0; i < 8; i++) { hall_map[i] = 99u; }
    ok = mcl_cal_hall_detect(&hal, &h, &cfg, MCL_FROM_FLOAT(0.5f), hall_map);
    CHECK("相序检测：返回 OK", ok == MCL_OK);
    CHECK("相序检测：hall_map[1] 被填入有效步（0~5）", hall_map[0x01] <= 5u);
    CHECK("相序检测：无效组合填 0（hall_map[7]≤5）", hall_map[0x07] <= 5u);

    /* ---- 4. 磁链 λ 测量 ---- */
    /* 空载反电动势 = ω·λ。设 ω=100 rad/s、λ=0.02 → e_peak=2.0V。
       三相端电压应反映这个幅值：设 va=2.0（α 对齐瞬时峰值），vb=vc=-1.0（120° 对称）。
       三相→αβ：u_alpha=va=2.0，u_beta=0.577*(vb+vb+va)=0.577*(1.0)=0.577，mag≈2.082。 */
    h.phase_v_a = 2.0f; h.phase_v_b = -1.0f; h.phase_v_c = -1.0f;
    ok = mcl_cal_flux_linkage(&hal, &h, &cfg, MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(100.0f), &linkage);
    CHECK("磁链测量：返回 OK", ok == MCL_OK);
    /* v_peak 取 α 对齐峰值时 mag≈2.082，λ=mag/ω=0.02082 */
    CHECK("磁链测量：λ ≈ V_peak/ω", fabsf(MCL_TO_FLOAT(linkage) - 0.02082f) < 5e-3f);

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
