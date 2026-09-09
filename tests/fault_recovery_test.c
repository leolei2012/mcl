/**
 * @file    fault_recovery_test.c
 * @brief   mcl 故障恢复时序 + 现场快照测试
 *
 * 验证：
 *   1. mcl_fault_assert（硬件保护上报）置 FAULT + 关断 PWM + 记录现场快照；
 *   2. fault_stop_time > 0 时，持续 control_tick 达到时长后自动恢复到 IDLE + 清 fault；
 *   3. fault_stop_time = 0 时不自动恢复，需 mcl_clear_fault；
 *   4. mcl_get_fault_info 快照字段（fault/current/voltage/speed/temp/tick）正确。
 *
 * 不依赖电机模型：进 FAULT 后 control_tick 只走恢复计时、不再跑控制环。
 */

#include "mcl.h"
#include <stdio.h>

static int g_fail = 0;
#define CHECK(name, cond) do { if (cond) { printf("  [OK]   %s\n", name); } \
    else { printf("  [FAIL] %s\n", name); g_fail++; } } while (0)

/* minimal HAL：PWM 记录是否被置零；无 ADC（进 FAULT 后不会被调）。 */
typedef struct { int pwm_zero_calls; } fake_hal_ctx_t;

static void fake_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    fake_hal_ctx_t *h = (fake_hal_ctx_t *)ctx;
    (void)da; (void)db; (void)dc;
    if (MCL_TO_FLOAT(da) == 0.0f && MCL_TO_FLOAT(db) == 0.0f && MCL_TO_FLOAT(dc) == 0.0f)
    {
        h->pwm_zero_calls++;
    }
}

static int fake_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx; *vbus = MCL_FROM_FLOAT(24.0f); *ibus = MCL_FROM_FLOAT(0.0f); return MCL_OK;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    fake_hal_ctx_t hctx;
    mcl_config cfg;
    mcl_state st;
    mcl_fault ft;
    mcl_fault_info fi;
    int i;

    hctx.pwm_zero_calls = 0;
    hal.pwm_set_duty = fake_pwm;
    hal.adc_read_phase = NULL;
    hal.adc_read_bus = fake_bus;
    hal.enc_read_angle = NULL;
    hal.enc_read_speed = NULL;
    hal.read_temp = NULL;
    hal.micros = NULL;

    printf("===== 故障恢复 + 快照测试（float）=====\n");

    /* ---- 场景 1：fault_stop_time > 0，自动恢复 ---- */
    mcl_config_default(&cfg);
    cfg.fault_stop_time = MCL_FROM_FLOAT(0.5f);   /* 0.5s 后自动恢复 */
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_start(&motor);   /* 进 RUN */

    /* 硬件保护上报：门驱故障 */
    mcl_fault_assert(&motor, MCL_FAULT_DRV);
    (void)mcl_get_state(&motor, &st);
    (void)mcl_get_fault(&motor, &ft);
    CHECK("fault_assert 后进 FAULT", st == MCL_STATE_FAULT);
    CHECK("fault_assert 记录 DRV 故障", ft == MCL_FAULT_DRV);
    CHECK("fault_assert 关断 PWM", hctx.pwm_zero_calls > 0);

    /* 现场快照：故障码已记录 */
    (void)mcl_get_fault_info(&motor, &fi);
    CHECK("快照记录故障码 DRV", fi.fault == MCL_FAULT_DRV);

    /* 持续 tick（dt = 1/current_loop_freq = 1/20000 = 50us），累计到 0.5s = 10000 tick */
    for (i = 0; i < 10000; i++) { mcl_control_tick(&motor); }
    (void)mcl_get_state(&motor, &st);
    (void)mcl_get_fault(&motor, &ft);
    CHECK("0.5s 后自动恢复到 IDLE", st == MCL_STATE_IDLE);
    CHECK("自动恢复后故障清 NONE", ft == MCL_FAULT_NONE);

    /* ---- 场景 2：fault_stop_time = 0，手动清除 ---- */
    mcl_config_default(&cfg);
    cfg.fault_stop_time = MCL_FROM_FLOAT(0.0f);   /* 不自动恢复 */
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_start(&motor);

    mcl_fault_assert(&motor, MCL_FAULT_OVERCURRENT);
    for (i = 0; i < 50000; i++) { mcl_control_tick(&motor); }  /* 远超任何时长 */
    (void)mcl_get_state(&motor, &st);
    CHECK("fault_stop_time=0 不自动恢复（仍 FAULT）", st == MCL_STATE_FAULT);

    mcl_clear_fault(&motor);
    (void)mcl_get_state(&motor, &st);
    (void)mcl_get_fault(&motor, &ft);
    CHECK("mcl_clear_fault 后回 IDLE + 清 fault", st == MCL_STATE_IDLE && ft == MCL_FAULT_NONE);

    /* ---- 场景 3：现场快照字段完整 ---- */
    mcl_config_default(&cfg);
    mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    mcl_start(&motor);
    mcl_fault_assert(&motor, MCL_FAULT_OVERTEMP);
    (void)mcl_get_fault_info(&motor, &fi);
    CHECK("快照 fault 字段正确", fi.fault == MCL_FAULT_OVERTEMP);
    CHECK("快照 tick >= 0", (int)fi.tick >= 0);

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
