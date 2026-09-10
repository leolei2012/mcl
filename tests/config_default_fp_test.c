/**
 * @file    config_default_fp_test.c
 * @brief   mcl_config_default() 定点精度回归测试 + mcl_init 错误码
 *
 * 回归背景（曾出现过的静默失效链）：
 *   1. mcl_config_default() 里保护阈值等物理量（30V/8V/10A/100℃）在 Q15/Q31 下
 *      MCL_FROM_FLOAT() 会饱和成 ±满量程（0x7FFF…/0x7FFFFFFF），导致
 *      overvoltage == undervoltage，mcl_config_validate() 判失败；
 *   2. mcl_init() 曾是 void 返回，校验失败时静默 return 且不注入 hal，
 *      mcl_start 仍置 RUN、mcl_control_tick 每拍因 hal==NULL 静默 return，
 *      电机不转且无任何报错。
 *
 * 本测试锁定：
 *   a. 三精度下 mcl_config_default() 的默认值都能通过 mcl_config_validate()；
 *   b. mcl_init() 对非法配置返回 MCL_ERR_PARAM；
 *   c. mcl_init() 成功后返回 MCL_OK 且 hal 已注入（state 可达 RUN）。
 *
 * 三种精度各编译一版（见 build.ps1），主流程对精度无关。
 */

#include "mcl.h"
#include <stdio.h>

static int g_fail = 0;
#define CHECK(name, cond) do { if (cond) { printf("  [OK]   %s\n", name); } \
    else { printf("  [FAIL] %s\n", name); g_fail++; } } while (0)

typedef struct { int pwm_calls; } fake_hal_ctx_t;

static void fake_pwm(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    (void)ctx; (void)da; (void)db; (void)dc;
    ((fake_hal_ctx_t *)ctx)->pwm_calls++;
}

int main(void)
{
    mcl motor;
    mcl_hal_ops hal;
    fake_hal_ctx_t hctx;
    mcl_config cfg;
    int rc;

    /* 所有回调置 NULL 即可：本测试不跑 control_tick 控制环 */
    hal.pwm_set_duty = fake_pwm;
    hal.adc_read_phase = NULL;
    hal.adc_read_bus = NULL;
    hal.enc_read_angle = NULL;
    hal.enc_read_speed = NULL;
    hal.read_hall = NULL;
    hal.adc_read_phase_voltage = NULL;
    hal.micros = NULL;
    hal.read_temp = NULL;
    hctx.pwm_calls = 0;

#if defined(MCL_USE_Q15)
    printf("===== mcl_config_default 定点回归（Q15）=====\n");
#elif defined(MCL_USE_Q31)
    printf("===== mcl_config_default 定点回归（Q31）=====\n");
#else
    printf("===== mcl_config_default 定点回归（float）=====\n");
#endif

    /* ---- a. 默认配置三精度下都能通过校验 ---- */
    mcl_config_default(&cfg);
    CHECK("默认配置 validate 通过", mcl_config_validate(&cfg) == MCL_OK);

    /* ---- b. 非法配置：mcl_init 返回 MCL_ERR_PARAM 且不回吞 ---- */
    mcl_config_default(&cfg);
    /* 制造 overvoltage <= undervoltage 的非法关系（对任何精度都非法） */
    cfg.limits.overvoltage = MCL_FROM_FLOAT(0.5f);
    cfg.limits.undervoltage = MCL_FROM_FLOAT(0.5f);
    rc = mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    CHECK("非法配置 mcl_init 返回 MCL_ERR_PARAM", rc == MCL_ERR_PARAM);

    /* ---- c. 合法配置：mcl_init 成功且 hal 注入，可 START 到 RUN ---- */
    mcl_config_default(&cfg);
    rc = mcl_init(&motor, &cfg, &hal, &hctx, NULL, NULL, NULL);
    CHECK("合法配置 mcl_init 返回 MCL_OK", rc == MCL_OK);
    CHECK("init 成功后 hal 已注入（start 可达 RUN）", mcl_start(&motor) == MCL_OK);

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
