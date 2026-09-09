/**
 * @file    protection_test.c
 * @brief   mcl 保护功能单元测试
 *
 * 直接测试 mcl_protection 接口，覆盖全部保护路径：
 *   过流、过压、欠压、过温、堵转、温度降额，以及：
 *   - 使能位关闭分支（enabled 位掩码）
 *   - 边界值（恰好等于阈值 → 不触发）
 *   - 堵转计时与恢复复位
 *   - 故障检测优先级（过流 > 过压 > 欠压 > 过温 > 堵转）
 *
 * 不依赖 HAL / 电机模型，直接注入 ia/ib/ic/vbus/temp/speed/dt。
 *
 * 【精度/量纲约定】本测试只验证「相对阈值触发/复位」逻辑，注入量与阈值
 * 均取 per-unit 小量（落在 Q15/Q31 可表达范围 [-1,1) 内），三种精度共用
 * 同一套数值。物理量 A/V/℃/s 在定点模式下的归一化由库方按 base 约定完成，
 * 测试这里故意规避 >1 的绝对值，以覆盖 Q15 饱和陷阱之外的纯逻辑。
 */

#include "mcl.h"
#include <stdio.h>
#include <math.h>

static int g_fail = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { \
            printf("  [OK]   %s\n", name); \
        } else { \
            printf("  [FAIL] %s\n", name); \
            g_fail++; \
        } \
    } while (0)

#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (tol))

/* 「不降额 / 系数 1.0」：mcl_protection_derate 返回 MCL_FROM_FLOAT(1.0f)。
 * float 下精确 1.0；Q15 下为满幅 32767/32768≈0.99997，Q31 下 ≈0.9999999995。
 * 容差放宽到 Q15 一个 LSB（2^-15≈3.05e-5）。 */
#define NEAR_ONE(x, tol) (fabsf((float)(x) - 1.0f) <= (tol))

int main(void)
{
    mcl_protection prot;
    mcl_protection_limits lim;
    mcl_fault f;
    mcl_scalar derate;

    /* per-unit 阈值（全部 <1，三种精度一致） */
    lim.enabled           = MCL_PROTECT_ALL;
    lim.overcurrent       = MCL_FROM_FLOAT(0.10f);
    lim.overvoltage       = MCL_FROM_FLOAT(0.30f);
    lim.undervoltage      = MCL_FROM_FLOAT(0.08f);
    lim.temp_derate_start = MCL_FROM_FLOAT(0.80f);
    lim.overtemp          = MCL_FROM_FLOAT(0.95f);
    lim.stall_speed       = MCL_FROM_FLOAT(0.05f);
    lim.stall_time        = MCL_FROM_FLOAT(0.25f);

    printf("===== mcl 保护单元测试（%s）=====\n",
#if defined(MCL_USE_Q15)
           "Q15"
#elif defined(MCL_USE_Q31)
           "Q31"
#else
           "float"
#endif
    );

    /* ---------- 正常值 → 不触发 ---------- */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("正常值不触发任何故障", f == MCL_FAULT_NONE);

    /* ---------- 过流：任一相 ∣i∣>阈值 ---------- */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.15f), MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.0f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过流触发（ia 超阈值）", f == MCL_FAULT_OVERCURRENT);

    /* 负电流过流（绝对值） */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.15f), MCL_FROM_FLOAT(0.0f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过流触发（ib 负值绝对值超阈值）", f == MCL_FAULT_OVERCURRENT);

    /* 边界：恰好等于阈值 → 不触发 */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.10f), MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.0f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过流边界（恰好等于阈值不触发）", f == MCL_FAULT_NONE);

    /* ---------- 过压 ---------- */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
        MCL_FROM_FLOAT(0.35f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过压触发", f == MCL_FAULT_OVERVOLTAGE);

    /* ---------- 欠压 ---------- */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
        MCL_FROM_FLOAT(0.05f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("欠压触发", f == MCL_FAULT_UNDERVOLTAGE);

    /* ---------- 过温 ---------- */
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.97f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过温触发", f == MCL_FAULT_OVERTEMP);

    /* ---------- 堵转：低速持续超时 ---------- */
    mcl_protection_init(&prot, &lim);
    f = MCL_FAULT_NONE;
    /* speed=0，dt=0.1，累计 3 步 = 0.3 > 0.25 触发 */
    for (int i = 0; i < 3; i++)
    {
        f = mcl_protection_check(&prot,
            MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
            MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
            MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.1f));
    }
    CHECK("堵转触发（0.3 超 0.25 阈值）", f == MCL_FAULT_STALL);
    CHECK("堵转标志 stalled 置位", prot.stalled == true);

    /* 堵转恢复：速度恢复 → 计时清零 */
    mcl_protection_init(&prot, &lim);
    f = MCL_FAULT_NONE;
    for (int i = 0; i < 2; i++)   /* 0.2 < 0.25 未触发 */
    {
        f = mcl_protection_check(&prot,
            MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
            MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
            MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.1f));
    }
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.02f), MCL_FROM_FLOAT(-0.01f), MCL_FROM_FLOAT(-0.01f),
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.1f));   /* 速度恢复 */
    CHECK("堵转未触发（速度恢复复位计时）", f == MCL_FAULT_NONE);
    CHECK("堵转标志 stalled 复位", prot.stalled == false);

    /* ---------- 温度降额 ---------- */
    mcl_protection_init(&prot, &lim);
    derate = mcl_protection_derate(&prot, MCL_FROM_FLOAT(0.50f));
    CHECK("降额：低温不降额（=1.0）", NEAR_ONE(MCL_TO_FLOAT(derate), 5e-5f));

    /* 中点 (0.80+0.95)/2 = 0.875 → derate = (0.95-0.875)/(0.15) = 0.5 */
    derate = mcl_protection_derate(&prot, MCL_FROM_FLOAT(0.875f));
    CHECK("降额：中点线性降额（=0.5）", NEAR(MCL_TO_FLOAT(derate), 0.5f, 2e-2f));

    derate = mcl_protection_derate(&prot, MCL_FROM_FLOAT(0.95f));
    CHECK("降额：达关断温度（=0.0）", NEAR(MCL_TO_FLOAT(derate), 0.0f, 1e-3f));

    derate = mcl_protection_derate(&prot, MCL_FROM_FLOAT(0.97f));
    CHECK("降额：超关断温度（=0.0）", NEAR(MCL_TO_FLOAT(derate), 0.0f, 1e-3f));

    /* ---------- 使能位关闭 ---------- */
    lim.enabled = MCL_PROTECT_OVERVOLTAGE;
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.90f), MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.0f),   /* 过流但未启用 */
        MCL_FROM_FLOAT(0.24f), MCL_FROM_FLOAT(0.40f),
        MCL_FROM_FLOAT(0.50f), MCL_FROM_FLOAT(0.001f));
    CHECK("过流未启用时不触发过流", f == MCL_FAULT_NONE);

    derate = mcl_protection_derate(&prot, MCL_FROM_FLOAT(0.97f));
    CHECK("过温未启用时降额恒 1.0", NEAR_ONE(MCL_TO_FLOAT(derate), 5e-5f));

    /* ---------- 优先级：过流 > 过压 ---------- */
    lim.enabled = MCL_PROTECT_ALL;
    mcl_protection_init(&prot, &lim);
    f = mcl_protection_check(&prot,
        MCL_FROM_FLOAT(0.90f), MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.0f),   /* 过流 */
        MCL_FROM_FLOAT(0.90f), MCL_FROM_FLOAT(0.97f),                         /* 过压+过温 */
        MCL_FROM_FLOAT(0.0f), MCL_FROM_FLOAT(0.001f));                        /* 堵转 */
    CHECK("优先级：过流优先于过压/过温/堵转", f == MCL_FAULT_OVERCURRENT);

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
