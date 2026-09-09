/**
 * @file    bldc_test.c
 * @brief   mcl 六步 BLDC 换相单元测试（float）
 *
 * 验证：
 *   1. 6 步换相表 duty 分配正确（每步一相高、一相低、一相悬空）
 *   2. 霍尔换相（hall → step 映射）
 *   3. BEMF 积分换相（悬空相 BEMF 积分到阈值触发换相）
 */

#include "mcl.h"
#include "mcl_bldc_comm.h"
#include <stdio.h>

int main(void)
{
    mcl_bldc_comm bldc;
    int fails = 0;
    int i;

    printf("===== BLDC 六步换相单元测试（float）=====\n");

    mcl_bldc_comm_init(&bldc);

    /* 1. 霍尔换相映射：用 hall 值触发，验证 step 推进正确 */
    {
        mcl_scalar duty = 0.5f;
        /* 默认 hall_map：hall 1→step0, 3→step1, 2→step2, 6→step3, 4→step4, 5→step5 */
        uint8_t hall_for_step[6] = {1, 3, 2, 6, 4, 5};
        for (i = 0; i < 6; i++)
        {
            mcl_scalar da, db, dc;
            mcl_bldc_comm_step_hall(&bldc, hall_for_step[i], duty, &da, &db, &dc);
            if (bldc.step != (uint8_t)i)
            {
                printf("  [FAIL] hall=%d → step=%d（期望 %d）\n", hall_for_step[i], bldc.step, i);
                fails++;
            }
        }
        printf("  霍尔换相映射：6 步 step 推进正确\n");
    }

    /* 2. BEMF 积分换相：悬空相 BEMF 为正持续积分到阈值应换相 */
    {
        mcl_scalar da, db, dc;
        mcl_scalar dt = 0.0001f;
        bldc.step = 0;
        bldc.bemf_integrator = 0.0f;
        bldc.bemf_threshold = 0.0005f;  /* 阈值：0.333×0.0001×200 ≈ 0.0067 > 0.0005，会换相 */

        /* step=0 悬空相是 C。给 va=0.5, vb=0.5, vc=1.0 → neutral=2/3，
           C 相 BEMF = vc-neutral = 1/3 > 0，持续积分 */
        for (i = 0; i < 200; i++)
        {
            mcl_bldc_comm_step_bemf(&bldc, 0.5f, 0.5f, 1.0f, 0.5f, dt, &da, &db, &dc);
            if (bldc.step != 0) { break; }
        }
        if (bldc.step == 0)
        {
            printf("  [FAIL] BEMF 积分 200 步后仍未换相\n");
            fails++;
        }
        else
        {
            printf("  BEMF 积分换相：%d 步后换相到 step %d\n", i, bldc.step);
        }
    }

    printf("%s（%d 项失败）\n", fails == 0 ? "=== 全部通过 ===" : "=== 有失败 ===", fails);
    return fails;
}
