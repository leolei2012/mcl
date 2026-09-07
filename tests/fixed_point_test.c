/**
 * @file    fixed_point_test.c
 * @brief   mcl 定点基础验证（Q15/Q31/float 三种精度共用）
 *
 * 验证定点基础：MCL_DIV / MCL_MUL / 坐标变换 / SVPWM / 数学函数。
 * 所有物理量已归一化到 [-1,1)，角度用归一化表示（定点）或弧度（float）。
 *
 * 编译（工程根目录）：
 *   gcc -std=c99 -Iinclude tests/fixed_point_test.c src -lm -o tests/fp.exe
 *   gcc -std=c99 -Iinclude -DMCL_USE_Q15 tests/fixed_point_test.c src -lm -o tests/fp_q15.exe
 *   gcc -std=c99 -Iinclude -DMCL_USE_Q31 tests/fixed_point_test.c src -lm -o tests/fp_q31.exe
 */

#include "mcl.h"
#include <stdio.h>

static int fails = 0;

static void check(const char *name, float got, float expect, float tol)
{
    float err = got - expect;
    if (err < 0.0f) { err = -err; }
    int ok = (err <= tol);
    if (!ok) { fails++; }
    printf("  [%s] %-24s got=%.6f  expect=%.6f  err=%.2e\n",
           ok ? "OK" : "FAIL", name, got, expect, err);
}

/* 归一化角度：定点 0.25 = 90°；float 弧度 π/2 */
static mcl_scalar angle_90deg(void)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    return MCL_FROM_FLOAT(0.25f);
#else
    return MCL_FROM_FLOAT(1.5707963268f);
#endif
}

int main(void)
{
#if defined(MCL_USE_Q15)
    printf("======== mcl 定点基础验证（Q15）========\n");
#elif defined(MCL_USE_Q31)
    printf("======== mcl 定点基础验证（Q31）========\n");
#else
    printf("======== mcl 定点基础验证（float）========\n");
#endif

    /* 1. MCL_DIV（分母 < 1，结果 < 1） */
    check("DIV 0.25/0.5", MCL_TO_FLOAT(MCL_DIV(MCL_FROM_FLOAT(0.25f), MCL_FROM_FLOAT(0.5f))),
          0.5f, 0.02f);
    check("DIV 0.5/0.75", MCL_TO_FLOAT(MCL_DIV(MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(0.75f))),
          0.6667f, 0.02f);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    /* 除零保护仅定点有意义（float 裸除法返回 inf，定点除零会崩溃需保护） */
    check("DIV 除零保护", MCL_TO_FLOAT(MCL_DIV(MCL_FROM_FLOAT(0.5f), (mcl_scalar)0)),
          1.0f, 0.02f);
#endif

    /* 2. MCL_MUL */
    check("MUL 0.5*0.5", MCL_TO_FLOAT(MCL_MUL(MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(0.5f))),
          0.25f, 0.02f);
    check("MUL -0.5*0.5", MCL_TO_FLOAT(MCL_MUL(MCL_FROM_FLOAT(-0.5f), MCL_FROM_FLOAT(0.5f))),
          -0.25f, 0.02f);

    /* 3. Clarke + Park（三相平衡电流，幅值 0.5 归一化） */
    {
        mcl_scalar ia = MCL_FROM_FLOAT(0.5f);
        mcl_scalar ib = MCL_FROM_FLOAT(-0.25f);
        mcl_scalar ic = MCL_FROM_FLOAT(-0.25f);
        mcl_scalar alpha, beta, id, iq;
        mcl_transform_clarke(ia, ib, ic, &alpha, &beta);
        mcl_transform_park(alpha, beta, (mcl_scalar)0, &id, &iq);
        check("Park id(0)", MCL_TO_FLOAT(id), 0.5f, 0.03f);
        check("Park iq(0)", MCL_TO_FLOAT(iq), 0.0f, 0.03f);
    }

    /* 4. SVPWM（旋转电压矢量 0.5，占空比应落 [0,1]） */
    {
        mcl_scalar da, db, dc;
        mcl_svpwm_run(MCL_FROM_FLOAT(0.5f), (mcl_scalar)0, MCL_FROM_FLOAT(1.0f), &da, &db, &dc);
        float fa = MCL_TO_FLOAT(da), fb = MCL_TO_FLOAT(db), fc = MCL_TO_FLOAT(dc);
        int in_range = (fa >= 0.0f && fa <= 1.0f && fb >= 0.0f && fb <= 1.0f && fc >= 0.0f && fc <= 1.0f);
        check("SVPWM 占空比在 [0,1]", in_range ? 1.0f : 0.0f, 1.0f, 0.0f);
    }

    /* 5. 数学函数（归一化角度） */
    check("sin(90°)", MCL_TO_FLOAT(mcl_math_sin(angle_90deg())), 1.0f, 0.02f);
    check("cos(90°)", MCL_TO_FLOAT(mcl_math_cos(angle_90deg())), 0.0f, 0.02f);

    /* 6. 反 Park（相位 90°，vd=0.5 → v_alpha≈0, v_beta≈0.5） */
    {
        mcl_scalar va, vb;
        mcl_transform_inv_park(MCL_FROM_FLOAT(0.5f), (mcl_scalar)0, angle_90deg(), &va, &vb);
        check("invPark v_alpha(90°)", MCL_TO_FLOAT(va), 0.0f, 0.03f);
        check("invPark v_beta(90°)", MCL_TO_FLOAT(vb), 0.5f, 0.03f);
    }

    printf("\n%s（%d 项失败）\n", fails == 0 ? "=== 全部通过 ===" : "=== 有失败 ===", fails);
    return fails;
}
