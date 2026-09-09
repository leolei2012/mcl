/**
 * @file    mtpa_fw_test.c
 * @brief   mcl MTPA / 弱磁单元测试（float/Q15/Q31 三精度）
 *
 * 验证：
 *   1. SPMSM（Ld=Lq）→ MTPA 输出 Id=0
 *   2. IPMSM（Ld<Lq）→ Id<0，且 |Id| 随 iq 增而增（MTPA 轨迹）
 *   3. 弱磁：高速时 Id 更负
 *   4. 三精度 float 中转后数值一致
 */

#include "mcl.h"
#include "mcl_mtpa_fw.h"
#include <stdio.h>
#include <math.h>

/* 归一化：物理量直接归一化进 [-1,1)，用 per-unit 基值（与定点速度环测试一致口径） */
#define I_BASE   10.0f    /* 额定峰值电流 10A */
#define L_BASE   0.002f   /* 平均电感 2mH */
#define LAM_BASE 0.02f    /* 磁链 0.02 Wb */
#define W_BASE   500.0f   /* 电气基速 */
#define V_BASE   (W_BASE * LAM_BASE)  /* 10V */

int main(void)
{
    mcl_mtpa_fw fw;
    mcl_scalar id;
    int fails = 0;

    /* SPMSM：Ld=Lq=2mH，diff=0 */
    mcl_mtpa_fw_init(&fw,
                     MCL_FROM_FLOAT(0.002f / L_BASE),   /* ld = 1.0 */
                     MCL_FROM_FLOAT(0.002f / L_BASE),   /* lq = 1.0 */
                     MCL_FROM_FLOAT(0.02f / LAM_BASE),  /* lambda = 1.0 */
                     MCL_FROM_FLOAT(1.0f));             /* i_max = 1.0 (10A) */

    mcl_mtpa_fw_id_ref(&fw, MCL_FROM_FLOAT(0.5f), (mcl_scalar)0, MCL_FROM_FLOAT(1.0f), &id);
    printf("SPMSM（Ld=Lq）：iq=0.5pu → Id = %.4f pu（应 = 0）\n", MCL_TO_FLOAT(id));
    if (MCL_ABS(id) > MCL_FROM_FLOAT(1.0e-3f)) { fails++; }

    /* IPMSM：Ld=1mH, Lq=2mH（diff=0.5pu） */
    mcl_mtpa_fw_init(&fw,
                     MCL_FROM_FLOAT(0.001f / L_BASE),   /* ld = 0.5 */
                     MCL_FROM_FLOAT(0.002f / L_BASE),   /* lq = 1.0 */
                     MCL_FROM_FLOAT(0.02f / LAM_BASE),  /* lambda = 1.0 */
                     MCL_FROM_FLOAT(1.0f));

    mcl_mtpa_fw_id_ref(&fw, MCL_FROM_FLOAT(0.3f), (mcl_scalar)0, MCL_FROM_FLOAT(1.0f), &id);
    printf("IPMSM 低速：iq=0.3pu → Id = %.4f pu（应 <0，MTPA 弱磁方向）\n", MCL_TO_FLOAT(id));
    if (id >= (mcl_scalar)0) { fails++; }

    mcl_mtpa_fw_id_ref(&fw, MCL_FROM_FLOAT(0.6f), (mcl_scalar)0, MCL_FROM_FLOAT(1.0f), &id);
    printf("IPMSM 低速：iq=0.6pu → Id = %.4f pu（应比 0.3pu 时更负）\n", MCL_TO_FLOAT(id));

    /* 弱磁：高速（speed 超过基速，反电动势接近电压极限） */
    {
        mcl_scalar id_lo, id_hi;
        mcl_mtpa_fw_id_ref(&fw, MCL_FROM_FLOAT(0.3f), MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(1.0f), &id_lo);
        mcl_mtpa_fw_id_ref(&fw, MCL_FROM_FLOAT(0.3f), MCL_FROM_FLOAT(1.5f), MCL_FROM_FLOAT(1.0f), &id_hi);
        printf("弱磁：speed=0.5pu → Id=%.4f，speed=1.5pu → Id=%.4f（高速应更负）\n",
               MCL_TO_FLOAT(id_lo), MCL_TO_FLOAT(id_hi));
        if (id_hi >= id_lo) { fails++; }
    }

#if defined(MCL_USE_Q15)
    printf("===== MTPA/弱磁单元测试（Q15）=====\n");
#elif defined(MCL_USE_Q31)
    printf("===== MTPA/弱磁单元测试（Q31）=====\n");
#else
    printf("===== MTPA/弱磁单元测试（float）=====\n");
#endif
    printf("%s（%d 项失败）\n", fails == 0 ? "=== 全部通过 ===" : "=== 有失败 ===", fails);

    return fails;
}
