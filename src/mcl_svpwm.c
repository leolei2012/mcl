/**
 * @file    mcl_svpwm.c
 * @brief   mcl 电机控制库：空间矢量 PWM 调制实现（min-max 注入三次谐波）
 *
 * 采用 min-max 注入法，等价于七段式 SVPWM，避免扇区判断。
 * 精度跟随 mcl_scalar。
 */

#include "mcl_svpwm.h"

/* 常量：0.5 与 sqrt(3)/2，三种精度 */
#if defined(MCL_USE_Q15)
    #define MCL_HALF     ((mcl_scalar)16384)         /* Q15(0.5) */
    #define MCL_SQRT3_2  ((mcl_scalar)28378)         /* Q15(0.8660254) */
#elif defined(MCL_USE_Q31)
    #define MCL_HALF     ((mcl_scalar)1073741824)    /* Q31(0.5) */
    #define MCL_SQRT3_2  ((mcl_scalar)1859775393)    /* Q31(0.8660254) */
#else
    #define MCL_HALF     ((mcl_scalar)0.5f)
    #define MCL_SQRT3_2  ((mcl_scalar)0.8660254038f)
#endif

static mcl_scalar mcl_svpwm_min3(mcl_scalar a, mcl_scalar b, mcl_scalar c)
{
    mcl_scalar m = a;
    if (b < m) { m = b; }
    if (c < m) { m = c; }
    return m;
}

static mcl_scalar mcl_svpwm_max3(mcl_scalar a, mcl_scalar b, mcl_scalar c)
{
    mcl_scalar m = a;
    if (b > m) { m = b; }
    if (c > m) { m = c; }
    return m;
}

void mcl_svpwm_run(mcl_scalar v_alpha, mcl_scalar v_beta, mcl_scalar max_duty,
                   mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc)
{
    mcl_scalar va;
    mcl_scalar vb;
    mcl_scalar vc;
    mcl_scalar vmin;
    mcl_scalar vmax;
    mcl_scalar v0;

    /* 反 Clark → 三相电压（每相 [-1, 1]） */
    va = v_alpha;
    vb = MCL_ADD(MCL_MUL(MCL_NEG(v_alpha), MCL_HALF), MCL_MUL(v_beta, MCL_SQRT3_2));
    vc = MCL_SUB(MCL_MUL(MCL_NEG(v_alpha), MCL_HALF), MCL_MUL(v_beta, MCL_SQRT3_2));

    /* 零序分量 v0 = -(vmin + vmax)/2（先各自乘 1/2 防中间量溢出） */
    vmin = mcl_svpwm_min3(va, vb, vc);
    vmax = mcl_svpwm_max3(va, vb, vc);
    v0 = MCL_NEG(MCL_ADD(MCL_MUL(vmin, MCL_HALF), MCL_MUL(vmax, MCL_HALF)));

    /* 注入零序 → 映射到 [0,1] → 乘调制比上限 */
    *da = MCL_MUL(MCL_ADD(MCL_MUL(MCL_ADD(va, v0), MCL_HALF), MCL_HALF), max_duty);
    *db = MCL_MUL(MCL_ADD(MCL_MUL(MCL_ADD(vb, v0), MCL_HALF), MCL_HALF), max_duty);
    *dc = MCL_MUL(MCL_ADD(MCL_MUL(MCL_ADD(vc, v0), MCL_HALF), MCL_HALF), max_duty);
}
