/**
 * @file    mcl_transform.c
 * @brief   mcl 电机控制库：Clark / Park 坐标变换实现
 */

#include "mcl_transform.h"
#include "mcl_math.h"

/* 1/sqrt(3)，三种精度下的常量值 */
#if defined(MCL_USE_Q15)
    #define MCL_ONE_BY_SQRT3  ((mcl_scalar)18919)       /* Q15(0.5773503) */
#elif defined(MCL_USE_Q31)
    #define MCL_ONE_BY_SQRT3  ((mcl_scalar)1240398171)  /* Q31(0.5773503) */
#else
    #define MCL_ONE_BY_SQRT3  ((mcl_scalar)0.5773502692f)
#endif

void mcl_transform_clarke(mcl_scalar ia, mcl_scalar ib, mcl_scalar ic,
                          mcl_scalar *alpha, mcl_scalar *beta)
{
    (void)ic;

    /* 幅值不变 Clarke：i_alpha = ia；i_beta = (ia + 2*ib) / sqrt(3) */
    *alpha = ia;
    *beta = MCL_MUL(MCL_ADD(ia, MCL_ADD(ib, ib)), MCL_ONE_BY_SQRT3);
}

void mcl_transform_park(mcl_scalar alpha, mcl_scalar beta, mcl_scalar phase,
                        mcl_scalar *id, mcl_scalar *iq)
{
    mcl_scalar s;
    mcl_scalar c;
    mcl_math_sincos(phase, &s, &c);

    /* id = alpha*cos + beta*sin */
    /* iq = beta*cos - alpha*sin */
    *id = MCL_ADD(MCL_MUL(alpha, c), MCL_MUL(beta, s));
    *iq = MCL_SUB(MCL_MUL(beta, c), MCL_MUL(alpha, s));
}

void mcl_transform_inv_park(mcl_scalar vd, mcl_scalar vq, mcl_scalar phase,
                            mcl_scalar *alpha, mcl_scalar *beta)
{
    mcl_scalar s;
    mcl_scalar c;
    mcl_math_sincos(phase, &s, &c);

    /* v_alpha = vd*cos - vq*sin */
    /* v_beta  = vd*sin + vq*cos */
    *alpha = MCL_SUB(MCL_MUL(vd, c), MCL_MUL(vq, s));
    *beta = MCL_ADD(MCL_MUL(vd, s), MCL_MUL(vq, c));
}
