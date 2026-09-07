/**
 * @file    mcl_bldc_comm.c
 * @brief   mcl 电机控制库：六步方波换相实现
 */

#include "mcl_bldc_comm.h"

/* 常量：1/3 与 BEMF 过零阈值 */
#if defined(MCL_USE_Q15)
    #define MCL_ONE_THIRD           ((mcl_scalar)10923)
    #define MCL_BEMF_ZC_THRESHOLD   ((mcl_scalar)328)
#elif defined(MCL_USE_Q31)
    #define MCL_ONE_THIRD           ((mcl_scalar)715827883)
    #define MCL_BEMF_ZC_THRESHOLD   ((mcl_scalar)21474836)
#else
    #define MCL_ONE_THIRD           ((mcl_scalar)0.3333333f)
    #define MCL_BEMF_ZC_THRESHOLD   ((mcl_scalar)0.01f)
#endif

/** 按换相步输出三相占空比（一相高、一相低、一相悬空） */
static void bldc_apply(uint8_t step, mcl_scalar duty,
                       mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc)
{
    mcl_scalar neg = MCL_NEG(duty);

    *da = (mcl_scalar)0;
    *db = (mcl_scalar)0;
    *dc = (mcl_scalar)0;

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

void mcl_bldc_comm_init(mcl_bldc_comm *self)
{
    if (self == NULL)
    {
        return;
    }

    self->step = 0;
    self->invert = false;

    /* 默认 120° 霍尔映射（H1 H2 H3），实际应由校准（hall_detect）确定 */
    self->hall_map[0] = 0;
    self->hall_map[1] = 0;   /* 001 */
    self->hall_map[2] = 2;   /* 010 */
    self->hall_map[3] = 1;   /* 011 */
    self->hall_map[4] = 4;   /* 100 */
    self->hall_map[5] = 5;   /* 101 */
    self->hall_map[6] = 3;   /* 110 */
    self->hall_map[7] = 0;
}

void mcl_bldc_comm_step_hall(mcl_bldc_comm *self, uint8_t hall, mcl_scalar duty,
                             mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc)
{
    uint8_t step;

    if (self == NULL)
    {
        return;
    }

    step = self->hall_map[hall & 0x07u];
    if (self->invert)
    {
        step = (uint8_t)((step + 3u) % 6u);
    }

    self->step = step;
    bldc_apply(step, duty, da, db, dc);
}

void mcl_bldc_comm_step_bemf(mcl_bldc_comm *self, mcl_scalar bemf_a, mcl_scalar bemf_b,
                             mcl_scalar bemf_c, mcl_scalar duty,
                             mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc)
{
    mcl_scalar neutral;
    mcl_scalar float_bemf;
    uint8_t float_phase;

    if (self == NULL)
    {
        return;
    }

    /* 虚拟中性点 = (a + b + c) / 3 */
    neutral = MCL_MUL(MCL_ADD(MCL_ADD(bemf_a, bemf_b), bemf_c), MCL_ONE_THIRD);

    /* 悬空相：step 0/3 → C；1/4 → B；2/5 → A */
    switch (self->step)
    {
    case 0:
    case 3: float_phase = 2; break;
    case 1:
    case 4: float_phase = 1; break;
    default: float_phase = 0; break;
    }

    switch (float_phase)
    {
    case 0: float_bemf = MCL_SUB(bemf_a, neutral); break;
    case 1: float_bemf = MCL_SUB(bemf_b, neutral); break;
    default: float_bemf = MCL_SUB(bemf_c, neutral); break;
    }

    /* 悬空相 BEMF 过零（阈值）→ 换相（TODO：30° 延迟 + 滤波完整状态机） */
    if (MCL_ABS(float_bemf) < MCL_BEMF_ZC_THRESHOLD)
    {
        self->step = (uint8_t)((self->step + 1u) % 6u);
    }

    bldc_apply(self->step, duty, da, db, dc);
}
