/**
 * @file    mcl_pll.c
 * @brief   mcl 电机控制库：PLL 锁相环实现（相位跟踪 + 速度估计）
 *
 * 对输入相位做 PI 跟踪，输出平滑相位与速度。
 * 角度约定：float 用弧度（wrap 到 [-π, π]）；定点用归一化（wrap 到 [-0.5, 0.5]）。
 */

#include "mcl_pll.h"
#include "mcl_math.h"

#ifndef MCL_DISABLE_OBSERVER

/* 半圈 / 全圈常量（float：π / 2π；定点：0.5 / 1.0） */
#if defined(MCL_USE_Q15)
    #define MCL_PLL_HALF_TURN ((mcl_scalar)16384)
    #define MCL_PLL_FULL_TURN ((mcl_scalar)32767)
#elif defined(MCL_USE_Q31)
    #define MCL_PLL_HALF_TURN ((mcl_scalar)1073741824)
    #define MCL_PLL_FULL_TURN ((mcl_scalar)2147483647)
#else
    #define MCL_PLL_HALF_TURN ((mcl_scalar)MCL_PI)
    #define MCL_PLL_FULL_TURN ((mcl_scalar)MCL_TWO_PI)
#endif

/** 角度归一到 [-半圈, 半圈) */
static mcl_scalar mcl_pll_wrap(mcl_scalar x)
{
    while (x > MCL_PLL_HALF_TURN)
    {
        x -= MCL_PLL_FULL_TURN;
    }
    while (x < -MCL_PLL_HALF_TURN)
    {
        x += MCL_PLL_FULL_TURN;
    }
    return x;
}

void mcl_pll_init(mcl_pll *self, mcl_scalar kp, mcl_scalar ki)
{
    if (self == NULL)
    {
        return;
    }

    self->kp = kp;
    self->ki = ki;
    mcl_pll_reset(self);
}

void mcl_pll_reset(mcl_pll *self)
{
    if (self == NULL)
    {
        return;
    }

    self->phase = (mcl_scalar)0;
    self->speed = (mcl_scalar)0;
    self->i_term = (mcl_scalar)0;
}

void mcl_pll_run(mcl_pll *self, mcl_scalar phase, mcl_scalar dt,
                 mcl_scalar *phase_out, mcl_scalar *speed_out)
{
    mcl_scalar err;

    if (self == NULL)
    {
        return;
    }

    /* 相位误差（wrap 到半圈内） */
    err = mcl_pll_wrap(MCL_SUB(phase, self->phase));

    /* PI：speed = kp*err + i_term；i_term += ki*err*dt */
    self->i_term = MCL_ADD(self->i_term, MCL_MUL(MCL_MUL(self->ki, err), dt));
    self->speed = MCL_ADD(MCL_MUL(self->kp, err), self->i_term);

    /* 积分相位 */
    self->phase = mcl_pll_wrap(MCL_ADD(self->phase, MCL_MUL(self->speed, dt)));

    if (phase_out != NULL)
    {
        *phase_out = self->phase;
    }
    if (speed_out != NULL)
    {
        *speed_out = self->speed;
    }
}

#endif /* MCL_DISABLE_OBSERVER */
