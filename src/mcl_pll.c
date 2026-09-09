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
    self->last_phase = (mcl_scalar)0;
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

    /* VESC 式 PLL（foc_math.c foc_pll_run）：
     *   phase += (speed + kp·err)·dt
     *   speed += ki·err·dt
     * kp 是「相位锁定比例增益」（单位 1/s，作用于相位积分，加到速度项上），
     * ki 是「速度积分增益」（单位 1/s²）。两者都不直接产出速度，避免了
     * 「speed = kp·err」结构在定点下 kp>1 时直接饱和的问题。
     *
     * 量纲约定：
     *   - float：phase=rad、speed=rad/s、dt=s，kf 直接成立。
     *   - 定点：phase=「圈」(1.0=2π)、speed=「电气速度 pu」(=ω/W_BASE)、dt=dt_pu。
     *     相位积分需 1/(2π) 因子把「rad 增量」转「圈」（同 mcl.c 的 mcl_speed_to_phase_incr）；
     *     kp 单位=(电气速度 pu)/圈、ki 单位=(电气速度 pu)/(圈·dt_pu)，均已含 2π 换算。 */

    /* 相位积分：phase += (speed + kp·err)·dt */
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    {
        mcl_scalar phase_rate = MCL_ADD(MCL_MUL(self->kp, err), self->speed);
        self->phase = mcl_pll_wrap(MCL_ADD(self->phase,
            MCL_MUL(MCL_MUL(phase_rate, dt), MCL_FROM_FLOAT(1.0f / 6.28318530718f))));
    }
#else
    self->phase = mcl_pll_wrap(MCL_ADD(self->phase,
        MCL_MUL(MCL_ADD(MCL_MUL(self->kp, err), self->speed), dt)));
#endif

    /* 速度积分：speed += ki·err·dt */
    self->speed = MCL_ADD(self->speed, MCL_MUL(MCL_MUL(self->ki, err), dt));

    /* PLL wind-up 保护（对齐 VESC mcpwm_foc.c 的 pll_speed 限幅）。
       仅 float 启用：用输入相位差分（限幅 ±π/3）估计瞬时速度 ref=diff/dt，
       把 PLL 积分速度限幅到 3×ref，防止失锁时 speed 无限累积。
       定点下 speed 由 MCL_ADD 自然饱和到 [0,1)，且「圈/dt_pu」与「电气速度 pu」
       差 2π 常数（2π 定点不可表达），故跳过——饱和已足够防无限增长。 */
#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)
    {
        mcl_scalar diff = mcl_pll_wrap(MCL_SUB(phase, self->last_phase));
        mcl_scalar diff_lim = MCL_FROM_FLOAT(1.0f / 6.0f);   /* π/3 */
        mcl_scalar ref_speed;

        if (diff > diff_lim) { diff = diff_lim; }
        if (diff < MCL_NEG(diff_lim)) { diff = MCL_NEG(diff_lim); }
        if (dt > (mcl_scalar)0)
        {
            ref_speed = MCL_DIV(diff, dt);                    /* rad/s，与 speed 同量纲 */
            if (MCL_ABS(ref_speed) > MCL_FROM_FLOAT(1e-3f))
            {
                mcl_scalar lim = MCL_MUL(MCL_ABS(ref_speed), MCL_FROM_FLOAT(3.0f));
                if (self->speed > lim) { self->speed = lim; }
                if (self->speed < MCL_NEG(lim)) { self->speed = MCL_NEG(lim); }
            }
        }
        self->last_phase = phase;
    }
#endif

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
