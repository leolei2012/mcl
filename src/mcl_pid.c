/**
 * @file    mcl_pid.c
 * @brief   mcl 电机控制库：通用 PID 控制器实现
 *
 * 位置式 PID，限幅 + 抗积分饱和。核心运算走 MCL_MUL/MCL_ADD/MCL_SUB，
 * 精度跟随 mcl_scalar（float / Q15 / Q31）。
 *
 * 系数约定：
 *  - kp：比例系数（连续域）
 *  - ki：积分系数（连续域，内部乘 dt 离散化）
 *  - kd：微分系数（离散域，已含 1/dt，误差差分）
 */

#include "mcl_pid.h"

void mcl_pid_init(mcl_pid *self, const mcl_pid_params *params)
{
    if (self == NULL || params == NULL)
    {
        return;
    }

    self->params = *params;
    mcl_pid_reset(self);
}

void mcl_pid_set_params(mcl_pid *self, const mcl_pid_params *params)
{
    if (self == NULL || params == NULL)
    {
        return;
    }

    self->params = *params;
}

void mcl_pid_reset(mcl_pid *self)
{
    if (self == NULL)
    {
        return;
    }

    self->i_term = (mcl_scalar)0;
    self->prev_error = (mcl_scalar)0;
    self->prev_out = (mcl_scalar)0;
}

mcl_scalar mcl_pid_run(mcl_pid *self, mcl_scalar error, mcl_scalar dt)
{
    mcl_scalar p_term;
    mcl_scalar d_term;
    mcl_scalar out;

    if (self == NULL)
    {
        return (mcl_scalar)0;
    }

    /* 比例项 */
    p_term = MCL_MUL(self->params.kp, error);

    /* 积分项：i_term += ki * error * dt，累加后按 i_min/i_max 限幅（抗积分饱和） */
    self->i_term = MCL_ADD(self->i_term,
                           MCL_MUL(MCL_MUL(self->params.ki, error), dt));
    if (self->i_term > self->params.i_max)
    {
        self->i_term = self->params.i_max;
    }
    if (self->i_term < self->params.i_min)
    {
        self->i_term = self->params.i_min;
    }

    /* 微分项：kd * (error - prev_error) */
    d_term = MCL_MUL(self->params.kd, MCL_SUB(error, self->prev_error));
    self->prev_error = error;

    /* 汇总 + 输出限幅 */
    out = MCL_ADD(p_term, MCL_ADD(self->i_term, d_term));
    if (out > self->params.out_max)
    {
        out = self->params.out_max;
    }
    if (out < self->params.out_min)
    {
        out = self->params.out_min;
    }

    self->prev_out = out;
    return out;
}
