/**
 * @file    mcl_protection.c
 * @brief   mcl 电机控制库：保护检测实现
 */

#include "mcl_protection.h"

void mcl_protection_init(mcl_protection *self, const mcl_protection_limits *limits)
{
    if (self == NULL || limits == NULL)
    {
        return;
    }

    self->limits = *limits;
    self->stall_timer = (mcl_scalar)0;
    self->stalled = false;
}

mcl_fault mcl_protection_check(mcl_protection *self, mcl_scalar ia, mcl_scalar ib, mcl_scalar ic,
                               mcl_scalar vbus, mcl_scalar temp, mcl_scalar speed, mcl_scalar dt)
{
    if (self == NULL)
    {
        return MCL_FAULT_NONE;
    }

    /* 过流 */
    if ((self->limits.enabled & MCL_PROTECT_OVERCURRENT) &&
        (MCL_ABS(ia) > self->limits.overcurrent ||
         MCL_ABS(ib) > self->limits.overcurrent ||
         MCL_ABS(ic) > self->limits.overcurrent))
    {
        return MCL_FAULT_OVERCURRENT;
    }

    /* 过压 / 欠压 */
    if ((self->limits.enabled & MCL_PROTECT_OVERVOLTAGE) &&
        vbus > self->limits.overvoltage)
    {
        return MCL_FAULT_OVERVOLTAGE;
    }
    if ((self->limits.enabled & MCL_PROTECT_UNDERVOLTAGE) &&
        vbus < self->limits.undervoltage)
    {
        return MCL_FAULT_UNDERVOLTAGE;
    }

    /* 过温 */
    if ((self->limits.enabled & MCL_PROTECT_OVERTEMP) &&
        temp > self->limits.overtemp)
    {
        return MCL_FAULT_OVERTEMP;
    }

    /* 堵转：速度低于阈值且持续超时 */
    if (self->limits.enabled & MCL_PROTECT_STALL)
    {
        if (MCL_ABS(speed) < self->limits.stall_speed)
        {
            self->stall_timer = MCL_ADD(self->stall_timer, dt);
            if (self->stall_timer > self->limits.stall_time)
            {
                self->stalled = true;
                return MCL_FAULT_STALL;
            }
        }
        else
        {
            self->stall_timer = (mcl_scalar)0;
            self->stalled = false;
        }
    }

    return MCL_FAULT_NONE;
}

mcl_scalar mcl_protection_derate(mcl_protection *self, mcl_scalar temp)
{
    mcl_scalar range;

    /* 「系数 1.0 = 不降额」必须用 MCL_FROM_FLOAT(1.0f) 表达满幅：
     * 裸整数 (mcl_scalar)1 在 Q15/Q31 下是 1 LSB（≈0.00003 / ≈4.7e-7），
     * 会被误当成「几乎不降额到 0」，导致电流限幅直接失效。 */
    if (self == NULL)
    {
        return MCL_FROM_FLOAT(1.0f);
    }

    /* 过温保护关闭则不降额 */
    if (!(self->limits.enabled & MCL_PROTECT_OVERTEMP))
    {
        return MCL_FROM_FLOAT(1.0f);
    }

    if (temp <= self->limits.temp_derate_start)
    {
        return MCL_FROM_FLOAT(1.0f);
    }
    if (temp >= self->limits.overtemp)
    {
        return (mcl_scalar)0;
    }

    range = MCL_SUB(self->limits.overtemp, self->limits.temp_derate_start);
    if (range <= (mcl_scalar)0)
    {
        return (mcl_scalar)0;
    }

    /* 线性降额：(overtemp - temp) / (overtemp - temp_start)，结果 ∈ [0,1] */
    return MCL_DIV(MCL_SUB(self->limits.overtemp, temp), range);
}
