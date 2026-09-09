/**
 * @file    mcl_observer.c
 * @brief   mcl 电机控制库：观测器载体实现（转发到具体算法）
 */

#include "mcl_observer.h"

#ifndef MCL_DISABLE_OBSERVER

void mcl_observer_init(mcl_observer *self, const mcl_observer_ops *ops,
                       void *impl, void *params)
{
    if (self == NULL)
    {
        return;
    }

    /* 无论 ops 是否为空都记录，保证 update/reset/get_confidence 的空指针检查生效 */
    self->ops = ops;
    self->impl = impl;
    self->params = params;

    if (ops != NULL && ops->init != NULL)
    {
        ops->init(impl, params);
    }
}

void mcl_observer_reset(mcl_observer *self)
{
    if (self == NULL || self->ops == NULL || self->ops->reset == NULL)
    {
        return;
    }

    self->ops->reset(self->impl);
}

void mcl_observer_update(mcl_observer *self, mcl_scalar v_alpha, mcl_scalar v_beta,
                         mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                         mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    if (self == NULL || self->ops == NULL || self->ops->update == NULL)
    {
        return;
    }

    self->ops->update(self->impl, v_alpha, v_beta, i_alpha, i_beta, dt,
                      phase_rad, speed_rad_s);
}

mcl_scalar mcl_observer_get_confidence(mcl_observer *self)
{
    if (self == NULL || self->ops == NULL || self->ops->get_confidence == NULL)
    {
        return (mcl_scalar)1;
    }

    return self->ops->get_confidence(self->impl);
}

void mcl_observer_seed(mcl_observer *self, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    if (self == NULL || self->ops == NULL || self->ops->seed == NULL)
    {
        return;
    }

    self->ops->seed(self->impl, flux_alpha, flux_beta);
}

#endif /* MCL_DISABLE_OBSERVER */
