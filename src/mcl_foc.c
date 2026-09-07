/**
 * @file    mcl_foc.c
 * @brief   mcl 电机控制库：FOC 电流环编排实现
 *
 * 把 d/q 轴电流 PI、解耦前馈串成一次电流环运算，输出 d/q 轴电压。
 * 纯算法、无硬件依赖；坐标变换与调制由门面调度。
 */

#include "mcl_foc.h"

void mcl_foc_init(mcl_foc *self, const mcl_config *cfg)
{
    if (self == NULL || cfg == NULL)
    {
        return;
    }

    mcl_pid_init(&self->pid_d, &cfg->current_pid);
    mcl_pid_init(&self->pid_q, &cfg->current_pid);

    /* 简化：以单一相电感近似 Ld≈Lq（SPMSM）；IPMSM 需分开配置 */
    mcl_mtpa_fw_init(&self->mtpa_fw,
                     cfg->phase_inductance,   /* ld */
                     cfg->phase_inductance,   /* lq */
                     cfg->bemf_const,         /* lambda（V/(rad/s) = Wb） */
                     cfg->rated_current);     /* i_max */

    self->phase_resistance = cfg->phase_resistance;
    self->phase_inductance = cfg->phase_inductance;
}

void mcl_foc_run(mcl_foc *self, mcl_scalar id, mcl_scalar iq,
                 mcl_scalar id_ref, mcl_scalar iq_ref,
                 mcl_scalar speed, mcl_scalar vbus, mcl_scalar dt,
                 mcl_scalar *vd, mcl_scalar *vq)
{
    mcl_scalar vd_pi;
    mcl_scalar vq_pi;
    mcl_scalar vd_ff;
    mcl_scalar vq_ff;

    if (self == NULL || vd == NULL || vq == NULL)
    {
        return;
    }

    (void)vbus;  /* 弱磁在 mcl_mtpa_fw_id_ref 中处理，此处预留 */

    /* d/q 轴电流 PI */
    vd_pi = mcl_pid_run(&self->pid_d, MCL_SUB(id_ref, id), dt);
    vq_pi = mcl_pid_run(&self->pid_q, MCL_SUB(iq_ref, iq), dt);

    /* 解耦前馈（稳态）：
       Vd_ff = -ω·Lq·iq
       Vq_ff = ω·(Ld·id + λ) */
    vd_ff = MCL_NEG(MCL_MUL(MCL_MUL(speed, self->mtpa_fw.lq), iq));
    vq_ff = MCL_MUL(speed, MCL_ADD(MCL_MUL(self->mtpa_fw.ld, id), self->mtpa_fw.lambda));

    *vd = MCL_ADD(vd_pi, vd_ff);
    *vq = MCL_ADD(vq_pi, vq_ff);
}
