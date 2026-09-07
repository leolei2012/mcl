/**
 * @file    mcl_mtpa_fw.c
 * @brief   mcl 电机控制库：MTPA 电流分配与弱磁控制实现
 *
 * 注意：当前为 float 语义实现。MTPA 公式中的常数（8、4、√3）与除法，
 * 在定点（Q15/Q31）下需额外缩放与定点除法（TODO），切定点时需重新处理。
 */

#include "mcl_mtpa_fw.h"
#include "mcl_math.h"

void mcl_mtpa_fw_init(mcl_mtpa_fw *self, mcl_scalar ld, mcl_scalar lq,
                      mcl_scalar lambda, mcl_scalar i_max)
{
    if (self == NULL)
    {
        return;
    }

    self->ld = ld;
    self->lq = lq;
    self->lambda = lambda;
    self->i_max = i_max;
    self->fw_id_min = MCL_NEG(i_max);
}

void mcl_mtpa_fw_id_ref(mcl_mtpa_fw *self, mcl_scalar iq_ref, mcl_scalar speed,
                        mcl_scalar vbus, mcl_scalar *id_ref)
{
    mcl_scalar id;
    mcl_scalar lq_ld;

    if (self == NULL || id_ref == NULL)
    {
        return;
    }

    id = (mcl_scalar)0;
    lq_ld = MCL_SUB(self->lq, self->ld);   /* Lq - Ld */

    /* MTPA：SPMSM（Lq==Ld）→ Id=0；IPMSM（Lq>Ld）→ 公式 */
    if (lq_ld > (mcl_scalar)0)
    {
        /* Id = (λ - sqrt(λ² + 8·(Lq-Ld)²·iq²)) / (4·(Lq-Ld))
           TODO 定点：常数 8/4 需缩放，除法需定点实现 */
        mcl_scalar t1 = MCL_MUL(self->lambda, self->lambda);
        mcl_scalar t2 = MCL_MUL(lq_ld, lq_ld);
        mcl_scalar t3 = MCL_MUL(iq_ref, iq_ref);
        mcl_scalar disc = mcl_math_sqrt(MCL_ADD(t1, MCL_MUL((mcl_scalar)8, MCL_MUL(t2, t3))));
        id = MCL_SUB(self->lambda, disc) / MCL_MUL((mcl_scalar)4, lq_ld);
    }

    /* 弱磁（简化）：超基速时按电压极限反解 Id
       Id_fw = (V_limit/|ω| - λ) / Ld，取更负者
       TODO 定点：除法需定点实现 */
    if (speed > (mcl_scalar)0)
    {
        mcl_scalar v_limit = vbus / (mcl_scalar)1.7320508f;  /* vbus / √3 */
        mcl_scalar id_fw = MCL_SUB(v_limit / speed, self->lambda) / self->ld;
        if (id_fw < id)
        {
            id = id_fw;
        }
    }

    /* 限幅到弱磁下限（防过度弱磁） */
    if (id < self->fw_id_min)
    {
        id = self->fw_id_min;
    }

    *id_ref = id;
}
