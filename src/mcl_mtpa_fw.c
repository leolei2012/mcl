/**
 * @file    mcl_mtpa_fw.c
 * @brief   mcl 电机控制库：MTPA 电流分配与弱磁控制实现
 *
 * MTPA / 弱磁公式含常数 8、4、√3（>1）与除法，直接定点会溢出；且凸极差 ΔL 的
 * 取值范围使常数无法统一缩放进 [-1,1)。故内部用 float 中转（MCL_TO_FLOAT /
 * MCL_FROM_FLOAT）计算，保证三精度（float/Q15/Q31）数值一致；纯定点优化为后续长尾。
 */

#include "mcl_mtpa_fw.h"
#include "mcl_math.h"
#include <math.h>

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
    float lambda;
    float lq;
    float ld;
    float dl;
    float iq;
    float id;

    if (self == NULL || id_ref == NULL)
    {
        return;
    }

    /* 全部转到 float 域计算（避开 8/4/√3 常数的定点溢出） */
    lambda = (float)MCL_TO_FLOAT(self->lambda);
    lq = (float)MCL_TO_FLOAT(self->lq);
    ld = (float)MCL_TO_FLOAT(self->ld);
    dl = lq - ld;                       /* Lq - Ld ≥ 0 */
    iq = (float)MCL_TO_FLOAT(iq_ref);
    id = 0.0f;

    /* MTPA：SPMSM（dl==0）→ Id=0；IPMSM（dl>0）→ 公式
       Id = (λ - sqrt(λ² + 8·dl²·iq²)) / (4·dl) */
    if (dl > 0.0f)
    {
        float disc = sqrtf(lambda * lambda + 8.0f * dl * dl * iq * iq);
        id = (lambda - disc) / (4.0f * dl);
    }

    /* 弱磁：超基速时按电压极限反解 Id，取更负者
       V_limit = vbus/√3（SVPWM 线性调制区最大相电压幅值）
       Id_fw = (V_limit/|ω| - λ) / Ld */
    {
        float spd = (float)MCL_TO_FLOAT(speed);
        float vb = (float)MCL_TO_FLOAT(vbus);
        if (spd > 0.0f)
        {
            float v_limit = vb / 1.7320508f;
            float id_fw = (v_limit / spd - lambda) / ld;
            if (id_fw < id)
            {
                id = id_fw;
            }
        }
    }

    /* 限幅到弱磁下限（防过度弱磁） */
    {
        float id_min = (float)MCL_TO_FLOAT(self->fw_id_min);
        if (id < id_min)
        {
            id = id_min;
        }
    }

    *id_ref = MCL_FROM_FLOAT(id);
}
