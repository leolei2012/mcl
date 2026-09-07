/**
 * @file    mcl_mtpa_fw.h
 * @brief   mcl 电机控制库：MTPA 电流分配与弱磁控制
 */

#ifndef MCL_MTPA_FW_H
#define MCL_MTPA_FW_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MTPA / 弱磁运行时状态
 */
typedef struct
{
    mcl_scalar ld;                   /**< d 轴电感 H */
    mcl_scalar lq;                   /**< q 轴电感 H */
    mcl_scalar lambda;               /**< 永磁磁链 Wb */
    mcl_scalar i_max;                /**< 电流幅值上限 A */
    mcl_scalar fw_id_min;            /**< 弱磁 Id 下限（负，A） */
} mcl_mtpa_fw;

/**
 * @brief 初始化 MTPA / 弱磁
 * @param self   实例
 * @param ld     d 轴电感 H
 * @param lq     q 轴电感 H
 * @param lambda 永磁磁链 Wb
 * @param i_max  电流幅值上限 A
 */
void mcl_mtpa_fw_init(mcl_mtpa_fw *self, mcl_scalar ld, mcl_scalar lq, mcl_scalar lambda, mcl_scalar i_max);

/**
 * @brief 计算 d 轴电流参考（含 MTPA 与弱磁）
 *
 * 输入 Iq 参考与运行点（转速 / 母线电压），输出满足：
 *  - 低速恒转矩区：MTPA 轨迹（凸极机 Id < 0）
 *  - 高速弱磁区：按电压极限追加负向 Id
 *
 * @param self  实例
 * @param iq_ref q 轴电流参考 A
 * @param speed  电角速度 rad/s
 * @param vbus   母线电压 V
 * @param id_ref d 轴电流参考 A（输出）
 */
void mcl_mtpa_fw_id_ref(mcl_mtpa_fw *self, mcl_scalar iq_ref, mcl_scalar speed, mcl_scalar vbus,
                        mcl_scalar *id_ref);

#ifdef __cplusplus
}
#endif

#endif /* MCL_MTPA_FW_H */
