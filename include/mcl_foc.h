/**
 * @file    mcl_foc.h
 * @brief   mcl 电机控制库：FOC 电流环编排
 *
 * 把 PID、MTPA/弱磁、解耦前馈串成一次电流环运算，输出 d/q 轴电压。
 * 纯算法、无硬件依赖；坐标变换与调制由门面调度（见 mcl.h）。
 */

#ifndef MCL_FOC_H
#define MCL_FOC_H

#include "mcl_types.h"
#include "mcl_config.h"
#include "mcl_pid.h"
#include "mcl_mtpa_fw.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FOC 电流环编排状态
 */
typedef struct
{
    mcl_pid pid_d;              /**< d 轴电流 PI */
    mcl_pid pid_q;              /**< q 轴电流 PI */
    mcl_mtpa_fw mtpa_fw;        /**< MTPA / 弱磁 */
    mcl_scalar phase_resistance;     /**< 相电阻（前馈用） */
    mcl_scalar phase_inductance;     /**< 相电感（解耦用） */
} mcl_foc;

/**
 * @brief 初始化 FOC 电流环
 * @param self 实例
 * @param cfg  集中配置（取电流环 PID、电机参数、电流上限）
 */
void mcl_foc_init(mcl_foc *self, const mcl_config *cfg);

/**
 * @brief FOC 电流环单步
 *
 * @param self   实例
 * @param id     d 轴电流反馈 A
 * @param iq     q 轴电流反馈 A
 * @param id_ref d 轴电流参考 A
 * @param iq_ref q 轴电流参考 A
 * @param speed  电角速度 rad/s（解耦前馈用）
 * @param vbus   母线电压 V（弱磁用）
 * @param dt     控制周期 s
 * @param vd     d 轴电压 V（输出）
 * @param vq     q 轴电压 V（输出）
 */
void mcl_foc_run(mcl_foc *self, mcl_scalar id, mcl_scalar iq, mcl_scalar id_ref, mcl_scalar iq_ref,
                 mcl_scalar speed, mcl_scalar vbus, mcl_scalar dt, mcl_scalar *vd, mcl_scalar *vq);

#ifdef __cplusplus
}
#endif

#endif /* MCL_FOC_H */
