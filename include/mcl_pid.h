/**
 * @file    mcl_pid.h
 * @brief   mcl 电机控制库：通用 PID 控制器（限幅 + 抗积分饱和）
 */

#ifndef MCL_PID_H
#define MCL_PID_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器运行时状态
 */
typedef struct
{
    mcl_pid_params params;      /**< 参数 */
    mcl_scalar i_term;          /**< 积分项 */
    mcl_scalar prev_error;      /**< 上次误差（微分用） */
    mcl_scalar prev_out;        /**< 上次输出 */
} mcl_pid;

/**
 * @brief 初始化 PID 控制器
 * @param self   控制器实例
 * @param params 参数（含限幅与积分限幅）
 */
void mcl_pid_init(mcl_pid *self, const mcl_pid_params *params);

/**
 * @brief 运行时更新参数（不清积分）
 * @param self   控制器实例
 * @param params 新参数
 */
void mcl_pid_set_params(mcl_pid *self, const mcl_pid_params *params);

/**
 * @brief 复位控制器（清积分与历史）
 * @param self 控制器实例
 */
void mcl_pid_reset(mcl_pid *self);

/**
 * @brief 执行一次 PID 运算
 * @param self  控制器实例
 * @param error 误差（目标 - 反馈）
 * @param dt    控制周期 s
 * @return 控制输出（已限幅）
 */
mcl_scalar mcl_pid_run(mcl_pid *self, mcl_scalar error, mcl_scalar dt);

#ifdef __cplusplus
}
#endif

#endif /* MCL_PID_H */
