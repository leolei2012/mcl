/**
 * @file    mcl_calibration.h
 * @brief   mcl 电机控制库：校准（电流零漂、编码器对齐、相电阻/电感测量）
 *
 * 校准为阻塞式流程，由宿主在电机停转 / 安全状态下调用。
 * 依赖通过 HAL 注入，模块本身不持全局状态。
 */

#ifndef MCL_CALIBRATION_H
#define MCL_CALIBRATION_H

#include "mcl_types.h"
#include "mcl_config.h"
#include "mcl_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电流传感器零漂校准
 *
 * 在无 PWM 输出（或全零矢量）下采样三相电流，取平均作为零漂。
 *
 * @param hal     HAL 操作集
 * @param ctx     HAL 上下文
 * @param samples 采样次数
 * @param offset  三相零漂 A（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_current_offset(const mcl_hal_ops *hal, void *ctx,
                                 uint32_t samples, mcl_scalar offset[3]);

/**
 * @brief 编码器电气零位对齐
 *
 * 施加直流电流把转子吸到 d 轴，读取编码器角度得到零位偏移。
 *
 * @param hal          HAL 操作集
 * @param ctx          HAL 上下文
 * @param cfg          配置（极对数等）
 * @param align_current 对齐电流 A
 * @param offset       编码器零位偏移 rad（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_encoder_align(const mcl_hal_ops *hal, void *ctx,
                                const mcl_config *cfg, mcl_scalar align_current,
                                mcl_scalar *offset);

/**
 * @brief 相电阻测量
 * @param hal        HAL 操作集
 * @param ctx        HAL 上下文
 * @param cfg        配置
 * @param current    注入电流 A
 * @param resistance 相电阻 Ω（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_resistance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar current,
                             mcl_scalar *resistance);

/**
 * @brief 相电感测量
 * @param hal         HAL 操作集
 * @param ctx         HAL 上下文
 * @param cfg         配置
 * @param duty        施加占空比 0~1
 * @param inductance  相电感 H（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_inductance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *inductance);

#ifdef __cplusplus
}
#endif

#endif /* MCL_CALIBRATION_H */
