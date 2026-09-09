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
#include "mcl_bldc_comm.h"

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
 * 注入固定电压矢量（沿 α 轴，duty 斜坡上升）把转子吸到 d 轴，
 * 延时稳定后读取编码器角度得到零位偏移。
 *
 * @param hal          HAL 操作集
 * @param ctx          HAL 上下文
 * @param cfg          配置（极对数等）
 * @param align_duty   对齐注入占空比（0~1，开环电压）
 * @param offset       编码器零位偏移 rad（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_encoder_align(const mcl_hal_ops *hal, void *ctx,
                                const mcl_config *cfg, mcl_scalar align_duty,
                                mcl_scalar *offset);

/**
 * @brief 相电阻测量（参考 VESC mcpwm_foc_measure_resistance）
 *
 * 注入直流电压（沿 α 轴锁定 d 轴，电流不产生转矩），斜坡上升 + 延时等
 * 电流稳定（di/dt→0）后多次采样平均，R = V_avg / I_avg。
 *
 * @param hal        HAL 操作集
 * @param ctx        HAL 上下文
 * @param cfg        配置
 * @param duty       注入占空比（0~1，开环电压）
 * @param resistance 相电阻 Ω（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_resistance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *resistance);

/**
 * @brief 相电感测量（参考 VESC 电压脉冲法）
 *
 * 消磁后施加电压脉冲（沿 α 轴），测电流上升率，
 * L = V · Δt / Δi（短脉冲下 R·i 项可忽略）。
 *
 * @param hal         HAL 操作集
 * @param ctx         HAL 上下文
 * @param cfg         配置
 * @param duty        施加占空比（0~1，开环电压）
 * @param inductance  相电感 H（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_inductance(const mcl_hal_ops *hal, void *ctx,
                             const mcl_config *cfg, mcl_scalar duty,
                             mcl_scalar *inductance);

/**
 * @brief 霍尔相序检测（六步 BLDC）
 *
 * 依次施加 6 个换相步的固定电压矢量（每步直流对齐转子），
 * 读取每个换相步对应的霍尔状态，反推出 hall_map（霍尔组合 → 换相步）。
 *
 * @param hal      HAL 操作集
 * @param ctx      HAL 上下文
 * @param cfg      配置
 * @param duty     对齐占空比（0~1）
 * @param hall_map 霍尔映射（输出，8 项，索引 = 霍尔组合 0~7，值 = 换相步 0~5）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_hall_detect(const mcl_hal_ops *hal, void *ctx,
                              const mcl_config *cfg, mcl_scalar duty,
                              uint8_t hall_map[8]);

/**
 * @brief 磁链 λ 测量（开环拖动法）
 *
 * 开环转到已知转速（空载，电流≈0，反电动势 ≈ 电压），测端电压幅值，
 * λ = V_peak / ω。参考 VESC conf_general_measure_flux_linkage（简化版）。
 *
 * @param hal   HAL 操作集
 * @param ctx   HAL 上下文
 * @param cfg   配置
 * @param duty  开环拖动的电压占空比（0~1）
 * @param speed 拖动电气角速度 rad/s
 * @param linkage 磁链 λ Wb（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_cal_flux_linkage(const mcl_hal_ops *hal, void *ctx,
                               const mcl_config *cfg, mcl_scalar duty,
                               mcl_scalar speed, mcl_scalar *linkage);

#ifdef __cplusplus
}
#endif

#endif /* MCL_CALIBRATION_H */
