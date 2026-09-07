/**
 * @file    mcl_hal.h
 * @brief   mcl 电机控制库：HAL 抽象接口（由宿主实现并注入）
 *
 * mcl 核心不含任何寄存器操作；对硬件的访问统一经本文件定义的函数指针完成。
 * 所有回调均带 void *ctx，支持多实例（编码规范 §8）。
 */

#ifndef MCL_HAL_H
#define MCL_HAL_H

#include <stdint.h>

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HAL 抽象操作集，由宿主在 mcl_init 时注入
 *
 * 回调语义：
 *  - 采样 / 编码器读取类：返回 MCL_OK 或 MCL_ERR_HAL（读取失败）。
 *  - pwm_set_duty：高频路径，无返回值；出错由 fault_assert 或保护逻辑处理。
 *  - 所有指针在失败时保持未修改。
 */
typedef struct
{
    /**
     * @brief 输出三相占空比
     * @param ctx HAL 上下文
     * @param da  A 相占空比 [-1, 1]
     * @param db  B 相占空比 [-1, 1]
     * @param dc  C 相占空比 [-1, 1]
     */
    void (*pwm_set_duty)(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc);

    /**
     * @brief 读取三相电流
     * @param ctx HAL 上下文
     * @param ia  A 相电流 A（输出）
     * @param ib  B 相电流 A（输出）
     * @param ic  C 相电流 A（输出，单/双电阻方案可置 0 由库重构）
     * @return MCL_OK / MCL_ERR_HAL
     */
    int (*adc_read_phase)(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic);

    /**
     * @brief 读取母线电压与电流
     * @param ctx  HAL 上下文
     * @param vbus 母线电压 V（输出）
     * @param ibus 母线电流 A（输出）
     * @return MCL_OK / MCL_ERR_HAL
     */
    int (*adc_read_bus)(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus);

    /**
     * @brief 读取转子电气角（有感）
     * @param ctx       HAL 上下文
     * @param angle_rad 电气角 rad（输出，已含极对数折算）
     * @return MCL_OK / MCL_ERR_HAL（无感模式可返回错误）
     */
    int (*enc_read_angle)(void *ctx, mcl_scalar *angle_rad);

    /**
     * @brief 读取转子速度（有感）
     * @param ctx        HAL 上下文
     * @param speed_rad_s 机械角速度 rad/s（输出）
     * @return MCL_OK / MCL_ERR_HAL
     */
    int (*enc_read_speed)(void *ctx, mcl_scalar *speed_rad_s);

    /**
     * @brief 微秒时间基准
     * @param ctx HAL 上下文
     * @return 自启动以来的微秒数
     */
    uint32_t (*micros)(void *ctx);

    /**
     * @brief 读取温度
     * @param ctx        HAL 上下文
     * @param temp_motor 电机温度 ℃（输出）
     * @param temp_fet   功率级 / MOSFET 温度 ℃（输出）
     * @return MCL_OK / MCL_ERR_HAL
     */
    int (*read_temp)(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet);
} mcl_hal_ops;

#ifdef __cplusplus
}
#endif

#endif /* MCL_HAL_H */
