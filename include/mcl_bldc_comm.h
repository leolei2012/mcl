/**
 * @file    mcl_bldc_comm.h
 * @brief   mcl 电机控制库：六步方波换相编排
 *
 * 支持霍尔换相与无感 BEMF 过零换相，输出梯形波调制的三相占空比。
 */

#ifndef MCL_BLDC_COMM_H
#define MCL_BLDC_COMM_H

#include <stdint.h>

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 六步换相状态
 */
typedef struct
{
    uint8_t step;               /**< 当前换相步 0~5 */
    uint8_t hall_map[8];        /**< 霍尔组合(0~7) → 换相步 映射 */
    bool    invert;             /**< 相序反转 */
} mcl_bldc_comm;

/**
 * @brief 初始化六步换相
 * @param self 实例
 */
void mcl_bldc_comm_init(mcl_bldc_comm *self);

/**
 * @brief 按霍尔状态换相（120°/60° 由 hall_map 体现）
 * @param self 实例
 * @param hall 霍尔状态（bit0/1/2 对应 H1/H2/H3）
 * @param duty 占空比 0~1
 * @param da   A 相占空比（输出）
 * @param db   B 相占空比（输出）
 * @param dc   C 相占空比（输出）
 */
void mcl_bldc_comm_step_hall(mcl_bldc_comm *self, uint8_t hall, mcl_scalar duty,
                             mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc);

/**
 * @brief 按 BEMF 过零换相（无感）
 * @param self 实例
 * @param bemf_a A 相反电动势（标幺或电压）
 * @param bemf_b B 相反电动势
 * @param bemf_c C 相反电动势
 * @param duty 占空比 0~1
 * @param da   A 相占空比（输出）
 * @param db   B 相占空比（输出）
 * @param dc   C 相占空比（输出）
 */
void mcl_bldc_comm_step_bemf(mcl_bldc_comm *self, mcl_scalar bemf_a, mcl_scalar bemf_b, mcl_scalar bemf_c,
                             mcl_scalar duty, mcl_scalar *da, mcl_scalar *db, mcl_scalar *dc);

#ifdef __cplusplus
}
#endif

#endif /* MCL_BLDC_COMM_H */
