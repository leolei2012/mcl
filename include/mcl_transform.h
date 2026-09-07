/**
 * @file    mcl_transform.h
 * @brief   mcl 电机控制库：Clark / Park 坐标变换
 *
 * 幅值不变变换（amplitude-invariant），精度跟随 mcl_scalar。
 * 角度约定：float 模式用弧度；定点模式用归一化角度（1.0 = 2π），见 mcl_math.h。
 */

#ifndef MCL_TRANSFORM_H
#define MCL_TRANSFORM_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Clark 变换：三相静止 → 两相静止（幅值不变）
 * @param ia    A 相电流
 * @param ib    B 相电流
 * @param ic    C 相电流（两电阻采样时可为 0，由库按 ia+ib+ic=0 处理）
 * @param alpha α 轴分量（输出）
 * @param beta  β 轴分量（输出）
 */
void mcl_transform_clarke(mcl_scalar ia, mcl_scalar ib, mcl_scalar ic,
                          mcl_scalar *alpha, mcl_scalar *beta);

/**
 * @brief Park 变换：两相静止 → 同步旋转
 * @param alpha α 轴分量
 * @param beta  β 轴分量
 * @param phase 电气角（float：弧度；定点：归一化）
 * @param id    d 轴分量（输出）
 * @param iq    q 轴分量（输出）
 */
void mcl_transform_park(mcl_scalar alpha, mcl_scalar beta, mcl_scalar phase,
                        mcl_scalar *id, mcl_scalar *iq);

/**
 * @brief 反 Park 变换：同步旋转 → 两相静止
 * @param vd    d 轴电压
 * @param vq    q 轴电压
 * @param phase 电气角（float：弧度；定点：归一化）
 * @param alpha α 轴电压（输出）
 * @param beta  β 轴电压（输出）
 */
void mcl_transform_inv_park(mcl_scalar vd, mcl_scalar vq, mcl_scalar phase,
                            mcl_scalar *alpha, mcl_scalar *beta);

#ifdef __cplusplus
}
#endif

#endif /* MCL_TRANSFORM_H */
