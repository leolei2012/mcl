/**
 * @file    mcl_config.h
 * @brief   mcl 电机控制库：配置（用户设置入口）
 *
 * 用户配置 mcl 只需关注本文件：
 *   1. mcl_config_default(&cfg) 拿一份安全默认值；
 *   2. 只修改自己需要的字段；
 *   3. mcl_init() 会自动校验配置合法性。
 *
 * 示例：
 * @code
 *   mcl_config cfg;
 *   mcl_config_default(&cfg);
 *   cfg.pole_pairs = 7;                 // 只改需要改的
 *   cfg.phase_resistance = MCL_FROM_FLOAT(0.5f);
 *   cfg.rated_current = MCL_FROM_FLOAT(20.0f);
 *   mcl_init(&motor, &cfg, &hal, hal_ctx, obs_ops, obs_impl, obs_params);
 * @endcode
 */

#ifndef MCL_CONFIG_H
#define MCL_CONFIG_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 集中配置（唯一事实源）
 */
typedef struct
{
    /* ---- 电机参数 ---- */
    uint8_t    pole_pairs;       /**< 极对数 */
    mcl_scalar phase_resistance; /**< 相电阻 Ω */
    mcl_scalar phase_inductance; /**< 相电感 H */
    mcl_scalar bemf_const;       /**< 反电动势常数（= 磁链 λ）V/(rad/s) */
    mcl_scalar rated_current;    /**< 额定电流 A */
    mcl_scalar rated_speed_rpm;  /**< 额定转速 rpm */

    /* ---- 运行配置 ---- */
    uint32_t   pwm_freq_hz;          /**< PWM 频率 Hz */
    uint32_t   current_loop_freq_hz; /**< 电流环频率 Hz */
    uint8_t    speed_loop_divider;   /**< 速度环分频（相对电流环） */
    uint8_t    pos_loop_divider;     /**< 位置环分频（相对电流环） */
    mcl_scalar max_duty;             /**< 最大占空比 0~1 */
    mcl_scalar bus_voltage;          /**< 标称母线电压 V */
    mcl_scalar time_base;            /**< per-unit 时间基值 T_BASE（秒，=1/W_BASE，<1 可存定点）；
                                          dt 归一化 dt_pu = dt/T_BASE = dt·W_BASE。
                                          float 默认 1.0 = 不归一化（dt 为物理秒） */

    /* ---- 控制环 ---- */
    mcl_pid_params current_pid; /**< 电流环 PID */
    mcl_pid_params speed_pid;   /**< 速度环 PID */
    mcl_pid_params pos_pid;     /**< 位置环 PID */

    /* ---- 反馈 ---- */
    mcl_feedback_cfg feedback;  /**< 反馈配置 */

    /* ---- PLL（无感速度/相位跟踪） ---- */
    mcl_scalar pll_kp;          /**< PLL 比例 */
    mcl_scalar pll_ki;          /**< PLL 积分 */

    /* ---- 无感自动开环启动（VESC 式，低速开环拖动 → 切闭环） ---- */
    mcl_scalar openloop_rpm;        /**< 开环转速上限 rpm（机械） */
    mcl_scalar openloop_rpm_low;    /**< 最小电流时开环转速比例 [0,1] */
    mcl_scalar openloop_hyst;       /**< 低速持续多久才进开环 s */
    mcl_scalar openloop_time_lock;  /**< 开环序列锁定（预定位）时间 s */
    mcl_scalar openloop_time_ramp;  /**< 开环序列斜坡加速时间 s */
    mcl_scalar openloop_time;       /**< 开环序列匀速保持时间 s */
    mcl_scalar openloop_boost_q;    /**< 开环 q 轴电流 boost A */
    mcl_scalar openloop_max_q;      /**< 开环 q 轴电流上限 A（<0 表示不限） */
    mcl_scalar openloop_drag_q;     /**< 开环拖动阶段 q 轴电流 A（I/F 固定电流） */
    mcl_scalar openloop_seed_angle; /**< 退出开环 seed 观测器的负载角超前量（float：rad；定点：归一化圈数，默认 90°） */

    /* ---- 保护 ---- */
    mcl_protection_limits limits; /**< 保护阈值 */
    mcl_scalar fault_stop_time;   /**< 故障后自动恢复时间 s（0 = 手动清除） */

    /* ---- 校准 ---- */
    mcl_scalar current_offset[3]; /**< 三相电流零漂 A */
} mcl_config;

/**
 * @brief 填一套安全默认配置（零除保护、不误触发）
 * @param cfg 配置（输出，会被整体覆写）
 */
void mcl_config_default(mcl_config *cfg);

/**
 * @brief 校验配置合法性
 * @param cfg 配置
 * @return MCL_OK 或 MCL_ERR_PARAM
 */
int mcl_config_validate(const mcl_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MCL_CONFIG_H */
