/**
 * @file    mcl_config.h
 * @brief   mcl 电机控制库：集中配置（用户配置 mcl 的唯一入口）
 *
 * 用户配置 mcl 只需关注本文件：
 *   1. `mcl_config_default(&cfg)` 拿一份安全默认值；
 *   2. 只修改自己需要的字段；
 *   3. `mcl_init()` 会自动校验配置合法性（见 mcl_config_validate）。
 *
 * @code
 *   mcl_config cfg;
 *   mcl_config_default(&cfg);
 *   cfg.pole_pairs = 7;                        // 只改需要改的
 *   cfg.phase_resistance = MCL_FROM_FLOAT(0.5f);
 *   cfg.rated_current = MCL_FROM_FLOAT(20.0f);
 *   mcl_init(&motor, &cfg, &hal, hal_ctx, obs_ops, obs_impl, obs_params);
 * @endcode
 *
 * ============================ 重要注意事项 ============================
 *
 * 1. 【精度模式】mcl_scalar 由编译宏决定（见 mcl_types.h）：
 *    - float（默认）：所有字段是物理量（Ω、H、A、V、rad/s、rpm、s、℃）。
 *    - Q15/Q31 定点：所有物理量必须先按 per-unit 归一化到 [-1,1)（见下 2），
 *      否则 MCL_FROM_FLOAT 会饱和溢出、数值全错。
 *
 * 2. 【per-unit 归一化（定点必修）】定点下 base 必须满足物理约束（详见
 *    docs/spec/mcl_fixed_point.md §9）：
 *        V_BASE = W_BASE·λ_BASE = R_BASE·I_BASE = L_BASE·W_BASE·I_BASE
 *    - 电压类字段（bemf_const 之类）按 V_BASE 归一化；
 *    - 电流类字段按 I_BASE 归一化；
 *    - 电感类字段按 L_BASE 归一化；
 *    - 时间类字段按 T_BASE = 1/W_BASE 归一化。
 *    基值本身常 >1，不能存进字段，只用于「配置时在 float 域换算」。
 *
 * 3. 【角度约定】float 模式用弧度；Q15/Q31 用归一化角度（1.0 = 2π = 一圈），
 *    0.5 = π、0.25 = π/2。含角度的字段（如 openloop_seed_angle）需按此约定。
 *
 * 4. 【时间与 time_base】dt 归一化 dt_pu = dt/T_BASE = dt·W_BASE：
 *    - float：time_base 保持默认 1.0（不归一化，dt 为物理秒）。
 *    - 定点：time_base = 1/W_BASE，且「以秒为单位、与 dt 比较/累加」的阈值
 *      （限值里的 stall_time、fault_stop_time、openloop_*_time）都要随
 *      time_base 一起归一化，否则会提前 1/T_BASE 倍触发（见 fixed_point.md §9.1）。
 *
 * 5. 【PID 增益必须 <1（定点）】Q15/Q31 范围 [-1,1)，kp/ki 通常 <1；
 *    物理量下整定好的增益切定点后需按归一化量纲重新整定。
 *
 * 6. 【rad/s ↔ rpm】库内部速度用「电气角速度 rad/s」（或定点归一化速度）。
 *    机械转速 rpm = 电气 rad/s ÷ pole_pairs × 60/(2π)。
 *    rated_speed_rpm、openloop_rpm 是「机械 rpm」；极对数折算由库处理。
 *
 * 7. 【功能裁剪】若编译时定义了 MCL_DISABLE_*（见 mcl_types.h），对应字段
 *    不再参与运行（如 MCL_DISABLE_OBSERVER 时 PLL/开环字段无效；
 *    MCL_DISABLE_CALIBRATION 时 current_offset 无效）。
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
    /* ==================== 电机参数 ==================== */

    uint8_t    pole_pairs;       /**< 极对数（>0，必填）。
                                      FOC 里用于：电气角↔机械角换算、机械方程、
                                      rpm↔电气 rad/s 换算。 */
    mcl_scalar phase_resistance; /**< 相电阻 R，Ω（float）/ per-unit 归一化（定点）。
                                      用于电流环解耦前馈、观测器。 */
    mcl_scalar phase_inductance; /**< q 轴电感 Lq（= 平均相电感），H。
                                      注意：SPMSM 时 Ld=Lq=此值；IPMSM 时立见 ld_lq_diff。 */
    mcl_scalar ld_lq_diff;       /**< 凸极差 Lq - Ld，H（≥0）。
                                      0 = SPMSM（MTPA 恒 Id=0）；>0 = IPMSM，启用 MTPA 曲线。
                                      实际 Ld = phase_inductance - ld_lq_diff。 */
    mcl_scalar bemf_const;       /**< 反电动势常数 = 永磁磁链 λ，V/(rad/s) = Wb。
                                      用于电流环 vq 前馈（ω·λ）、MTPA、转矩常数。 */
    mcl_scalar rated_current;    /**< 额定（峰值）电流 A。用于温度降额基数、MTPA i_max、
                                      开环电流限幅。>0 必填。 */
    mcl_scalar rated_speed_rpm;  /**< 额定转速，机械 rpm。参考值，暂仅用于弱磁/展示。 */

    /* ==================== 运行配置 ==================== */

    uint32_t   pwm_freq_hz;          /**< PWM 开关频率 Hz（>0）。用于死区/采样相关，当前算法
                                          主要用 current_loop_freq_hz。 */
    uint32_t   current_loop_freq_hz; /**< 电流环（控制 tick）频率 Hz（>0）。
                                          dt = 1 / current_loop_freq_hz。宿主须以此频率
                                          调 mcl_control_tick()。 */
    uint8_t    speed_loop_divider;   /**< 速度环分频（相对电流环，>0）。
                                          速度环频率 = current_loop_freq_hz / divider。 */
    uint8_t    pos_loop_divider;     /**< 位置环分频（相对电流环，>0）。
                                          位置环频率 = current_loop_freq_hz / divider。 */
    mcl_scalar max_duty;             /**< 最大占空比 0~1（≤1）。SVPWM 输出上限，防止过调制。 */
    mcl_scalar bus_voltage;          /**< 标称母线电压 V。用于保护过/欠压基准、SVPWM 电压
                                          归一化基准（per-unit 下 V_BASE=V_BUS 时 = 1）。 */
    mcl_scalar time_base;            /**< per-unit 时间基值 T_BASE，秒（= 1/W_BASE）。
                                          见文件头「注意事项 4」：float 保持 1.0，定点设
                                          T_BASE。dt_pu = dt / time_base。 */

    /* ==================== 控制环 ==================== */

    mcl_pid_params current_pid; /**< 电流环 PID（dq 轴共用）。
                                     注意：输出是电压，out_min/max 按母线归一化幅值；
                                     定点下 kp/ki 须 <1（见注意事项 5）。 */
    mcl_pid_params speed_pid;   /**< 速度环 PID。输出是 iq 参考（电流），
                                     out_min/max 按电流幅值。 */
    mcl_pid_params pos_pid;     /**< 位置环 PID。输出是速度参考（rpm）。 */

    /* ==================== 反馈 ==================== */

    mcl_feedback_cfg feedback;  /**< 反馈配置：类型（无感/编码器/霍尔）、编码器偏移与线数。
                                     无感时 type=MCL_FEEDBACK_NONE；有感须设 encoder 偏移。 */

    /* ==================== PLL（无感相位/速度跟踪） ==================== */

    mcl_scalar pll_kp;          /**< PLL 比例系数（相位误差 → 速度修正）。 */
    mcl_scalar pll_ki;          /**< PLL 积分系数（相位误差积分 → 速度）。 */

    /* ==================== 无感自动开环启动（VESC 式：低速开环拖动 → 切闭环） ====================
       仅无感模式（MCL_FEEDBACK_NONE + observer）使用；有感/裁剪 observer 时无效。 */

    mcl_scalar openloop_rpm;        /**< 开环拖动转速上限，机械 rpm。 */
    mcl_scalar openloop_rpm_low;    /**< 最小电流时的开环转速比例 [0,1]（0=固定 openloop_rpm）。 */
    mcl_scalar openloop_hyst;       /**< 估计速度低于开环阈值持续多久才进开环，s。
                                          时间阈值，定点需随 time_base 归一化（注意事项 4）。 */
    mcl_scalar openloop_time_lock;  /**< 开环序列锁定（id 对齐预定位）时间，s。 */
    mcl_scalar openloop_time_ramp;  /**< 开环序列斜坡加速时间，s。 */
    mcl_scalar openloop_time;       /**< 开环序列匀速保持时间，s。 */
    mcl_scalar openloop_boost_q;    /**< 开环 q 轴电流 boost，A（预留，自适应未启用）。 */
    mcl_scalar openloop_max_q;      /**< 开环 q 轴电流上限，A（<0 表示不限）。 */
    mcl_scalar openloop_drag_q;     /**< 开环拖动阶段 q 轴电流，A（I/F 固定拖动电流）。 */
    mcl_scalar openloop_seed_angle; /**< 退出开环时 seed 观测器的负载角超前量。
                                          float：弧度；定点：归一化圈数。默认 90°(π/2)。
                                          不同电机/负载需标定（空载约 87~90°）。 */

    /* ==================== 保护 ==================== */

    mcl_protection_limits limits; /**< 保护阈值与使能位（见 mcl_types.h MCL_PROTECT_*）。
                                      电流/电压/温度阈值按 float 物理量或 per-unit 归一化；
                                      stall_time 是时间阈值，定点需归一化（注意事项 4）。 */
    mcl_scalar fault_stop_time;   /**< 故障后自动恢复时间，s（0 = 手动清除，不自动恢复）。
                                      时间阈值，定点需归一化（注意事项 4）。 */

    /* ==================== 校准 ==================== */

    mcl_scalar current_offset[3]; /**< 三相电流零漂 A（abc）。由 mcl_calibrate_offset 测得，
                                      采集后从三相电流中减去。MCL_DISABLE_CALIBRATION 下无效。 */
} mcl_config;

/**
 * @brief 填一套安全默认配置（零除保护、不误触发）
 *
 * 默认值：4 极对、R=1Ω、Lq=1mH、λ=0.02Wb、额定 5A/3000rpm、
 * PWM/电流环 20kHz、速度/位置环分频 10、母线 24V、time_base=1.0（float 不归一化）。
 * 保护默认全开（MCL_PROTECT_ALL）。
 *
 * @param cfg 配置（输出，会被整体覆写）
 */
void mcl_config_default(mcl_config *cfg);

/**
 * @brief 校验配置合法性
 *
 * 校验项：极对数>0、频率>0、分频>0、max_duty∈(0,1]、电感>0、rated_current>0、
 * 过压>欠压。校验不通过时 mcl_init 拒绝初始化（进入 FAULT）。
 *
 * @param cfg 配置
 * @return MCL_OK 或 MCL_ERR_PARAM
 */
int mcl_config_validate(const mcl_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MCL_CONFIG_H */
