/**
 * @file    mcl.h
 * @brief   mcl 电机控制库：门面总头（宿主唯一入口）
 *
 * 聚合全部公开头，定义电机对象与门面 API：
 * 生命周期、模式、指令、查询、控制节拍、校准封装。
 *
 * 使用约定：
 *  - 宿主持有一个 mcl 实例（如 static mcl s_motor;），不分配堆内存。
 *  - 宿主在电流环中断内以固定频率调用 mcl_control_tick()。
 *  - 库不注册中断、不访问寄存器；硬件经 mcl_hal_ops 注入。
 */

#ifndef MCL_H
#define MCL_H

#include "mcl_types.h"
#include "mcl_config.h"
#include "mcl_math.h"
#include "mcl_hal.h"
#include "mcl_transform.h"
#include "mcl_pid.h"
#include "mcl_svpwm.h"
#include "mcl_observer.h"
#include "mcl_pll.h"
#include "mcl_foc.h"
#include "mcl_mtpa_fw.h"
#include "mcl_bldc_comm.h"
#include "mcl_protection.h"
#include "mcl_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机对象（宿主持有，静态分配）
 *
 * 结构体完整暴露仅为满足静态分配；内部字段标注 private，宿主不直接访问，
 * 一律通过 mcl_* API 操作。
 */
typedef struct
{
    mcl_config cfg;             /**< 配置快照 */
    mcl_mode   mode;            /**< 运行模式 */
    mcl_state  state;           /**< 运行状态 */
    mcl_fault  fault;           /**< 当前故障 */

    const mcl_hal_ops *hal;     /**< HAL 注入 */
    void              *hal_ctx; /**< HAL 上下文 */

    /* ---- 子模块实例（private） ---- */
    mcl_foc        foc;         /**< FOC 电流环编排 */
    mcl_bldc_comm  bldc;        /**< 六步换相编排 */
    mcl_observer   observer;    /**< 观测器载体（ops + impl + params） */
    mcl_pll        pll;         /**< PLL */
    mcl_pid        pid_speed;   /**< 速度环 */
    mcl_pid        pid_pos;     /**< 位置环 */
    mcl_mtpa_fw    mtpa_fw;     /**< MTPA / 弱磁 */
    mcl_protection protection;  /**< 保护 */

    /* ---- 运行时变量（private） ---- */
    mcl_ctrl_mode ctrl_mode;    /**< 控制模式（电流/速度/位置/开环） */
    mcl_scalar iq_ref;          /**< 当前 Iq 目标 A */
    mcl_scalar speed_ref_rpm;   /**< 速度目标 rpm */
    mcl_scalar pos_ref_rad;     /**< 位置目标 rad */
    mcl_scalar phase_rad;       /**< 当前电气角 rad */
    mcl_scalar speed_rad_s;     /**< 当前速度 rad/s */
    mcl_scalar vbus;            /**< 母线电压缓存 V */
    mcl_scalar id_now;          /**< 当前 Id A */
    mcl_scalar iq_now;          /**< 当前 Iq A */
    mcl_scalar duty_now;        /**< 当前占空比 */
    mcl_scalar v_alpha_prev;    /**< 上一周期 α 电压（观测器输入） */
    mcl_scalar v_beta_prev;     /**< 上一周期 β 电压（观测器输入） */
    mcl_scalar dt;              /**< 电流环周期 s（init 预计算） */
    mcl_scalar openloop_speed;  /**< 开环电气角速度 rad/s */
    mcl_scalar openloop_angle;  /**< 开环累计相位 rad */
    mcl_scalar openloop_phase;  /**< 开环固定相位 rad（预定位） */
    mcl_scalar openloop_mag;    /**< 开环幅值（VF：电压标幺 [-1,1]；IF/ALIGN：电流 A） */
    mcl_scalar ol_timer;        /**< 自动开环序列倒计时 s（>0 表示正在开环） */
    mcl_scalar ol_hyst_timer;   /**< 自动开环低速迟滞计时 s */
    mcl_scalar ol_speed;        /**< 自动开环当前电气角速度 rad/s */
    mcl_scalar ol_phase;        /**< 自动开环积分相位 rad */
    uint8_t   ol_stage;         /**< 自动开环阶段：0=未开环 1=锁定(对齐) 2=拖动 */
    uint32_t tick_count;        /**< 控制周期计数（分频用） */
    mcl_fault_info fault_info;  /**< 故障现场快照 */
    mcl_scalar fault_timer;     /**< 故障恢复计时 s */
} mcl;

/* ============================ 生命周期 ============================ */

/**
 * @brief 初始化电机对象
 * @param self       电机对象
 * @param cfg        集中配置（内部会拷贝快照）
 * @param hal        HAL 操作集（宿主实现）
 * @param hal_ctx    HAL 上下文（回调原样传回）
 * @param obs_ops    观测器算法接口（无感模式必需；有感模式可传 NULL）
 * @param obs_impl   观测器实例（实现方分配）
 * @param obs_params 观测器参数（实现方自定义，可为 NULL）
 */
void mcl_init(mcl *self, const mcl_config *cfg,
              const mcl_hal_ops *hal, void *hal_ctx,
              const mcl_observer_ops *obs_ops, void *obs_impl, void *obs_params);

/**
 * @brief 反初始化（停转并复位到 IDLE）
 * @param self 电机对象
 */
void mcl_deinit(mcl *self);

/**
 * @brief 运行时更新配置（仅可调参数）
 *
 * 只采纳可运行时调整的字段：PID 参数、保护阈值、fault_stop_time、max_duty。
 * 结构性字段（电机参数、频率、分频、反馈类型、校准零漂）运行时不更新，
 * 需重新 mcl_init 才能生效。
 *
 * @param self 电机对象
 * @param cfg  新配置（只采纳可运行时调整的字段）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_config(mcl *self, const mcl_config *cfg);

/**
 * @brief 读出当前配置快照
 * @param self 电机对象
 * @param out  配置（输出，拷贝当前快照）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_get_config(mcl *self, mcl_config *out);

/* ============================ 模式与启停 ============================ */

/**
 * @brief 切换运行模式
 * @param self 电机对象
 * @param mode 目标模式
 * @return MCL_OK / MCL_ERR_STATE（运行中不可切换）
 */
int mcl_set_mode(mcl *self, mcl_mode mode);

/**
 * @brief 启动电机
 * @param self 电机对象
 * @return MCL_OK / MCL_ERR_STATE / MCL_ERR_HAL
 */
int mcl_start(mcl *self);

/**
 * @brief 停止电机
 * @param self 电机对象
 * @return MCL_OK / MCL_ERR_STATE
 */
int mcl_stop(mcl *self);

/* ============================ 指令 ============================ */

/**
 * @brief 电流环指令（q 轴电流）
 * @param self   电机对象
 * @param iq_ref q 轴电流参考 A
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_current(mcl *self, mcl_scalar iq_ref);

/**
 * @brief 速度环指令
 * @param self      电机对象
 * @param speed_rpm 速度参考 rpm
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_speed(mcl *self, mcl_scalar speed_rpm);

/**
 * @brief 位置环指令
 * @param self    电机对象
 * @param pos_rad 位置参考 rad
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_position(mcl *self, mcl_scalar pos_rad);

/**
 * @brief 转矩指令（经 MTPA 换算为 d/q 电流参考）
 * @param self     电机对象
 * @param torque_nm 转矩参考 N·m
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_torque(mcl *self, mcl_scalar torque_nm);

/**
 * @brief 开环旋转电压矢量（V/F，全开环，参考 VESC OPENLOOP_DUTY）
 *
 * 相位按 speed_rpm 积分斜坡前进，绕过编码器/观测器；电流环也开环，
 * 直接输出幅值 voltage 的旋转电压矢量（经 SVPWM）。用于无感启动拖动、
 * 磁链/电感测量时的开环激励。
 *
 * @param self      电机对象
 * @param voltage   电压矢量幅值（标幺，[-1,1]，1.0 = 满母线）
 * @param speed_rpm 目标机械转速 rpm（相位斜坡斜率）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_openloop_vf(mcl *self, mcl_scalar voltage, mcl_scalar speed_rpm);

/**
 * @brief 开环旋转电流矢量（I/F，电流环闭环、相位开环，参考 VESC OPENLOOP）
 *
 * 相位按 speed_rpm 积分斜坡前进，电流环仍闭环（iq = current，id = 0）。
 * 相比 VF 更易在开环→无感闭环切换时保持相位连续（两者都用电流环）。
 *
 * @param self      电机对象
 * @param current   电流矢量幅值 A（q 轴电流参考）
 * @param speed_rpm 目标机械转速 rpm
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_openloop_if(mcl *self, mcl_scalar current, mcl_scalar speed_rpm);

/**
 * @brief 开环固定电流矢量（转子预定位 / 锁定，参考 VESC OPENLOOP_PHASE）
 *
 * 相位固定为 phase_rad，电流矢量幅值 current（沿 d 轴 = 对齐方向）。
 * 用于无感启动前把转子拉到已知角度，或编码器电零位对齐。
 *
 * @param self      电机对象
 * @param current   电流幅值 A
 * @param phase_rad 电气角 rad（float 模式；定点为归一化角）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_set_openloop_align(mcl *self, mcl_scalar current, mcl_scalar phase_rad);

/* ============================ 查询 ============================ */

/**
 * @brief 查询运行状态
 * @param self 电机对象
 * @param out  状态（输出）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_get_state(mcl *self, mcl_state *out);

/**
 * @brief 查询当前故障
 * @param self 电机对象
 * @param out  故障码（输出）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_get_fault(mcl *self, mcl_fault *out);

/**
 * @brief 查询故障现场快照（关断时刻的电流/电压/转速/温度）
 * @param self 电机对象
 * @param out  故障信息（输出）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_get_fault_info(mcl *self, mcl_fault_info *out);

/**
 * @brief 硬件保护触发上报（宿主在 BRK 刹车 / EXTI 中断里调用）
 *
 * 由硬件电路（过流比较器、门驱 nFAULT 引脚等）触发，宿主在相应中断里
 * 调用本函数通知库；库立即进 FAULT、记录快照并关断 PWM。
 *
 * @param self  电机对象
 * @param fault 故障类型（如 MCL_FAULT_OVERCURRENT / MCL_FAULT_DRV）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_fault_assert(mcl *self, mcl_fault fault);

/**
 * @brief 清除故障（回到 IDLE）
 * @param self 电机对象
 * @return MCL_OK / MCL_ERR_STATE
 */
int mcl_clear_fault(mcl *self);

/**
 * @brief 读取遥测
 * @param self 电机对象
 * @param out  遥测（输出）
 * @return MCL_OK / MCL_ERR_PARAM
 */
int mcl_get_telemetry(mcl *self, mcl_telemetry *out);

/* ============================ 控制节拍 ============================ */

/**
 * @brief 执行一个控制周期（宿主在电流环中断内以固定频率调用）
 *
 * 内部流程：采样 → Clarke → 相位/速度（编码器或观测器+PLL）→
 * 外环分频（位置/速度）→ 电流环 → 反 Park → SVPWM → 保护 → 输出。
 *
 * @param self 电机对象
 */
void mcl_control_tick(mcl *self);

/* ============================ 校准（阻塞式，电机停转时调用） ============================ */

/**
 * @brief 电流零漂校准
 * @param self 电机对象
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_calibrate_offset(mcl *self);

/**
 * @brief 编码器电零位对齐
 * @param self 电机对象
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_calibrate_align(mcl *self);

/**
 * @brief 相电阻测量
 * @param self       电机对象
 * @param resistance 相电阻 Ω（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_calibrate_resistance(mcl *self, mcl_scalar *resistance);

/**
 * @brief 相电感测量
 * @param self       电机对象
 * @param inductance 相电感 H（输出）
 * @return MCL_OK / MCL_ERR_HAL
 */
int mcl_calibrate_inductance(mcl *self, mcl_scalar *inductance);

#ifdef __cplusplus
}
#endif

#endif /* MCL_H */
