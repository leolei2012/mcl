/**
 * @file    mcl.c
 * @brief   mcl 电机控制库：门面实现（生命周期、指令、查询、控制节拍、校准封装）
 *
 * 控制节拍 mcl_control_tick 在电流环中断内由宿主调用，
 * 串起采样 → Clarke → 相位/速度 → 外环 → 电流环 → 反 Park → SVPWM → 保护 → 输出。
 */

#include "mcl.h"

/* 60/(2π)：rad/s → rpm 换算系数（TODO 定点：需缩放） */
#define MCL_RPM_PER_RAD_S  9.5492966f
/* 2π/60：rpm → rad/s 换算系数 */
#define MCL_RAD_PER_S_PER_RPM  0.1047197551f

/* ============================ 角度回绕 ============================ */

/* 角度归一到 [0, 整圈)：float 用 2π；定点用 1.0（归一化角） */
static mcl_scalar mcl_wrap_full_turn(mcl_scalar x)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    const mcl_scalar full = MCL_FROM_FLOAT(1.0f);
#else
    const mcl_scalar full = (mcl_scalar)MCL_TWO_PI;
#endif
    while (x >= full)
    {
        x = MCL_SUB(x, full);
    }
    while (x < (mcl_scalar)0)
    {
        x = MCL_ADD(x, full);
    }
    return x;
}

/* ============================ 控制节拍内部实现 ============================ */

/**
 * 无感自动开环→闭环切换（VESC 式，简化）。
 *
 * 仅在 sensorless + 闭环控制模式（CURRENT/SPEED/POSITION）下生效：
 * 估计速度低于开环阈值持续 openloop_hyst 后进入开环，按
 * 锁定(t_lock，id 对齐预定位) → 斜坡(t_ramp) → 匀速(t_const) 三段式
 * 拖动；期间用开环积分相位覆盖观测器相位，并把观测器磁链状态 seed
 * 到开环相位 +90°（空载 I/F 稳态转子超前磁场约 90°），退出时观测器
 * 从转子附近初值继续跟踪，实现平滑切换。
 *
 * 开环阶段写入 self->ol_stage：1=锁定(对齐)，2=拖动，0=未开环；
 * 电流环据此在锁定/拖动阶段分别用 id 对齐 / iq 拖动。
 */
static void mcl_openloop_auto(mcl *self, mcl_scalar *phase, mcl_scalar *speed)
{
    const mcl_scalar dt = self->dt;
    const mcl_scalar pp = (mcl_scalar)self->cfg.pole_pairs;
    const mcl_scalar t_lock = self->cfg.openloop_time_lock;
    const mcl_scalar t_ramp = self->cfg.openloop_time_ramp;
    const mcl_scalar t_const = self->cfg.openloop_time;
    const mcl_scalar total = MCL_ADD(MCL_ADD(t_lock, t_ramp), t_const);
    mcl_scalar ol_rpm_max;
    mcl_scalar ol_speed_max;
    mcl_scalar ol_rpm;
    mcl_scalar time_fwd;
    mcl_scalar dir;
    mcl_scalar sign;
    mcl_scalar s;
    mcl_scalar c;

    /* 开环转速上限（固定值）。
       TODO：VESC 按 |iq| 自适应（电流越大转速越高，I/F），依赖 openloop_rpm_low /
       openloop_boost_q / openloop_max_q；mcl 当前简化为固定 openloop_rpm，后续恢复。 */
    ol_rpm_max = self->cfg.openloop_rpm;
    ol_speed_max = MCL_MUL(MCL_MUL(ol_rpm_max, MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM)), pp);

    /* 3. 拖动方向：速度/位置环用 speed_ref 符号，电流环用 iq_ref 符号 */
    dir = (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
        ? self->speed_ref_rpm : self->iq_ref;
    sign = (dir >= (mcl_scalar)0) ? (mcl_scalar)1 : (mcl_scalar)-1;

    /* 4. 迟滞进入：估计速度低于开环上限则向 hyst 饱和累加，否则递减 */
    if (MCL_ABS(*speed) < ol_speed_max)
    {
        if (self->ol_hyst_timer < self->cfg.openloop_hyst)
        {
            self->ol_hyst_timer = MCL_ADD(self->ol_hyst_timer, dt);
            if (self->ol_hyst_timer > self->cfg.openloop_hyst)
            {
                self->ol_hyst_timer = self->cfg.openloop_hyst;
            }
        }
    }
    else if (self->ol_hyst_timer > (mcl_scalar)0)
    {
        self->ol_hyst_timer = MCL_SUB(self->ol_hyst_timer, dt);
        if (self->ol_hyst_timer < (mcl_scalar)0)
        {
            self->ol_hyst_timer = (mcl_scalar)0;
        }
    }

    /* 5. 启动开环序列 */
    if (self->ol_hyst_timer >= self->cfg.openloop_hyst && self->ol_timer <= (mcl_scalar)0)
    {
        self->ol_timer = total;
        self->ol_phase = *phase;   /* 从当前估计相位继续，保证相位连续 */
    }

    /* 6. 开环运行 */
    if (self->ol_timer > (mcl_scalar)0)
    {
        /* 三段式时间序列：锁定(转速0，id 对齐) → 斜坡(0→max) → 匀速(max) */
        time_fwd = MCL_SUB(total, self->ol_timer);
        if (time_fwd < t_lock)
        {
            ol_rpm = (mcl_scalar)0;
            self->ol_stage = 1;   /* 锁定：固定相位，id 对齐 */
        }
        else
        {
            self->ol_stage = 2;   /* 拖动：相位积分，iq 拖动 */
            ol_rpm = ol_rpm_max;
            if (time_fwd < MCL_ADD(t_lock, t_ramp))
            {
                ol_rpm = MCL_MUL(ol_rpm_max,
                                 MCL_SUB(time_fwd, t_lock) / t_ramp);   /* TODO 定点：MCL_DIV */
            }
        }

        self->ol_speed = MCL_MUL(MCL_MUL(MCL_MUL(ol_rpm,
                                   MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM)), pp), sign);

        /* 锁定阶段相位固定；拖动阶段相位积分 */
        self->ol_phase = mcl_wrap_full_turn(MCL_ADD(self->ol_phase, MCL_MUL(self->ol_speed, dt)));

        *phase = self->ol_phase;
        *speed = self->ol_speed;

        /* seed 观测器到转子实际相位（开环相位 + openloop_seed_angle 负载角补偿），
           帮助退出开环后立即跟踪。空载 I/F 稳态转子超前磁场约 90°，默认按 90° 补偿，
           不同电机/负载可标定此角。 */
        mcl_math_sincos(mcl_wrap_full_turn(MCL_ADD(self->ol_phase,
                        MCL_MUL(sign, self->cfg.openloop_seed_angle))), &s, &c);
        mcl_observer_seed(&self->observer,
                          MCL_MUL(c, self->cfg.bemf_const),
                          MCL_MUL(s, self->cfg.bemf_const));

        self->ol_timer = MCL_SUB(self->ol_timer, dt);
        if (self->ol_timer < (mcl_scalar)0)
        {
            self->ol_timer = (mcl_scalar)0;
        }
        self->ol_hyst_timer = (mcl_scalar)0;
    }
    else
    {
        /* 非开环：相位跟随观测器估计，保证下次进入连续 */
        self->ol_stage = 0;
        self->ol_phase = *phase;
    }
}

static void mcl_control_tick_foc(mcl *self)
{
    mcl_scalar ia;
    mcl_scalar ib;
    mcl_scalar ic;
    mcl_scalar i_alpha;
    mcl_scalar i_beta;
    mcl_scalar id;
    mcl_scalar iq;
    mcl_scalar id_ref;
    mcl_scalar iq_ref;
    mcl_scalar vd;
    mcl_scalar vq;
    mcl_scalar v_alpha;
    mcl_scalar v_beta;
    mcl_scalar da;
    mcl_scalar db;
    mcl_scalar dc;
    mcl_scalar phase;
    mcl_scalar speed;
    mcl_scalar vbus;
    mcl_scalar temp_max;
    mcl_scalar derate;
    mcl_fault fault;

    /* 1. 采样（减零漂） */
    if (self->hal->adc_read_phase != NULL &&
        self->hal->adc_read_phase(self->hal_ctx, &ia, &ib, &ic) != MCL_OK)
    {
        self->state = MCL_STATE_FAULT;
        self->fault = MCL_FAULT_SYNC_LOST;
        return;
    }
    ia = MCL_SUB(ia, self->cfg.current_offset[0]);
    ib = MCL_SUB(ib, self->cfg.current_offset[1]);
    ic = MCL_SUB(ic, self->cfg.current_offset[2]);

    vbus = self->cfg.bus_voltage;
    if (self->hal->adc_read_bus != NULL)
    {
        (void)self->hal->adc_read_bus(self->hal_ctx, &vbus, &ic);
    }
    self->vbus = vbus;

    /* 读温度（电机/FET 取更高者） */
    temp_max = (mcl_scalar)0;
    if (self->hal->read_temp != NULL)
    {
        mcl_scalar t_motor = (mcl_scalar)0;
        mcl_scalar t_fet = (mcl_scalar)0;
        if (self->hal->read_temp(self->hal_ctx, &t_motor, &t_fet) == MCL_OK)
        {
            temp_max = (t_motor > t_fet) ? t_motor : t_fet;
        }
    }
    derate = mcl_protection_derate(&self->protection, temp_max);

    /* 2. Clarke */
    mcl_transform_clarke(ia, ib, ic, &i_alpha, &i_beta);

    /* 3. 相位 / 速度 */
    phase = self->phase_rad;
    speed = self->speed_rad_s;

    if (self->mode == MCL_MODE_FOC_SENSORED)
    {
        if (self->hal->enc_read_angle != NULL)
        {
            (void)self->hal->enc_read_angle(self->hal_ctx, &phase);
        }
        if (self->hal->enc_read_speed != NULL)
        {
            (void)self->hal->enc_read_speed(self->hal_ctx, &speed);
        }
    }
    else /* MCL_MODE_FOC_SENSORLESS */
    {
        /* 观测器（用上一周期电压）估相位，PLL 跟踪 + 估速度 */
        mcl_observer_update(&self->observer, self->v_alpha_prev, self->v_beta_prev,
                            i_alpha, i_beta, self->dt, &phase, NULL);
        mcl_pll_run(&self->pll, phase, self->dt, &phase, &speed);

        /* 自动开环→闭环切换：闭环控制模式下，低速段开环拖动 */
        if (self->ctrl_mode == MCL_CTRL_CURRENT ||
            self->ctrl_mode == MCL_CTRL_SPEED ||
            self->ctrl_mode == MCL_CTRL_POSITION)
        {
            mcl_openloop_auto(self, &phase, &speed);
        }
    }

    /* 开环：相位/速度覆盖（绕过编码器与观测器） */
    if (self->ctrl_mode == MCL_CTRL_OPENLOOP_VF ||
        self->ctrl_mode == MCL_CTRL_OPENLOOP_IF)
    {
        /* 旋转矢量：相位按 openloop_speed 积分斜坡前进 */
        self->openloop_angle = mcl_wrap_full_turn(
            MCL_ADD(self->openloop_angle, MCL_MUL(self->openloop_speed, self->dt)));
        phase = self->openloop_angle;
        speed = self->openloop_speed;
    }
    else if (self->ctrl_mode == MCL_CTRL_OPENLOOP_ALIGN)
    {
        /* 固定矢量：相位锁定（转子预定位） */
        phase = self->openloop_phase;
        speed = (mcl_scalar)0;
    }

    self->phase_rad = phase;
    self->speed_rad_s = speed;

    /* 4. 外环（位置环 → 速度环 → 电流环，按分频级联） */
    iq_ref = self->iq_ref;

    /* 位置环（最外环）：输出速度参考 rpm */
    if (self->ctrl_mode == MCL_CTRL_POSITION && self->cfg.pos_loop_divider > 0u)
    {
        if ((self->tick_count % (uint32_t)self->cfg.pos_loop_divider) == 0u)
        {
            /* 机械角 = 电气角 / pole_pairs；乘倒数 1/pole_pairs（<1，定点可表示） */
            mcl_scalar pos_now = MCL_MUL(phase, MCL_FROM_FLOAT(1.0f / (float)self->cfg.pole_pairs));
            self->speed_ref_rpm = mcl_pid_run(&self->pid_pos,
                                              MCL_SUB(self->pos_ref_rad, pos_now),
                                              MCL_MUL(self->dt, (mcl_scalar)self->cfg.pos_loop_divider));
        }
    }

    /* 速度环：反馈转速 rpm = 电气角速度 rad/s ÷ 极对数 × 60/(2π) */
    if (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
    {
        if (self->cfg.speed_loop_divider > 0u &&
            (self->tick_count % (uint32_t)self->cfg.speed_loop_divider) == 0u)
        {
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
            /* 定点：speed 与 speed_ref_rpm 均已归一化（HAL/调用方在边界分别 ÷电气速度基值
               / ÷rpm 基值；两者数值相等，故直接比较，无 rpm↔rad/s 换算。
               详见 docs/spec/mcl_fixed_point.md） */
            iq_ref = mcl_pid_run(&self->pid_speed,
                                 MCL_SUB(self->speed_ref_rpm, speed),
                                 MCL_MUL(self->dt, (mcl_scalar)self->cfg.speed_loop_divider));
#else
            mcl_scalar fb_rpm = MCL_MUL(speed, MCL_FROM_FLOAT(MCL_RPM_PER_RAD_S /
                                        (float)self->cfg.pole_pairs));
            iq_ref = mcl_pid_run(&self->pid_speed,
                                 MCL_SUB(self->speed_ref_rpm, fb_rpm),
                                 MCL_MUL(self->dt, (mcl_scalar)self->cfg.speed_loop_divider));
#endif
        }
        self->iq_ref = iq_ref;
    }

    /* 温度降额：限制 iq 幅值 */
    if (derate < (mcl_scalar)1)
    {
        mcl_scalar iq_limit = MCL_MUL(self->cfg.rated_current, derate);
        if (iq_ref > iq_limit) { iq_ref = iq_limit; }
        if (iq_ref < MCL_NEG(iq_limit)) { iq_ref = MCL_NEG(iq_limit); }
    }

    /* 5. 电流环：MTPA/弱磁 → Park → PI + 解耦前馈 → 反 Park → SVPWM */
    if (self->ctrl_mode == MCL_CTRL_OPENLOOP_VF)
    {
        /* VF：全开环，直接输出旋转电压矢量（vd = 幅值，vq = 0），电流环不闭合。
           id/iq 仍做 Park 供遥测展示实际 dq 电流。 */
        mcl_transform_park(i_alpha, i_beta, phase, &id, &iq);
        vd = self->openloop_mag;
        vq = (mcl_scalar)0;
    }
    else
    {
        mcl_scalar iq_ref_loop = iq_ref;

        if (self->ctrl_mode == MCL_CTRL_OPENLOOP_IF)
        {
            /* I/F：电流环闭环，相位开环（iq = 幅值，id = 0），不做 MTPA */
            id_ref = (mcl_scalar)0;
            iq_ref_loop = self->openloop_mag;
        }
        else if (self->ctrl_mode == MCL_CTRL_OPENLOOP_ALIGN)
        {
            /* 预定位：固定相位，电流矢量沿 d 轴（id = 幅值，iq = 0），不做 MTPA */
            id_ref = self->openloop_mag;
            iq_ref_loop = (mcl_scalar)0;
        }
        else if (self->ol_stage == 1u)
        {
            /* 自动开环锁定：id 对齐预定位，把转子吸到开环相位 */
            id_ref = self->cfg.rated_current;
            iq_ref_loop = (mcl_scalar)0;
        }
        else if (self->ol_stage == 2u)
        {
            /* 自动开环拖动：I/F 固定 q 轴电流（带方向），id=0，相位开环积分 */
            mcl_scalar dir = (self->ctrl_mode == MCL_CTRL_SPEED || self->ctrl_mode == MCL_CTRL_POSITION)
                           ? self->speed_ref_rpm : self->iq_ref;
            id_ref = (mcl_scalar)0;
            iq_ref_loop = (dir >= (mcl_scalar)0) ? self->cfg.openloop_drag_q
                                                 : MCL_NEG(self->cfg.openloop_drag_q);
        }
        else
        {
            mcl_mtpa_fw_id_ref(&self->mtpa_fw, iq_ref, speed, vbus, &id_ref);
        }

        mcl_transform_park(i_alpha, i_beta, phase, &id, &iq);
        mcl_foc_run(&self->foc, id, iq, id_ref, iq_ref_loop, speed, vbus, self->dt, &vd, &vq);
    }
    mcl_transform_inv_park(vd, vq, phase, &v_alpha, &v_beta);
    mcl_svpwm_run(v_alpha, v_beta, self->cfg.max_duty, &da, &db, &dc);

    /* 缓存电压供下一周期观测器使用 */
    self->v_alpha_prev = v_alpha;
    self->v_beta_prev = v_beta;

    /* 6. 保护 */
    fault = mcl_protection_check(&self->protection, ia, ib, ic, vbus,
                                 temp_max, speed, self->dt);
    if (fault != MCL_FAULT_NONE)
    {
        self->fault = fault;
        self->state = MCL_STATE_FAULT;
        /* 记录故障现场快照（用于诊断） */
        self->fault_info.fault = fault;
        self->fault_info.current = iq;
        self->fault_info.voltage = vbus;
        self->fault_info.speed = speed;
        self->fault_info.temp = temp_max;
        self->fault_info.tick = self->tick_count;
        self->fault_timer = (mcl_scalar)0;
        if (self->hal->pwm_set_duty != NULL)
        {
            self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
        }
        return;
    }

    /* 7. 输出 */
    if (self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, da, db, dc);
    }

    /* 8. 更新遥测缓存 */
    self->id_now = id;
    self->iq_now = iq;
    self->duty_now = da;
}

static void mcl_control_tick_bldc(mcl *self)
{
    mcl_scalar ia;
    mcl_scalar ib;
    mcl_scalar ic;
    mcl_scalar da;
    mcl_scalar db;
    mcl_scalar dc;

    if (self->hal->adc_read_phase == NULL)
    {
        return;
    }
    (void)self->hal->adc_read_phase(self->hal_ctx, &ia, &ib, &ic);

    /* 简化：六步换相（hall 接口 / BEMF 电压采样待接入），此处按 BEMF 过零 */
    mcl_bldc_comm_step_bemf(&self->bldc, ia, ib, ic, self->duty_now, &da, &db, &dc);

    if (self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, da, db, dc);
    }
}

/* ============================ 生命周期 ============================ */

void mcl_init(mcl *self, const mcl_config *cfg,
              const mcl_hal_ops *hal, void *hal_ctx,
              const mcl_observer_ops *obs_ops, void *obs_impl, void *obs_params)
{
    if (self == NULL || cfg == NULL || hal == NULL)
    {
        return;
    }

    /* 配置校验：非法参数直接拒绝初始化 */
    if (mcl_config_validate(cfg) != MCL_OK)
    {
        self->state = MCL_STATE_FAULT;
        self->fault = MCL_FAULT_SYNC_LOST;
        return;
    }

    self->cfg = *cfg;
    self->hal = hal;
    self->hal_ctx = hal_ctx;
    self->mode = MCL_MODE_FOC_SENSORLESS;
    self->state = MCL_STATE_IDLE;
    self->fault = MCL_FAULT_NONE;
    self->ctrl_mode = MCL_CTRL_CURRENT;

    mcl_foc_init(&self->foc, cfg);
    mcl_bldc_comm_init(&self->bldc);
    mcl_observer_init(&self->observer, obs_ops, obs_impl, obs_params);
    mcl_pll_init(&self->pll, cfg->pll_kp, cfg->pll_ki);
    mcl_pid_init(&self->pid_speed, &cfg->speed_pid);
    mcl_pid_init(&self->pid_pos, &cfg->pos_pid);
    mcl_mtpa_fw_init(&self->mtpa_fw, cfg->phase_inductance, cfg->phase_inductance,
                     cfg->bemf_const, cfg->rated_current);
    mcl_protection_init(&self->protection, &cfg->limits);

    self->iq_ref = (mcl_scalar)0;
    self->speed_ref_rpm = (mcl_scalar)0;
    self->pos_ref_rad = (mcl_scalar)0;
    self->phase_rad = (mcl_scalar)0;
    self->speed_rad_s = (mcl_scalar)0;
    self->vbus = cfg->bus_voltage;
    self->id_now = (mcl_scalar)0;
    self->iq_now = (mcl_scalar)0;
    self->duty_now = (mcl_scalar)0;
    self->v_alpha_prev = (mcl_scalar)0;
    self->v_beta_prev = (mcl_scalar)0;
    self->openloop_speed = (mcl_scalar)0;
    self->openloop_angle = (mcl_scalar)0;
    self->openloop_phase = (mcl_scalar)0;
    self->openloop_mag = (mcl_scalar)0;
    self->ol_timer = (mcl_scalar)0;
    self->ol_hyst_timer = (mcl_scalar)0;
    self->ol_speed = (mcl_scalar)0;
    self->ol_phase = (mcl_scalar)0;
    self->ol_stage = 0u;
    self->tick_count = 0u;
    self->fault_timer = (mcl_scalar)0;
    self->fault_info.fault = MCL_FAULT_NONE;
    self->fault_info.current = (mcl_scalar)0;
    self->fault_info.voltage = (mcl_scalar)0;
    self->fault_info.speed = (mcl_scalar)0;
    self->fault_info.temp = (mcl_scalar)0;
    self->fault_info.tick = 0u;

    /* dt = 1/电流环频率 ÷ time_base 归一化（per-unit 时间 dt_pu = dt/T_BASE = dt·W_BASE）。
       time_base 默认 1.0 时保持物理秒；定点设 T_BASE（<1）消除 dt 的 Q15 量化误差。 */
    self->dt = MCL_DIV(MCL_FROM_FLOAT(1.0f / (float)cfg->current_loop_freq_hz), cfg->time_base);
}

void mcl_deinit(mcl *self)
{
    if (self == NULL)
    {
        return;
    }

    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }
    self->state = MCL_STATE_IDLE;
}

int mcl_set_config(mcl *self, const mcl_config *cfg)
{
    if (self == NULL || cfg == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 更新可运行时调整的字段（快照） */
    self->cfg.current_pid = cfg->current_pid;
    self->cfg.speed_pid = cfg->speed_pid;
    self->cfg.pos_pid = cfg->pos_pid;
    self->cfg.limits = cfg->limits;
    self->cfg.fault_stop_time = cfg->fault_stop_time;
    self->cfg.max_duty = cfg->max_duty;
    self->cfg.openloop_rpm = cfg->openloop_rpm;
    self->cfg.openloop_drag_q = cfg->openloop_drag_q;
    self->cfg.openloop_seed_angle = cfg->openloop_seed_angle;

    /* 同步到子模块 */
    mcl_pid_set_params(&self->foc.pid_d, &cfg->current_pid);
    mcl_pid_set_params(&self->foc.pid_q, &cfg->current_pid);
    mcl_pid_set_params(&self->pid_speed, &cfg->speed_pid);
    mcl_pid_set_params(&self->pid_pos, &cfg->pos_pid);
    self->protection.limits = cfg->limits;   /* 只更新阈值，保留堵转计时状态 */

    return MCL_OK;
}

int mcl_get_config(mcl *self, mcl_config *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->cfg;
    return MCL_OK;
}

/* ============================ 模式与启停 ============================ */

int mcl_set_mode(mcl *self, mcl_mode mode)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_ERR_STATE;
    }
    self->mode = mode;
    return MCL_OK;
}

int mcl_start(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_OK;
    }
    if (self->fault != MCL_FAULT_NONE)
    {
        return MCL_ERR_STATE;
    }

    self->state = MCL_STATE_RUN;
    self->tick_count = 0u;

    /* 无感闭环启动：预充电开环迟滞，使首拍即进入开环（预定位 + 拖动），
       避免低速段观测器未收敛时闭环乱甩转子（VESC 式自动开环启动） */
    if (self->mode == MCL_MODE_FOC_SENSORLESS &&
        (self->ctrl_mode == MCL_CTRL_CURRENT ||
         self->ctrl_mode == MCL_CTRL_SPEED ||
         self->ctrl_mode == MCL_CTRL_POSITION))
    {
        self->ol_hyst_timer = self->cfg.openloop_hyst;
    }

    return MCL_OK;
}

int mcl_stop(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }
    self->state = MCL_STATE_IDLE;
    return MCL_OK;
}

/* ============================ 指令 ============================ */

int mcl_set_current(mcl *self, mcl_scalar iq_ref)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_CURRENT;
    self->iq_ref = iq_ref;
    return MCL_OK;
}

int mcl_set_speed(mcl *self, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_SPEED;
    self->speed_ref_rpm = speed_rpm;
    return MCL_OK;
}

int mcl_set_position(mcl *self, mcl_scalar pos_rad)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    self->ctrl_mode = MCL_CTRL_POSITION;
    self->pos_ref_rad = pos_rad;
    return MCL_OK;
}

int mcl_set_torque(mcl *self, mcl_scalar torque_nm)
{
    mcl_scalar kt;

    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* iq = T / (1.5 * pole_pairs * lambda)（TODO 定点：除法） */
    kt = MCL_MUL(MCL_MUL((mcl_scalar)1.5f, (mcl_scalar)self->cfg.pole_pairs), self->cfg.bemf_const);
    self->ctrl_mode = MCL_CTRL_CURRENT;
    self->iq_ref = torque_nm / kt;
    return MCL_OK;
}

/* ============================ 开环指令 ============================ */

int mcl_set_openloop_vf(mcl *self, mcl_scalar voltage, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电压幅值限幅到 [-1, 1]（标幺，1.0 = 满母线） */
    if (voltage > (mcl_scalar)1) { voltage = (mcl_scalar)1; }
    if (voltage < (mcl_scalar)-1) { voltage = (mcl_scalar)-1; }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_VF;
    self->openloop_mag = voltage;
    self->openloop_speed = MCL_MUL(
        MCL_MUL(speed_rpm, MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM)),
        (mcl_scalar)self->cfg.pole_pairs);
    return MCL_OK;
}

int mcl_set_openloop_if(mcl *self, mcl_scalar current, mcl_scalar speed_rpm)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电流幅值限幅到额定电流 */
    if (current > self->cfg.rated_current) { current = self->cfg.rated_current; }
    if (current < MCL_NEG(self->cfg.rated_current)) { current = MCL_NEG(self->cfg.rated_current); }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_IF;
    self->openloop_mag = current;
    self->openloop_speed = MCL_MUL(
        MCL_MUL(speed_rpm, MCL_FROM_FLOAT(MCL_RAD_PER_S_PER_RPM)),
        (mcl_scalar)self->cfg.pole_pairs);
    return MCL_OK;
}

int mcl_set_openloop_align(mcl *self, mcl_scalar current, mcl_scalar phase_rad)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    /* 电流幅值限幅到额定电流 */
    if (current > self->cfg.rated_current) { current = self->cfg.rated_current; }
    if (current < MCL_NEG(self->cfg.rated_current)) { current = MCL_NEG(self->cfg.rated_current); }

    self->ctrl_mode = MCL_CTRL_OPENLOOP_ALIGN;
    self->openloop_mag = current;
    self->openloop_phase = mcl_wrap_full_turn(phase_rad);
    /* 预定位固定相位，不转动 */
    self->openloop_speed = (mcl_scalar)0;
    return MCL_OK;
}

/* ============================ 查询 ============================ */

int mcl_get_state(mcl *self, mcl_state *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->state;
    return MCL_OK;
}

int mcl_get_fault(mcl *self, mcl_fault *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->fault;
    return MCL_OK;
}

int mcl_get_fault_info(mcl *self, mcl_fault_info *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }
    *out = self->fault_info;
    return MCL_OK;
}

int mcl_fault_assert(mcl *self, mcl_fault fault)
{
    if (self == NULL || fault == MCL_FAULT_NONE)
    {
        return MCL_ERR_PARAM;
    }

    self->fault = fault;
    self->state = MCL_STATE_FAULT;

    /* 记录快照（硬件保护在中断里，用最近缓存值） */
    self->fault_info.fault = fault;
    self->fault_info.current = self->iq_now;
    self->fault_info.voltage = self->vbus;
    self->fault_info.speed = self->speed_rad_s;
    self->fault_info.temp = (mcl_scalar)0;
    self->fault_info.tick = self->tick_count;
    self->fault_timer = (mcl_scalar)0;

    /* 立即关断 PWM */
    if (self->hal != NULL && self->hal->pwm_set_duty != NULL)
    {
        self->hal->pwm_set_duty(self->hal_ctx, (mcl_scalar)0, (mcl_scalar)0, (mcl_scalar)0);
    }

    return MCL_OK;
}

int mcl_clear_fault(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    if (self->state == MCL_STATE_RUN)
    {
        return MCL_ERR_STATE;
    }
    self->fault = MCL_FAULT_NONE;
    self->state = MCL_STATE_IDLE;
    self->fault_timer = (mcl_scalar)0;
    return MCL_OK;
}

int mcl_get_telemetry(mcl *self, mcl_telemetry *out)
{
    if (self == NULL || out == NULL)
    {
        return MCL_ERR_PARAM;
    }

    out->speed_rpm = self->speed_rad_s;       /* TODO：rad/s → rpm 换算 */
    out->position_rad = self->phase_rad;
    out->iq = self->iq_now;
    out->id = self->id_now;
    out->vbus = self->vbus;
    out->ibus = (mcl_scalar)0;
    out->duty = self->duty_now;
    out->temp_motor = (mcl_scalar)0;
    out->temp_fet = (mcl_scalar)0;
    out->est_phase = self->phase_rad;
    out->est_speed_rad_s = self->speed_rad_s;
    return MCL_OK;
}

/* ============================ 控制节拍 ============================ */

void mcl_control_tick(mcl *self)
{
    if (self == NULL || self->hal == NULL)
    {
        return;
    }

    /* 故障自动恢复计时（VESC 式：fault_stop_time 后自动恢复） */
    if (self->state == MCL_STATE_FAULT && self->cfg.fault_stop_time > (mcl_scalar)0)
    {
        self->fault_timer = MCL_ADD(self->fault_timer, self->dt);
        if (self->fault_timer >= self->cfg.fault_stop_time)
        {
            self->state = MCL_STATE_IDLE;
            self->fault = MCL_FAULT_NONE;
            self->fault_timer = (mcl_scalar)0;
        }
    }

    if (self->state != MCL_STATE_RUN)
    {
        return;
    }

    if (self->mode == MCL_MODE_FOC_SENSORED || self->mode == MCL_MODE_FOC_SENSORLESS)
    {
        mcl_control_tick_foc(self);
    }
    else
    {
        mcl_control_tick_bldc(self);
    }

    self->tick_count++;
}

/* ============================ 校准封装 ============================ */

int mcl_calibrate_offset(mcl *self)
{
    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_current_offset(self->hal, self->hal_ctx, 256u, self->cfg.current_offset);
}

int mcl_calibrate_align(mcl *self)
{
    mcl_scalar offset;
    int ret;

    if (self == NULL)
    {
        return MCL_ERR_PARAM;
    }

    ret = mcl_cal_encoder_align(self->hal, self->hal_ctx, &self->cfg,
                                      self->cfg.rated_current, &offset);
    if (ret == MCL_OK)
    {
        self->cfg.feedback.encoder_offset = offset;
    }
    return ret;
}

int mcl_calibrate_resistance(mcl *self, mcl_scalar *resistance)
{
    if (self == NULL || resistance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_resistance(self->hal, self->hal_ctx, &self->cfg,
                                    self->cfg.rated_current, resistance);
}

int mcl_calibrate_inductance(mcl *self, mcl_scalar *inductance)
{
    if (self == NULL || inductance == NULL)
    {
        return MCL_ERR_PARAM;
    }
    return mcl_cal_inductance(self->hal, self->hal_ctx, &self->cfg,
                                    self->cfg.max_duty, inductance);
}
