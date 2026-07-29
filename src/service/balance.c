/**
 * @file balance.c
 * @brief 车载滚球摆杆平衡控制服务实现
 */

#include "balance.h"

#include "imu/imu.h"

#include <math.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BALANCE_GRAVITY_M_S2 9.80665f
#define BALANCE_MOTOR_SEND_DIV 2u
#define BALANCE_MOTOR_WRITE_FAIL_LIMIT 5u
#define BALANCE_MOTOR_ENABLE_RETRY_MS 100u

// ! ========================= 变 量 声 明 ========================= ! //

static BalanceConfig s_config = { 0 };
static BalanceStatus s_status = { 0 };
static Pid s_pos_pid = { 0 };
static bool s_initialized = false;
static bool s_enable_requested = false;
static bool s_fault_braked = false;
static uint8_t s_motor_send_tick = 0u;
static uint8_t s_motor_write_fail_count = 0u;
static uint8_t s_sweep_phase = 0u;
static uint32_t s_state_enter_ms = 0u;
static uint32_t s_last_imu_ms = 0u;
static uint32_t s_last_vision_ms = 0u;
static uint32_t s_last_valid_vision_ms = 0u;
static uint32_t s_prev_vision_ms = 0u;
static uint32_t s_motor_cmd_start_ms = 0u;
static uint32_t s_motor_enable_ms = 0u;
static bool s_motor_cmd_ready = false;
static bool s_return_zero_after_calib = false;
static uint32_t s_sweep_stable_start_ms = 0u;
static uint16_t s_calib_count = 0u;
static float s_pitch_sum = 0.0f;
static float s_acc_sum = 0.0f;
static float s_pitch_zero = 0.0f;
static float s_acc_zero = 0.0f;
static float s_acc_filt = 0.0f;
static float s_ball_filt = 0.0f;
static float s_ball_vel_filt = 0.0f;
static float s_ball_prev = 0.0f;
static float s_vis_angle = 0.0f;
static bool s_ball_filter_ready = false;

static volatile uint32_t s_motor_rx_ms = 0u;
static volatile float s_motor_rx_pos = 0.0f;
static volatile float s_motor_rx_spd = 0.0f;
static volatile float s_motor_rx_tor = 0.0f;
static volatile uint8_t s_motor_rx_error = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief balance_now_ms 内部辅助函数
 * @return 返回值
 */
static uint32_t balance_now_ms(void);
/**
 * @brief balance_clamp 内部辅助函数
 * @param value 数值
 * @param min 参数值
 * @param max 参数值
 * @return 返回值
 */
static float balance_clamp(float value, float min, float max);
/**
 * @brief balance_abs 内部辅助函数
 * @param value 数值
 * @return 返回值
 */
static float balance_abs(float value);
/**
 * @brief balance_lpf 内部辅助函数
 * @param input 参数值
 * @param output 参数值
 * @param tau_s 参数值
 * @param dt_s 周期时间，单位 s
 * @return 返回值
 */
static float balance_lpf(float input, float output, float tau_s, float dt_s);
/**
 * @brief balance_get_acc_axis 内部辅助函数
 * @param acc 参数值
 * @return 返回值
 */
static float balance_get_acc_axis(const ImuAcc* acc);
/**
 * @brief balance_value_valid 内部辅助函数
 * @param value 数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool balance_value_valid(float value);
/**
 * @brief balance_motor_fault 内部辅助函数
 * @param error_code 参数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool balance_motor_fault(uint8_t error_code);
/**
 * @brief balance_tune_valid 内部辅助函数
 * @param config 配置结构体指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool balance_tune_valid(const BalanceTuneConfig* config);
/**
 * @brief balance_mech_changed 内部辅助函数
 * @param config 配置结构体指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool balance_mech_changed(const BalanceTuneConfig* config);
/**
 * @brief balance_update_pid_mode 内部辅助函数
 */
static void balance_update_pid_mode(void);
/**
 * @brief balance_set_state 内部辅助函数
 * @param state 状态枚举值
 */
static void balance_set_state(BalanceState state);
/**
 * @brief balance_raise_fault 内部辅助函数
 * @param fault 参数值
 */
static void balance_raise_fault(BalanceFault fault);
/**
 * @brief balance_clear_fault 内部辅助函数
 */
static void balance_clear_fault(void);
/**
 * @brief balance_update_motor_feedback 内部辅助函数
 * @param now_ms 参数值
 */
static void balance_update_motor_feedback(uint32_t now_ms);
/**
 * @brief balance_update_imu 内部辅助函数
 * @param now_ms 参数值
 * @param dt_s 周期时间，单位 s
 */
static void balance_update_imu(uint32_t now_ms, float dt_s);
/**
 * @brief balance_update_online 内部辅助函数
 * @param now_ms 参数值
 */
static void balance_update_online(uint32_t now_ms);
/**
 * @brief balance_update_sweep 内部辅助函数
 * @param now_ms 参数值
 */
static void balance_update_sweep(uint32_t now_ms);
/**
 * @brief balance_update_control 内部辅助函数
 * @param dt_s 周期时间，单位 s
 */
static void balance_update_control(float dt_s);
/**
 * @brief balance_send_motor 内部辅助函数
 * @param now_ms 参数值
 */
static void balance_send_motor(uint32_t now_ms);
/**
 * @brief balance_enable_motor 内部辅助函数
 * @param now_ms 参数值
 */
static void balance_enable_motor(uint32_t now_ms);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BalanceStatusCode balance_init(const BalanceConfig* config) {
    s_initialized = false;

    if(config == 0 || config->port_ops.now_ms == 0) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(config->motor_id == 0u || config->motor_min_rad >= config->motor_max_rad ||
       config->motor_speed_rad_s <= 0.0f || config->motor_rate_rad_s <= 0.0f ||
       config->motor_sign == 0.0f || config->acc_sign == 0.0f ||
       config->pitch_sign == 0.0f || config->ball_sign == 0.0f ||
       config->acc_tau_s < 0.0f || config->pos_tau_s < 0.0f ||
       config->vel_tau_s < 0.0f || config->vision_angle_max_rad <= 0.0f ||
       config->vision_decay_tau_s < 0.0f || config->target_limit_m <= 0.0f ||
       config->vision_timeout_ms == 0u || config->imu_timeout_ms == 0u ||
       config->motor_timeout_ms == 0u || config->sweep_pos_m <= 0.0f ||
       config->sweep_err_m <= 0.0f || config->sweep_vel_m_s <= 0.0f ||
       config->sweep_stable_ms == 0u || config->calib_samples == 0u ||
       config->acc_axis > BALANCE_AXIS_Z ||
       (config->ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) != 0u ||
       config->pos_pid.max_out <= 0.0f ||
       config->pos_pid.diff_filter_alpha < 0.0f ||
       config->pos_pid.diff_filter_alpha > 1.0f ||
       config->motor_zero_rad < config->motor_min_rad ||
       config->motor_zero_rad > config->motor_max_rad) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(imu_instance == 0 || bus_motor_instance == 0) {
        return BALANCE_STATUS_DEPENDENCY_MISSING;
    }

    s_config = *config;
    memset(&s_status, 0, sizeof(s_status));
    pid_init_cfg(&s_pos_pid, &s_config.pos_pid);
    balance_update_pid_mode();

    s_enable_requested = s_config.auto_enable;
    s_status.enable_requested = s_enable_requested;
    s_fault_braked = false;
    s_motor_send_tick = 0u;
    s_motor_write_fail_count = 0u;
    s_sweep_phase = 0u;
    s_last_imu_ms = 0u;
    s_last_vision_ms = 0u;
    s_last_valid_vision_ms = 0u;
    s_prev_vision_ms = 0u;
    s_motor_cmd_start_ms = 0u;
    s_motor_enable_ms = 0u;
    s_motor_cmd_ready = false;
    s_return_zero_after_calib = false;
    s_sweep_stable_start_ms = 0u;
    s_calib_count = 0u;
    s_pitch_sum = 0.0f;
    s_acc_sum = 0.0f;
    s_pitch_zero = 0.0f;
    s_acc_zero = 0.0f;
    s_acc_filt = 0.0f;
    s_ball_filt = 0.0f;
    s_ball_vel_filt = 0.0f;
    s_ball_prev = 0.0f;
    s_vis_angle = 0.0f;
    s_ball_filter_ready = false;
    s_motor_rx_ms = 0u;
    s_motor_rx_pos = 0.0f;
    s_motor_rx_spd = 0.0f;
    s_motor_rx_tor = 0.0f;
    s_motor_rx_error = 0u;

    s_status.mode = BALANCE_MODE_HOLD_CENTER;
    s_status.target_m = 0.0f;
    s_status.ctrl_mask = s_config.ctrl_mask;
    s_status.motor_cmd_rad = s_config.motor_zero_rad;
    s_initialized = true;
    balance_set_state(BALANCE_STATE_CALIB);
    return BALANCE_STATUS_OK;
}

void balance_process(float dt_s) {
    uint32_t now_ms;

    if(!s_initialized || dt_s <= 0.0f) {
        return;
    }

    now_ms = balance_now_ms();
    s_status.stamp_ms = now_ms;
    balance_update_motor_feedback(now_ms);
    balance_update_imu(now_ms, dt_s);
    balance_update_online(now_ms);
    s_status.ball_loop_closed = s_status.state == BALANCE_STATE_ACTIVE &&
                                s_status.vision_valid &&
                                (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u;

    if(s_status.state != BALANCE_STATE_FAULT) {
        if(s_last_imu_ms != 0u && (now_ms - s_last_imu_ms) > s_config.imu_timeout_ms) {
            balance_raise_fault(BALANCE_FAULT_IMU_TIMEOUT);
        }

        if(s_motor_cmd_start_ms != 0u &&
           (now_ms - s_motor_cmd_start_ms) > s_config.motor_timeout_ms &&
           !s_status.motor_online) {
            balance_raise_fault(BALANCE_FAULT_MOTOR_TIMEOUT);
        }

        if(balance_motor_fault(s_status.motor_error)) {
            balance_raise_fault(BALANCE_FAULT_MOTOR_ERROR);
        }
    }

    if(s_status.state == BALANCE_STATE_CALIB) {
        if((now_ms - s_state_enter_ms) >= s_config.calib_settle_ms && s_status.imu_online) {
            s_pitch_sum += s_status.pitch_rad;
            s_acc_sum += s_status.acc_m_s2;
            s_calib_count++;

            if(s_calib_count >= s_config.calib_samples) {
                s_pitch_zero = s_pitch_sum / (float)s_calib_count;
                s_acc_zero = s_acc_sum / (float)s_calib_count;
                s_acc_filt = 0.0f;
                s_status.motor_cmd_rad = s_config.motor_zero_rad;
                s_return_zero_after_calib = true;
                if(s_enable_requested) {
                    balance_set_state(BALANCE_STATE_ACTIVE);
                    balance_enable_motor(now_ms);
                }
                else {
                    balance_set_state(BALANCE_STATE_READY);
                }
            }
        }
        return;
    }

    if(s_status.state == BALANCE_STATE_FAULT) {
        if(!s_fault_braked) {
            (void)bus_motor.brake(s_config.motor_id);
            s_fault_braked = true;
        }
        return;
    }

    if(!s_enable_requested) {
        balance_set_state(BALANCE_STATE_READY);
        if(s_status.motor_enabled) {
            (void)bus_motor.disable(s_config.motor_id);
        }
        return;
    }

    balance_set_state(BALANCE_STATE_ACTIVE);
    balance_enable_motor(now_ms);
    if(!s_status.motor_enabled) {
        s_motor_cmd_ready = false;
        return;
    }
    if(!s_motor_cmd_ready) {
        if(s_return_zero_after_calib) {
            s_status.motor_cmd_rad = s_config.motor_zero_rad;
            s_return_zero_after_calib = false;
            s_motor_cmd_ready = true;
            s_motor_send_tick = BALANCE_MOTOR_SEND_DIV - 1u;
            balance_send_motor(now_ms);
            return;
        }
        s_status.motor_cmd_rad = s_status.motor_pos_rad;
        s_motor_cmd_ready = true;
    }

    balance_update_sweep(now_ms);
    balance_update_control(dt_s);
    balance_send_motor(now_ms);
}

void balance_set_vision(const BalanceVisionSample* sample) {
    uint32_t arrival_ms;
    bool reset_filter;
    float dt_s;
    float pos;
    float alpha_pos;
    float alpha_vel;
    float vel_raw;

    if(!s_initialized || sample == 0) {
        return;
    }

    arrival_ms = balance_now_ms();
    s_last_vision_ms = arrival_ms;
    s_status.vision_online = true;
    s_status.ball_raw_m = sample->pos_m;
    s_status.vision_capture_ms = sample->capture_ms;
    s_status.vision_frame_seq = sample->frame_seq;

    if(!sample->valid || sample->quality < s_config.vision_quality_min ||
       !balance_value_valid(sample->pos_m)) {
        s_status.vision_valid = false;
        return;
    }

    reset_filter = s_last_valid_vision_ms == 0u ||
                   (arrival_ms - s_last_valid_vision_ms) > s_config.vision_timeout_ms;
    s_prev_vision_ms = s_last_valid_vision_ms;
    s_last_valid_vision_ms = arrival_ms;

    pos = s_config.ball_sign * sample->pos_m;
    if(!s_ball_filter_ready || reset_filter) {
        s_ball_filt = pos;
        s_ball_prev = pos;
        s_ball_vel_filt = 0.0f;
        s_ball_filter_ready = true;
        dt_s = 1.0f / 30.0f;
        pid_reset(&s_pos_pid);
    }
    else {
        if(s_prev_vision_ms != 0u && s_last_valid_vision_ms > s_prev_vision_ms) {
            dt_s = (float)(s_last_valid_vision_ms - s_prev_vision_ms) * 0.001f;
        }
        else {
            dt_s = 1.0f / 30.0f;
        }
        dt_s = balance_clamp(dt_s, 0.005f, 0.10f);
    }

    alpha_pos = dt_s / (s_config.pos_tau_s + dt_s);
    s_ball_filt += alpha_pos * (pos - s_ball_filt);
    vel_raw = (s_ball_filt - s_ball_prev) / dt_s;
    alpha_vel = dt_s / (s_config.vel_tau_s + dt_s);
    s_ball_vel_filt += alpha_vel * (vel_raw - s_ball_vel_filt);
    s_ball_prev = s_ball_filt;

    s_status.ball_pos_m = s_ball_filt;
    s_status.ball_vel_m_s = s_ball_vel_filt;
    s_status.vision_valid = true;

    if((s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u) {
        s_vis_angle = pid_calculate(&s_pos_pid, s_status.target_m, s_ball_filt, dt_s);
        s_vis_angle = balance_clamp(s_vis_angle,
                                    -s_config.vision_angle_max_rad,
                                    s_config.vision_angle_max_rad);
    }
    else {
        pid_reset(&s_pos_pid);
        s_vis_angle = 0.0f;
    }
}

void balance_set_command(const BalanceCommand* command) {
    if(!s_initialized || command == 0) {
        return;
    }

    s_enable_requested = command->enable;
    s_status.enable_requested = s_enable_requested;
    s_status.command_seq = command->command_seq;
    switch(command->mode) {
        case BALANCE_MODE_HOLD_CENTER:
        case BALANCE_MODE_HOLD_POS:
        case BALANCE_MODE_SWEEP_TEST:
            s_status.mode = command->mode;
            break;
        default:
            return;
    }

    if(s_status.mode == BALANCE_MODE_HOLD_CENTER) {
        s_status.target_m = 0.0f;
    }
    else if(s_status.mode == BALANCE_MODE_SWEEP_TEST) {
        s_sweep_phase = 0u;
        s_sweep_stable_start_ms = 0u;
        s_status.target_m = s_config.sweep_pos_m;
    }
    else {
        s_status.target_m = balance_clamp(command->target_m,
                                          -s_config.target_limit_m,
                                          s_config.target_limit_m);
    }

    pid_reset(&s_pos_pid);
    s_vis_angle = 0.0f;

    if(command->clear_fault) {
        balance_clear_fault();
    }
}

BalanceStatusCode balance_get_tune(BalanceTuneConfig* config) {
    if(!s_initialized || config == 0) {
        return BALANCE_STATUS_INVALID_PARAM;
    }

    config->motor_zero_rad = s_config.motor_zero_rad;
    config->motor_min_rad = s_config.motor_min_rad;
    config->motor_max_rad = s_config.motor_max_rad;
    config->motor_speed_rad_s = s_config.motor_speed_rad_s;
    config->motor_rate_rad_s = s_config.motor_rate_rad_s;
    config->motor_sign = s_config.motor_sign;
    config->acc_axis = s_config.acc_axis;
    config->acc_sign = s_config.acc_sign;
    config->pitch_sign = s_config.pitch_sign;
    config->ball_sign = s_config.ball_sign;
    config->level_gain = s_config.level_gain;
    config->acc_gain = s_config.acc_gain;
    config->acc_tau_s = s_config.acc_tau_s;
    config->pos_tau_s = s_config.pos_tau_s;
    config->vel_tau_s = s_config.vel_tau_s;
    config->pos_pid = s_config.pos_pid;
    config->vision_angle_max_rad = s_config.vision_angle_max_rad;
    config->vision_decay_tau_s = s_config.vision_decay_tau_s;
    config->target_limit_m = s_config.target_limit_m;
    config->vision_quality_min = s_config.vision_quality_min;
    config->vision_timeout_ms = s_config.vision_timeout_ms;
    config->sweep_pos_m = s_config.sweep_pos_m;
    config->sweep_err_m = s_config.sweep_err_m;
    config->sweep_vel_m_s = s_config.sweep_vel_m_s;
    config->sweep_stable_ms = s_config.sweep_stable_ms;
    config->ctrl_mask = s_config.ctrl_mask;
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_apply_tune(const BalanceTuneConfig* config) {
    if(!s_initialized || !balance_tune_valid(config)) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(s_status.state == BALANCE_STATE_ACTIVE && balance_mech_changed(config)) {
        return BALANCE_STATUS_STATE_DENIED;
    }

    s_config.motor_zero_rad = config->motor_zero_rad;
    s_config.motor_min_rad = config->motor_min_rad;
    s_config.motor_max_rad = config->motor_max_rad;
    s_config.motor_speed_rad_s = config->motor_speed_rad_s;
    s_config.motor_rate_rad_s = config->motor_rate_rad_s;
    s_config.motor_sign = config->motor_sign;
    s_config.acc_axis = config->acc_axis;
    s_config.acc_sign = config->acc_sign;
    s_config.pitch_sign = config->pitch_sign;
    s_config.ball_sign = config->ball_sign;
    s_config.level_gain = config->level_gain;
    s_config.acc_gain = config->acc_gain;
    s_config.acc_tau_s = config->acc_tau_s;
    s_config.pos_tau_s = config->pos_tau_s;
    s_config.vel_tau_s = config->vel_tau_s;
    s_config.pos_pid = config->pos_pid;
    s_config.vision_angle_max_rad = config->vision_angle_max_rad;
    s_config.vision_decay_tau_s = config->vision_decay_tau_s;
    s_config.target_limit_m = config->target_limit_m;
    s_config.vision_quality_min = config->vision_quality_min;
    s_config.vision_timeout_ms = config->vision_timeout_ms;
    s_config.sweep_pos_m = config->sweep_pos_m;
    s_config.sweep_err_m = config->sweep_err_m;
    s_config.sweep_vel_m_s = config->sweep_vel_m_s;
    s_config.sweep_stable_ms = config->sweep_stable_ms;
    s_config.ctrl_mask = config->ctrl_mask;
    s_status.ctrl_mask = s_config.ctrl_mask;
    s_status.target_m = balance_clamp(s_status.target_m,
                                      -s_config.target_limit_m,
                                      s_config.target_limit_m);
    s_status.motor_cmd_rad = balance_clamp(s_status.motor_cmd_rad,
                                           s_config.motor_min_rad,
                                           s_config.motor_max_rad);

    pid_init_cfg(&s_pos_pid, &s_config.pos_pid);
    balance_update_pid_mode();
    s_vis_angle = 0.0f;
    return BALANCE_STATUS_OK;
}

void balance_on_motor_feedback(const BusMotorFeedback* feedback, uint32_t stamp_ms) {
    if(feedback == 0 || feedback->id != s_config.motor_id) {
        return;
    }

    s_motor_rx_pos = feedback->position;
    s_motor_rx_spd = feedback->speed;
    s_motor_rx_tor = feedback->torque;
    s_motor_rx_error = feedback->error_code;
    s_motor_rx_ms = stamp_ms;
}

const BalanceStatus* balance_get_status(void) {
    return &s_status;
}

const char* balance_state_str(BalanceState state) {
    switch(state) {
        case BALANCE_STATE_INIT:
            return "INIT";
        case BALANCE_STATE_CALIB:
            return "CALIB";
        case BALANCE_STATE_READY:
            return "READY";
        case BALANCE_STATE_ACTIVE:
            return "ACTIVE";
        case BALANCE_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char* balance_mode_str(BalanceMode mode) {
    switch(mode) {
        case BALANCE_MODE_HOLD_CENTER:
            return "HOLD_CENTER";
        case BALANCE_MODE_HOLD_POS:
            return "HOLD_POS";
        case BALANCE_MODE_SWEEP_TEST:
            return "SWEEP_TEST";
        default:
            return "UNKNOWN";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t balance_now_ms(void) {
    return s_config.port_ops.now_ms();
}

static float balance_clamp(float value, float min, float max) {
    if(value < min) {
        return min;
    }
    if(value > max) {
        return max;
    }
    return value;
}

static float balance_abs(float value) {
    return value >= 0.0f ? value : -value;
}

static float balance_lpf(float input, float output, float tau_s, float dt_s) {
    float alpha;

    if(tau_s <= 0.0f) {
        return input;
    }

    alpha = dt_s / (tau_s + dt_s);
    return output + alpha * (input - output);
}

static float balance_get_acc_axis(const ImuAcc* acc) {
    if(acc == 0) {
        return 0.0f;
    }

    switch(s_config.acc_axis) {
        case BALANCE_AXIS_X:
            return acc->x;
        case BALANCE_AXIS_Y:
            return acc->y;
        case BALANCE_AXIS_Z:
            return acc->z;
        default:
            return 0.0f;
    }
}

static bool balance_value_valid(float value) {
    return isfinite(value) != 0;
}

static bool balance_motor_fault(uint8_t error_code) {
    return error_code >= 0x08u && error_code <= 0x0Eu;
}

static bool balance_tune_valid(const BalanceTuneConfig* config) {
    if(config == 0) {
        return false;
    }

    return balance_value_valid(config->motor_zero_rad) &&
           balance_value_valid(config->motor_min_rad) &&
           balance_value_valid(config->motor_max_rad) &&
           balance_value_valid(config->motor_speed_rad_s) &&
           balance_value_valid(config->motor_rate_rad_s) &&
           balance_value_valid(config->motor_sign) &&
           balance_value_valid(config->acc_sign) &&
           balance_value_valid(config->pitch_sign) &&
           balance_value_valid(config->ball_sign) &&
           balance_value_valid(config->level_gain) &&
           balance_value_valid(config->acc_gain) &&
           balance_value_valid(config->acc_tau_s) &&
           balance_value_valid(config->pos_tau_s) &&
           balance_value_valid(config->vel_tau_s) &&
           balance_value_valid(config->pos_pid.kp) &&
           balance_value_valid(config->pos_pid.ki) &&
           balance_value_valid(config->pos_pid.kd) &&
           balance_value_valid(config->pos_pid.max_out) &&
           balance_value_valid(config->pos_pid.integral_separation) &&
           balance_value_valid(config->pos_pid.dead_band) &&
           balance_value_valid(config->pos_pid.diff_filter_alpha) &&
           balance_value_valid(config->pos_pid.output_max_rate) &&
           balance_value_valid(config->vision_angle_max_rad) &&
           balance_value_valid(config->vision_decay_tau_s) &&
           balance_value_valid(config->target_limit_m) &&
           balance_value_valid(config->sweep_pos_m) &&
           balance_value_valid(config->sweep_err_m) &&
           balance_value_valid(config->sweep_vel_m_s) &&
           config->motor_min_rad < config->motor_max_rad &&
           config->motor_zero_rad >= config->motor_min_rad &&
           config->motor_zero_rad <= config->motor_max_rad &&
           config->motor_speed_rad_s > 0.0f &&
           config->motor_rate_rad_s > 0.0f &&
           config->motor_sign != 0.0f &&
           config->acc_sign != 0.0f &&
           config->pitch_sign != 0.0f &&
           config->ball_sign != 0.0f &&
           config->acc_axis <= BALANCE_AXIS_Z &&
           config->acc_tau_s >= 0.0f &&
           config->pos_tau_s >= 0.0f &&
           config->vel_tau_s >= 0.0f &&
           config->pos_pid.max_out > 0.0f &&
           config->pos_pid.integral_separation >= 0.0f &&
           config->pos_pid.dead_band >= 0.0f &&
           config->pos_pid.diff_filter_alpha >= 0.0f &&
           config->pos_pid.diff_filter_alpha <= 1.0f &&
           config->pos_pid.output_max_rate > 0.0f &&
           config->vision_angle_max_rad > 0.0f &&
           config->vision_decay_tau_s >= 0.0f &&
           config->target_limit_m > 0.0f &&
           config->vision_timeout_ms > 0u &&
           config->sweep_pos_m > 0.0f &&
           config->sweep_err_m > 0.0f &&
           config->sweep_vel_m_s > 0.0f &&
           config->sweep_stable_ms > 0u &&
           (config->ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) == 0u;
}

static bool balance_mech_changed(const BalanceTuneConfig* config) {
    return config->motor_zero_rad != s_config.motor_zero_rad ||
           config->motor_min_rad != s_config.motor_min_rad ||
           config->motor_max_rad != s_config.motor_max_rad ||
           config->motor_sign != s_config.motor_sign ||
           config->acc_axis != s_config.acc_axis ||
           config->acc_sign != s_config.acc_sign ||
           config->pitch_sign != s_config.pitch_sign ||
           config->ball_sign != s_config.ball_sign;
}

static void balance_update_pid_mode(void) {
    s_pos_pid.cfg.mode = (s_config.ctrl_mask & BALANCE_CTRL_VISION_I) != 0u
                             ? PID_MODE_PID
                             : PID_MODE_PD;
    s_config.pos_pid.mode = s_pos_pid.cfg.mode;
}

static void balance_set_state(BalanceState state) {
    if(s_status.state == state) {
        return;
    }

    s_status.state = state;
    s_state_enter_ms = balance_now_ms();
}

static void balance_raise_fault(BalanceFault fault) {
    s_status.fault_flags |= (uint32_t)fault;
    balance_set_state(BALANCE_STATE_FAULT);
}

static void balance_clear_fault(void) {
    s_status.fault_flags = BALANCE_FAULT_NONE;
    s_fault_braked = false;
    s_motor_write_fail_count = 0u;
    pid_reset(&s_pos_pid);
    s_vis_angle = 0.0f;
    balance_set_state(s_enable_requested ? BALANCE_STATE_ACTIVE : BALANCE_STATE_READY);
}

static void balance_update_motor_feedback(uint32_t now_ms) {
    uint32_t rx_ms = s_motor_rx_ms;

    if(rx_ms == 0u) {
        s_status.motor_online = false;
        return;
    }

    s_status.motor_pos_rad = s_motor_rx_pos;
    s_status.motor_spd_rad_s = s_motor_rx_spd;
    s_status.motor_error = s_motor_rx_error;
    s_status.motor_enabled = s_motor_rx_error == 0x01u;
    if(!s_status.motor_enabled) {
        s_motor_cmd_ready = false;
    }
    s_status.motor_online = (now_ms - rx_ms) <= s_config.motor_timeout_ms;
    (void)s_motor_rx_tor;
}

static void balance_update_imu(uint32_t now_ms, float dt_s) {
    ImuStatus imu_status;
    ImuAcc acc;
    ImuAngle angle;
    float pitch;
    float acc_axis;
    float pitch_rel;
    float acc_linear;

    imu_status = imu.update();
    if(imu_status != IMU_STATUS_OK) {
        s_status.imu_online = s_last_imu_ms != 0u &&
                              (now_ms - s_last_imu_ms) <= s_config.imu_timeout_ms;
        return;
    }

    acc = imu.get_acc();
    angle = imu.get_angle();
    pitch = s_config.pitch_sign * angle.pitch;
    acc_axis = s_config.acc_sign * balance_get_acc_axis(&acc);

    if(!balance_value_valid(pitch) || !balance_value_valid(acc_axis)) {
        balance_raise_fault(BALANCE_FAULT_DATA_INVALID);
        return;
    }

    s_last_imu_ms = now_ms;
    s_status.imu_online = true;
    s_status.pitch_rad = pitch;
    s_status.acc_m_s2 = acc_axis;

    if(s_status.state == BALANCE_STATE_CALIB) {
        return;
    }

    pitch_rel = pitch - s_pitch_zero;
    acc_linear = acc_axis - s_acc_zero + BALANCE_GRAVITY_M_S2 * sinf(pitch_rel);
    s_acc_filt = balance_lpf(acc_linear, s_acc_filt, s_config.acc_tau_s, dt_s);
    s_status.pitch_rad = pitch_rel;
    s_status.acc_m_s2 = s_acc_filt;
}

static void balance_update_online(uint32_t now_ms) {
    if(s_last_vision_ms == 0u || (now_ms - s_last_vision_ms) > s_config.vision_timeout_ms) {
        s_status.vision_online = false;
        s_status.vision_valid = false;
    }
}

static void balance_update_sweep(uint32_t now_ms) {
    bool stable;

    if(s_status.mode != BALANCE_MODE_SWEEP_TEST) {
        return;
    }

    s_status.target_m = s_sweep_phase == 0u ? s_config.sweep_pos_m : -s_config.sweep_pos_m;
    stable = s_status.vision_valid &&
             balance_abs(s_status.target_m - s_status.ball_pos_m) <= s_config.sweep_err_m &&
             balance_abs(s_status.ball_vel_m_s) <= s_config.sweep_vel_m_s;

    if(!stable) {
        s_sweep_stable_start_ms = 0u;
        return;
    }

    if(s_sweep_stable_start_ms == 0u) {
        s_sweep_stable_start_ms = now_ms;
        return;
    }

    if(s_sweep_phase == 0u && (now_ms - s_sweep_stable_start_ms) >= s_config.sweep_stable_ms) {
        s_sweep_phase = 1u;
        s_sweep_stable_start_ms = 0u;
        s_status.target_m = -s_config.sweep_pos_m;
        pid_reset(&s_pos_pid);
        s_vis_angle = 0.0f;
    }
}

static void balance_update_control(float dt_s) {
    float pitch_rel = s_status.pitch_rad;
    float target_angle;
    float max_step;

    s_status.level_angle_rad = (s_config.ctrl_mask & BALANCE_CTRL_LEVEL) != 0u
                                   ? s_config.level_gain * pitch_rel
                                   : 0.0f;
    s_status.acc_angle_rad = (s_config.ctrl_mask & BALANCE_CTRL_ACC) != 0u
                                 ? s_config.acc_gain * atan2f(s_acc_filt, BALANCE_GRAVITY_M_S2)
                                 : 0.0f;

    if(s_status.state == BALANCE_STATE_ACTIVE && s_status.vision_valid &&
       (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u) {
        s_status.vision_angle_rad = s_vis_angle;
    }
    else {
        s_vis_angle = balance_lpf(0.0f,
                                  s_vis_angle,
                                  s_config.vision_decay_tau_s,
                                  dt_s);
        s_status.vision_angle_rad = s_vis_angle;
    }

    target_angle = s_config.motor_zero_rad + s_config.motor_sign *
                                                 (s_status.level_angle_rad + s_status.acc_angle_rad + s_status.vision_angle_rad);
    target_angle = balance_clamp(target_angle, s_config.motor_min_rad, s_config.motor_max_rad);

    max_step = s_config.motor_rate_rad_s * dt_s;
    target_angle = balance_clamp(target_angle,
                                 s_status.motor_cmd_rad - max_step,
                                 s_status.motor_cmd_rad + max_step);
    s_status.motor_cmd_rad = target_angle;
}


static void balance_enable_motor(uint32_t now_ms) {
    BusMotorStatus status;

    if(s_status.motor_enabled) {
        return;
    }
    if(s_motor_enable_ms != 0u &&
       (now_ms - s_motor_enable_ms) < BALANCE_MOTOR_ENABLE_RETRY_MS) {
        return;
    }

    s_motor_enable_ms = now_ms;
    status = bus_motor.enable(s_config.motor_id);
    if(status != MOTOR_STATUS_OK) {
        if(s_motor_write_fail_count < UINT8_MAX) {
            s_motor_write_fail_count++;
        }
        if(s_motor_write_fail_count >= BALANCE_MOTOR_WRITE_FAIL_LIMIT) {
            balance_raise_fault(BALANCE_FAULT_MOTOR_WRITE);
        }
        return;
    }

    s_motor_write_fail_count = 0u;
}

static void balance_send_motor(uint32_t now_ms) {
    BusMotorStatus status;

    s_motor_send_tick = (uint8_t)((s_motor_send_tick + 1u) % BALANCE_MOTOR_SEND_DIV);
    if(s_motor_send_tick != 0u) {
        return;
    }

    status = bus_motor.set_pos_vel(s_config.motor_id,
                                   s_status.motor_cmd_rad,
                                   s_config.motor_speed_rad_s);
    if(status == MOTOR_STATUS_OK) {
        s_motor_write_fail_count = 0u;
        if(s_motor_cmd_start_ms == 0u) {
            s_motor_cmd_start_ms = now_ms;
        }
        return;
    }

    if(s_motor_write_fail_count < UINT8_MAX) {
        s_motor_write_fail_count++;
    }
    if(s_motor_write_fail_count >= BALANCE_MOTOR_WRITE_FAIL_LIMIT) {
        balance_raise_fault(BALANCE_FAULT_MOTOR_WRITE);
    }
}
