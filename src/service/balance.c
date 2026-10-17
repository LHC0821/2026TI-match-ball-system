/**
 * @file balance.c
 * @brief 车载滚球摆杆平衡控制服务实现
 */

#include "balance.h"

#include "imu/imu.h"
#include "log.h"

#include <math.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BALANCE_GRAVITY_M_S2 9.80665f
#define BALANCE_MOTOR_WRITE_FAIL_LIMIT 5u
#define BALANCE_MOTOR_ENABLE_RETRY_MS 100u
#define BALANCE_VISION_DEFAULT_DT_S (1.0f / 30.0f)
#define BALANCE_VISION_DT_MIN_S 0.005f
#define BALANCE_VISION_DT_MAX_S 0.10f
#define BALANCE_VISION_TIMEOUT_MS 300u

// ! ========================= 变 量 声 明 ========================= ! //

static BalanceConfig s_config = { 0 };
static BalanceStatus s_status = { 0 };
static bool s_initialized = false;
static bool s_motor_prepared = false;
static bool s_enable_requested = false;
static bool s_fault_braked = false;
static uint8_t s_motor_write_fail_count = 0u;
static uint32_t s_state_enter_ms = 0u;
static uint32_t s_last_valid_vision_ms = 0u;
static uint32_t s_last_vision_ms = 0u;
static uint32_t s_motor_enable_ms = 0u;
static bool s_motor_cmd_ready = false;
static bool s_return_zero_after_calib = false;
static uint16_t s_calib_count = 0u;
static float s_pitch_sum = 0.0f;
static float s_acc_sum = 0.0f;
static float s_pitch_zero = 0.0f;
static float s_acc_zero = 0.0f;
static float s_acc_filt = 0.0f;
static float s_vision_filt = 0.0f;
static float s_vision_integral = 0.0f;
static float s_vis_angle = 0.0f;
static bool s_acc_fast_mode = false;
static bool s_vision_filter_ready = false;

static volatile uint32_t s_motor_rx_ms = 0u;
static volatile float s_motor_rx_pos = 0.0f;
static volatile float s_motor_rx_spd = 0.0f;
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
 * @brief balance_lpf 内部辅助函数
 * @param input 输入值
 * @param output 上一次输出
 * @param tau_s 时间常数，单位 s
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
 * @brief balance_reset_vision_control 内部辅助函数
 */
static void balance_reset_vision_control(void);
/**
 * @brief balance_set_state 内部辅助函数
 * @param state 状态值
 */
static void balance_set_state(BalanceState state);
/**
 * @brief balance_raise_fault 内部辅助函数
 * @param fault 故障值
 */
static void balance_raise_fault(BalanceFault fault);
/**
 * @brief balance_clear_fault 内部辅助函数
 */
static void balance_clear_fault(void);
/**
 * @brief balance_update_motor_feedback 内部辅助函数
 * @param now_ms 当前时间，单位 ms
 */
static void balance_update_motor_feedback(uint32_t now_ms);
/**
 * @brief 更新视觉通信和有效数据超时状态
 * @param now_ms 当前时间，单位 ms
 */
static void balance_update_vision_timeout(uint32_t now_ms);
/**
 * @brief balance_update_imu 内部辅助函数
 * @param dt_s 周期时间，单位 s
 */
static void balance_update_imu(float dt_s);
/**
 * @brief balance_update_control 内部辅助函数
 * @param dt_s 周期时间，单位 s
 */
static void balance_update_control(float dt_s);
/**
 * @brief balance_enable_motor 内部辅助函数
 * @param now_ms 当前时间，单位 ms
 */
static void balance_enable_motor(uint32_t now_ms);
/**
 * @brief balance_send_motor 内部辅助函数
 * @param now_ms 当前时间，单位 ms
 */
static void balance_send_motor(uint32_t now_ms);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BalanceStatusCode balance_init(const BalanceConfig* config) {
    s_initialized = false;

    if(config == 0 ||
       config->port_ops.now_ms == 0 ||
       config->port_ops.start_signal_active == 0 ||
       config->port_ops.prepare_motor == 0) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(config->motor_id == 0u || config->motor_min_rad >= config->motor_max_rad ||
       config->motor_zero_rad < config->motor_min_rad ||
       config->motor_zero_rad > config->motor_max_rad ||
       fabsf(config->motor_vel_rad_s) > BALANCE_MOTOR_VEL_MAX_RAD_S ||
       config->motor_kp < 0.0f || config->motor_kp > BALANCE_MOTOR_KP_MAX ||
       config->motor_kd < 0.0f || config->motor_kd > BALANCE_MOTOR_KD_MAX ||
       fabsf(config->motor_tor_nm) > BALANCE_MOTOR_TOR_MAX_NM ||
       config->motor_sign == 0.0f ||
       config->acc_sign == 0.0f || config->pitch_sign == 0.0f ||
       config->ball_sign == 0.0f || config->acc_tau_s < 0.0f ||
       config->acc_fast_tau_s < 0.0f ||
       config->acc_fast_enter_m_s2 < 0.0f ||
       config->acc_fast_exit_m_s2 < 0.0f ||
       config->acc_fast_exit_m_s2 > config->acc_fast_enter_m_s2 ||
       config->vision_tau_s < 0.0f ||
       config->vision_max_angle_rad < 0.0f ||
       config->calib_samples == 0u) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(config->acc_axis > BALANCE_AXIS_Z ||
       (config->ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) != 0u ||
       !balance_value_valid(config->motor_zero_rad) ||
       !balance_value_valid(config->motor_min_rad) ||
       !balance_value_valid(config->motor_max_rad) ||
       !balance_value_valid(config->motor_vel_rad_s) ||
       !balance_value_valid(config->motor_kp) ||
       !balance_value_valid(config->motor_kd) ||
       !balance_value_valid(config->motor_tor_nm) ||
       !balance_value_valid(config->tor_pitch_gain) ||
       !balance_value_valid(config->motor_sign) ||
       !balance_value_valid(config->acc_sign) ||
       !balance_value_valid(config->pitch_sign) ||
       !balance_value_valid(config->ball_sign) ||
       !balance_value_valid(config->level_gain) ||
       !balance_value_valid(config->acc_gain) ||
       !balance_value_valid(config->start_ff_angle_rad) ||
       !balance_value_valid(config->acc_tau_s) ||
       !balance_value_valid(config->acc_fast_tau_s) ||
       !balance_value_valid(config->acc_fast_enter_m_s2) ||
       !balance_value_valid(config->acc_fast_exit_m_s2) ||
       !balance_value_valid(config->vision_tau_s) ||
       !balance_value_valid(config->vision_kp) ||
       !balance_value_valid(config->vision_ki) ||
       !balance_value_valid(config->vision_kd) ||
       !balance_value_valid(config->vision_kff) ||
       !balance_value_valid(config->vision_max_angle_rad)) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(imu_instance == 0 || bus_motor_instance == 0) {
        return BALANCE_STATUS_DEPENDENCY_MISSING;
    }

    s_config = *config;
    memset(&s_status, 0, sizeof(s_status));
    balance_reset_vision_control();

    s_enable_requested = s_config.auto_enable;
    s_motor_prepared = false;
    s_status.enable_requested = s_enable_requested;
    s_fault_braked = false;
    s_motor_write_fail_count = 0u;
    s_state_enter_ms = 0u;
    s_last_valid_vision_ms = 0u;
    s_last_vision_ms = 0u;
    s_motor_enable_ms = 0u;
    s_motor_cmd_ready = false;
    s_return_zero_after_calib = false;
    s_calib_count = 0u;
    s_pitch_sum = 0.0f;
    s_acc_sum = 0.0f;
    s_pitch_zero = 0.0f;
    s_acc_zero = 0.0f;
    s_acc_filt = 0.0f;
    s_vision_filt = 0.0f;
    s_vision_integral = 0.0f;
    s_vis_angle = 0.0f;
    s_acc_fast_mode = false;
    s_vision_filter_ready = false;
    s_motor_rx_ms = 0u;
    s_motor_rx_pos = 0.0f;
    s_motor_rx_spd = 0.0f;
    s_motor_rx_error = 0u;

    s_status.motor_cmd_rad = s_config.motor_zero_rad;
    s_status.motor_cmd_raw_rad = s_config.motor_zero_rad;
    s_status.acc_tau_s = s_config.acc_tau_s;
    s_status.vision_limit_angle_rad = s_config.vision_max_angle_rad;
    s_status.ctrl_mask = s_config.ctrl_mask;
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
    s_status.start_signal_active = s_config.port_ops.start_signal_active();
    balance_update_motor_feedback(now_ms);
    balance_update_imu(dt_s);
    balance_update_vision_timeout(now_ms);

    s_status.ball_loop_closed = s_status.state == BALANCE_STATE_ACTIVE &&
                                s_status.vision_valid &&
                                (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u;

    if(s_status.state != BALANCE_STATE_FAULT &&
       balance_motor_fault(s_status.motor_error)) {
        balance_raise_fault(BALANCE_FAULT_MOTOR_ERROR);
    }

    if(s_status.state == BALANCE_STATE_CALIB) {
        if((now_ms - s_state_enter_ms) >= s_config.calib_settle_ms &&
           s_status.imu_online) {
            s_pitch_sum += s_status.pitch_rad;
            s_acc_sum += s_status.acc_m_s2;
            s_calib_count++;

            if(s_calib_count >= s_config.calib_samples) {
                s_pitch_zero = s_pitch_sum / (float)s_calib_count;
                s_acc_zero = s_acc_sum / (float)s_calib_count;
                s_acc_filt = 0.0f;
                s_acc_fast_mode = false;
                s_status.acc_fast_filter = false;
                s_status.acc_tau_s = s_config.acc_tau_s;
                s_status.motor_cmd_rad = s_config.motor_zero_rad;
                s_status.motor_cmd_raw_rad = s_config.motor_zero_rad;
                s_return_zero_after_calib = true;
                if(!s_config.port_ops.prepare_motor(s_config.motor_id)) {
                    balance_raise_fault(BALANCE_FAULT_MOTOR_WRITE);
                    return;
                }
                s_motor_prepared = true;
                if(s_enable_requested) {
                    balance_set_state(BALANCE_STATE_ACTIVE);
                    balance_enable_motor(balance_now_ms());
                }
                else {
                    balance_set_state(BALANCE_STATE_READY);
                }
            }
        }
        return;
    }

    if(s_status.state == BALANCE_STATE_FAULT) {
        if(s_motor_prepared && !s_fault_braked) {
            (void)bus_motor.brake(s_config.motor_id);
            s_fault_braked = true;
        }
        return;
    }

    if(!s_enable_requested) {
        balance_set_state(BALANCE_STATE_READY);
        balance_enable_motor(now_ms);
        if(!s_status.motor_enabled) {
            return;
        }
        if(!s_motor_cmd_ready) {
            s_status.motor_cmd_rad = s_status.motor_pos_rad;
            s_status.motor_cmd_raw_rad = s_status.motor_pos_rad;
            s_motor_cmd_ready = true;
        }
        balance_send_motor(now_ms);
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
            s_status.motor_cmd_raw_rad = s_config.motor_zero_rad;
            s_return_zero_after_calib = false;
            s_motor_cmd_ready = true;
            balance_send_motor(now_ms);
            return;
        }
        s_status.motor_cmd_rad = s_status.motor_pos_rad;
        s_status.motor_cmd_raw_rad = s_status.motor_pos_rad;
        s_motor_cmd_ready = true;
    }

    balance_update_control(dt_s);
    balance_send_motor(now_ms);
}

void balance_set_vision(const BalanceVisionSample* sample) {
    uint32_t arrival_ms;
    bool reset_filter;
    float dt_s;
    float offset;
    float alpha_pos;
    float offset_vel;
    float vision_ff;
    float vision_error;
    float integral_direction;
    float integral_candidate;
    float vision_raw;
    float vision_angle;
    float motor_raw;
    bool allow_integral;

    if(!s_initialized || sample == 0) {
        return;
    }

    arrival_ms = balance_now_ms();
    s_last_vision_ms = arrival_ms;
    s_status.vision_online = true;
    s_status.vision_offset_raw_px = sample->offset_px;
    s_status.vision_capture_ms = sample->capture_ms;
    s_status.vision_frame_seq = sample->frame_seq;

    if(!sample->valid ||
       !balance_value_valid(sample->offset_px) ||
       !balance_value_valid(sample->velocity_px_s)) {
        s_status.vision_valid = false;
        s_status.vision_offset_px = 0.0f;
        s_status.vision_offset_vel_px_s = 0.0f;
        s_vision_filter_ready = false;
        balance_reset_vision_control();
        return;
    }

    reset_filter = !s_vision_filter_ready;
    offset = s_config.ball_sign * sample->offset_px;

    if(reset_filter) {
        s_vision_filt = offset;
        s_vision_filter_ready = true;
        dt_s = BALANCE_VISION_DEFAULT_DT_S;
        balance_reset_vision_control();
    }
    else {
        if(s_last_valid_vision_ms != 0u && arrival_ms > s_last_valid_vision_ms) {
            dt_s = (float)(arrival_ms - s_last_valid_vision_ms) * 0.001f;
        }
        else {
            dt_s = BALANCE_VISION_DEFAULT_DT_S;
        }
        dt_s = balance_clamp(dt_s,
                             BALANCE_VISION_DT_MIN_S,
                             BALANCE_VISION_DT_MAX_S);
    }

    s_last_valid_vision_ms = arrival_ms;
    alpha_pos = dt_s / (s_config.vision_tau_s + dt_s);
    s_vision_filt += alpha_pos * (offset - s_vision_filt);
    offset_vel = s_config.ball_sign * sample->velocity_px_s;

    s_status.vision_offset_px = s_vision_filt;
    s_status.vision_offset_vel_px_s = offset_vel;
    s_status.vision_valid = true;
    if(s_enable_requested &&
       (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u) {
        vision_error = -s_vision_filt;
        s_status.vision_p_rad = s_config.vision_kp * vision_error;
        s_status.vision_d_rad = -s_config.vision_kd * offset_vel;
        vision_ff = -s_config.vision_kff * offset_vel;
        allow_integral = true;
        integral_candidate = s_vision_integral + vision_error * dt_s;
        integral_direction = s_config.vision_ki * vision_error;
        vision_raw = s_status.vision_p_rad +
                     s_config.vision_ki * integral_candidate +
                     s_status.vision_d_rad +
                     vision_ff;
        vision_angle = balance_clamp(vision_raw,
                                     -s_config.vision_max_angle_rad,
                                     s_config.vision_max_angle_rad);
        if((vision_raw > vision_angle && integral_direction > 0.0f) ||
           (vision_raw < vision_angle && integral_direction < 0.0f)) {
            allow_integral = false;
        }

        motor_raw = s_config.motor_zero_rad +
                    s_config.motor_sign *
                        (s_status.level_angle_rad +
                         s_status.acc_angle_rad +
                         vision_angle);
        integral_direction *= s_config.motor_sign;
        if((motor_raw > s_config.motor_max_rad && integral_direction > 0.0f) ||
           (motor_raw < s_config.motor_min_rad && integral_direction < 0.0f)) {
            allow_integral = false;
        }

        if(allow_integral) {
            s_vision_integral = integral_candidate;
        }
        s_status.vision_i_rad = s_config.vision_ki * s_vision_integral;
        vision_raw = s_status.vision_p_rad +
                     s_status.vision_i_rad +
                     s_status.vision_d_rad +
                     vision_ff;
        s_vis_angle = balance_clamp(vision_raw,
                                    -s_config.vision_max_angle_rad,
                                    s_config.vision_max_angle_rad);
        s_status.vision_raw_angle_rad = vision_raw;
        s_status.vision_limit_angle_rad = s_config.vision_max_angle_rad;
        s_status.vision_angle_saturated = s_vis_angle != vision_raw;
    }
    else {
        balance_reset_vision_control();
    }
}

void balance_set_command(const BalanceCommand* command) {
    if(!s_initialized || command == 0) {
        return;
    }

    s_enable_requested = command->enable;
    s_status.enable_requested = s_enable_requested;
    s_status.command_seq = command->command_seq;
    if(!s_enable_requested) {
        balance_reset_vision_control();
    }

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
    config->motor_vel_rad_s = s_config.motor_vel_rad_s;
    config->motor_kp = s_config.motor_kp;
    config->motor_kd = s_config.motor_kd;
    config->motor_tor_nm = s_config.motor_tor_nm;
    config->tor_pitch_gain = s_config.tor_pitch_gain;
    config->level_gain = s_config.level_gain;
    config->acc_gain = s_config.acc_gain;
    config->start_ff_angle_rad = s_config.start_ff_angle_rad;
    config->acc_tau_s = s_config.acc_tau_s;
    config->acc_fast_tau_s = s_config.acc_fast_tau_s;
    config->acc_fast_enter_m_s2 = s_config.acc_fast_enter_m_s2;
    config->acc_fast_exit_m_s2 = s_config.acc_fast_exit_m_s2;
    config->vision_tau_s = s_config.vision_tau_s;
    config->vision_kp = s_config.vision_kp;
    config->vision_ki = s_config.vision_ki;
    config->vision_kd = s_config.vision_kd;
    config->vision_kff = s_config.vision_kff;
    config->vision_max_angle_rad = s_config.vision_max_angle_rad;
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_apply_tune(const BalanceTuneConfig* config) {
    if(!s_initialized || !balance_tune_valid(config)) {
        return BALANCE_STATUS_INVALID_PARAM;
    }

    s_config.motor_zero_rad = config->motor_zero_rad;
    s_config.motor_min_rad = config->motor_min_rad;
    s_config.motor_max_rad = config->motor_max_rad;
    s_config.motor_vel_rad_s = config->motor_vel_rad_s;
    s_config.motor_kp = config->motor_kp;
    s_config.motor_kd = config->motor_kd;
    s_config.motor_tor_nm = config->motor_tor_nm;
    s_config.tor_pitch_gain = config->tor_pitch_gain;
    s_config.level_gain = config->level_gain;
    s_config.acc_gain = config->acc_gain;
    s_config.start_ff_angle_rad = config->start_ff_angle_rad;
    s_config.acc_tau_s = config->acc_tau_s;
    s_config.acc_fast_tau_s = config->acc_fast_tau_s;
    s_config.acc_fast_enter_m_s2 = config->acc_fast_enter_m_s2;
    s_config.acc_fast_exit_m_s2 = config->acc_fast_exit_m_s2;
    s_config.vision_tau_s = config->vision_tau_s;
    s_config.vision_kp = config->vision_kp;
    s_config.vision_ki = config->vision_ki;
    s_config.vision_kd = config->vision_kd;
    s_config.vision_kff = config->vision_kff;
    s_config.vision_max_angle_rad = config->vision_max_angle_rad;
    s_status.motor_cmd_rad = balance_clamp(s_status.motor_cmd_rad,
                                           s_config.motor_min_rad,
                                           s_config.motor_max_rad);
    balance_reset_vision_control();
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_set_ctrl_mask(uint8_t ctrl_mask) {
    if(!s_initialized || (ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) != 0u) {
        return BALANCE_STATUS_INVALID_PARAM;
    }

    s_config.ctrl_mask = ctrl_mask;
    s_status.ctrl_mask = ctrl_mask;
    if((ctrl_mask & BALANCE_CTRL_VISION) == 0u) {
        balance_reset_vision_control();
    }

    log_info("TUNE CTRL mask=0x%02X level=%u acc=%u vision=%u",
             ctrl_mask,
             (ctrl_mask & BALANCE_CTRL_LEVEL) != 0u ? 1u : 0u,
             (ctrl_mask & BALANCE_CTRL_ACC) != 0u ? 1u : 0u,
             (ctrl_mask & BALANCE_CTRL_VISION) != 0u ? 1u : 0u);
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_set_motor_angle(float angle_rad) {
    if(!s_initialized || !balance_value_valid(angle_rad) ||
       angle_rad < s_config.motor_min_rad ||
       angle_rad > s_config.motor_max_rad) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(s_enable_requested || !s_motor_prepared ||
       s_status.state == BALANCE_STATE_CALIB ||
       s_status.state == BALANCE_STATE_FAULT) {
        return BALANCE_STATUS_STATE_DENIED;
    }

    s_status.motor_cmd_rad = angle_rad;
    s_status.motor_cmd_raw_rad = angle_rad;
    s_status.motor_cmd_saturated = false;
    s_motor_cmd_ready = true;
    return BALANCE_STATUS_OK;
}

void balance_on_motor_feedback(const BusMotorFeedback* feedback, uint32_t stamp_ms) {
    if(feedback == 0 || feedback->id != s_config.motor_id) {
        return;
    }

    s_motor_rx_pos = feedback->position;
    s_motor_rx_spd = feedback->speed;
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
           balance_value_valid(config->motor_vel_rad_s) &&
           balance_value_valid(config->motor_kp) &&
           balance_value_valid(config->motor_kd) &&
           balance_value_valid(config->motor_tor_nm) &&
           balance_value_valid(config->tor_pitch_gain) &&
           balance_value_valid(config->level_gain) &&
           balance_value_valid(config->acc_gain) &&
           balance_value_valid(config->start_ff_angle_rad) &&
           balance_value_valid(config->acc_tau_s) &&
           balance_value_valid(config->acc_fast_tau_s) &&
           balance_value_valid(config->acc_fast_enter_m_s2) &&
           balance_value_valid(config->acc_fast_exit_m_s2) &&
           balance_value_valid(config->vision_tau_s) &&
           balance_value_valid(config->vision_kp) &&
           balance_value_valid(config->vision_ki) &&
           balance_value_valid(config->vision_kd) &&
           balance_value_valid(config->vision_kff) &&
           balance_value_valid(config->vision_max_angle_rad) &&
           config->motor_min_rad < config->motor_max_rad &&
           config->motor_zero_rad >= config->motor_min_rad &&
           config->motor_zero_rad <= config->motor_max_rad &&
           fabsf(config->motor_vel_rad_s) <= BALANCE_MOTOR_VEL_MAX_RAD_S &&
           config->motor_kp >= 0.0f &&
           config->motor_kp <= BALANCE_MOTOR_KP_MAX &&
           config->motor_kd >= 0.0f &&
           config->motor_kd <= BALANCE_MOTOR_KD_MAX &&
           fabsf(config->motor_tor_nm) <= BALANCE_MOTOR_TOR_MAX_NM &&
           config->acc_tau_s >= 0.0f &&
           config->acc_fast_tau_s >= 0.0f &&
           config->acc_fast_enter_m_s2 >= 0.0f &&
           config->acc_fast_exit_m_s2 >= 0.0f &&
           config->acc_fast_exit_m_s2 <= config->acc_fast_enter_m_s2 &&
           config->vision_tau_s >= 0.0f &&
           config->vision_max_angle_rad >= 0.0f;
}

static void balance_reset_vision_control(void) {
    s_vision_integral = 0.0f;
    s_status.vision_raw_angle_rad = 0.0f;
    s_status.vision_p_rad = 0.0f;
    s_status.vision_i_rad = 0.0f;
    s_status.vision_d_rad = 0.0f;
    s_status.vision_angle_rad = 0.0f;
    s_status.vision_limit_angle_rad = s_config.vision_max_angle_rad;
    s_status.vision_angle_saturated = false;
    s_vis_angle = 0.0f;
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
    balance_reset_vision_control();
    balance_set_state(s_enable_requested ? BALANCE_STATE_ACTIVE : BALANCE_STATE_READY);
}

static void balance_update_motor_feedback(uint32_t now_ms) {
    (void)now_ms;

    if(s_motor_rx_ms == 0u) {
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
    s_status.motor_online = true;
}

static void balance_update_vision_timeout(uint32_t now_ms) {
    if(s_last_vision_ms == 0u ||
       (now_ms - s_last_vision_ms) > BALANCE_VISION_TIMEOUT_MS) {
        s_status.vision_online = false;
    }

    if(s_last_valid_vision_ms == 0u ||
       (now_ms - s_last_valid_vision_ms) > BALANCE_VISION_TIMEOUT_MS) {
        s_status.vision_valid = false;
        s_status.vision_offset_px = 0.0f;
        s_status.vision_offset_vel_px_s = 0.0f;
        s_vision_filter_ready = false;
        balance_reset_vision_control();
    }
}

static void balance_update_imu(float dt_s) {
    ImuStatus imu_status;
    ImuAcc acc;
    ImuAngle angle;
    float pitch;
    float acc_axis;
    float pitch_rel;
    float acc_input;
    float acc_error;
    float acc_tau_s;

    imu_status = imu.update();
    if(imu_status != IMU_STATUS_OK) {
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

    s_status.imu_online = true;
    s_status.pitch_raw_rad = pitch;
    s_status.acc_raw_m_s2 = acc_axis;
    s_status.pitch_rad = pitch;
    s_status.acc_m_s2 = acc_axis;

    if(s_status.state == BALANCE_STATE_CALIB) {
        return;
    }

    pitch_rel = pitch - s_pitch_zero;
    acc_input = acc_axis - s_acc_zero;
    acc_error = acc_input - s_acc_filt;
    if(!s_acc_fast_mode &&
       fabsf(acc_error) > s_config.acc_fast_enter_m_s2) {
        s_acc_fast_mode = true;
    }
    else if(s_acc_fast_mode &&
            fabsf(acc_error) < s_config.acc_fast_exit_m_s2) {
        s_acc_fast_mode = false;
    }

    acc_tau_s = s_acc_fast_mode
                    ? s_config.acc_fast_tau_s
                    : s_config.acc_tau_s;
    s_acc_filt = balance_lpf(acc_input, s_acc_filt, acc_tau_s, dt_s);
    s_status.pitch_rad = pitch_rel;
    s_status.acc_input_m_s2 = acc_input;
    s_status.acc_m_s2 = s_acc_filt;
    s_status.acc_error_m_s2 = acc_error;
    s_status.acc_tau_s = acc_tau_s;
    s_status.acc_fast_filter = s_acc_fast_mode;
}

static void balance_update_control(float dt_s) {
    float pitch_rel = s_status.pitch_rad;
    float level_angle;
    float acc_angle;
    float start_ff_angle;
    float vision_angle;
    float target_angle;

    (void)dt_s;
    level_angle = (s_config.ctrl_mask & BALANCE_CTRL_LEVEL) != 0u
                      ? s_config.level_gain * pitch_rel
                      : 0.0f;
    acc_angle = (s_config.ctrl_mask & BALANCE_CTRL_ACC) != 0u
                    ? s_config.acc_gain * atan2f(s_acc_filt, BALANCE_GRAVITY_M_S2)
                    : 0.0f;
    start_ff_angle = s_status.start_signal_active
                         ? s_config.start_ff_angle_rad
                         : 0.0f;
    vision_angle = (s_status.vision_valid &&
                    (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u)
                       ? s_vis_angle
                       : 0.0f;

    s_status.level_angle_rad = level_angle;
    s_status.acc_angle_rad = acc_angle;
    s_status.vision_angle_rad = vision_angle;

    target_angle = s_config.motor_zero_rad + s_config.motor_sign *
                   (level_angle + acc_angle + vision_angle + start_ff_angle);
    s_status.motor_cmd_raw_rad = target_angle;
    s_status.motor_cmd_rad = balance_clamp(target_angle,
                                           s_config.motor_min_rad,
                                           s_config.motor_max_rad);
    s_status.motor_cmd_saturated = s_status.motor_cmd_rad != target_angle;
}

static void balance_enable_motor(uint32_t now_ms) {
    BusMotorStatus status;

    if(!s_motor_prepared || s_status.motor_enabled) {
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
    float tor_ff;

    (void)now_ms;
    if(!s_motor_prepared) {
        return;
    }
    tor_ff = s_config.motor_tor_nm +
             s_config.tor_pitch_gain * s_status.pitch_rad;
    tor_ff = balance_clamp(tor_ff,
                           -BALANCE_MOTOR_TOR_MAX_NM,
                           BALANCE_MOTOR_TOR_MAX_NM);
    s_status.motor_tor_ff_nm = tor_ff;
    status = bus_motor.set_mit(s_config.motor_id,
                               s_status.motor_cmd_rad,
                               s_config.motor_vel_rad_s,
                               s_config.motor_kp,
                               s_config.motor_kd,
                               tor_ff);
    if(status == MOTOR_STATUS_OK) {
        s_motor_write_fail_count = 0u;
        return;
    }

    if(s_motor_write_fail_count < UINT8_MAX) {
        s_motor_write_fail_count++;
    }
    if(s_motor_write_fail_count >= BALANCE_MOTOR_WRITE_FAIL_LIMIT) {
        balance_raise_fault(BALANCE_FAULT_MOTOR_WRITE);
    }
}
