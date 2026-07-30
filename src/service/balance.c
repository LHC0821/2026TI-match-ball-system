/**
 * @file balance.c
 * @brief 车载滚球摆杆平衡控制服务实现
 */

#include "balance.h"

#include "imu/imu.h"
#include "log.h"
#include "pid.h"

#include <math.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BALANCE_GRAVITY_M_S2 9.80665f
#define BALANCE_MOTOR_WRITE_FAIL_LIMIT 5u
#define BALANCE_MOTOR_ENABLE_RETRY_MS 100u
#define BALANCE_VISION_DEFAULT_DT_S (1.0f / 30.0f)
#define BALANCE_VISION_DT_MIN_S 0.005f
#define BALANCE_VISION_DT_MAX_S 0.10f

// ! ========================= 变 量 声 明 ========================= ! //

static BalanceConfig s_config = { 0 };
static BalanceStatus s_status = { 0 };
static Pid s_vision_pid = { 0 };
static Pid s_motor_pos_i_pid = { 0 };
static bool s_initialized = false;
static bool s_enable_requested = false;
static bool s_fault_braked = false;
static uint8_t s_motor_write_fail_count = 0u;
static uint32_t s_state_enter_ms = 0u;
static uint32_t s_last_valid_vision_ms = 0u;
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
static float s_vision_prev = 0.0f;
static float s_vision_vel = 0.0f;
static float s_vis_angle = 0.0f;
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
 * @brief balance_init_vision_pid 内部辅助函数
 */
static void balance_init_vision_pid(void);
/**
 * @brief 初始化电机位置外环积分补偿
 */
static void balance_init_motor_pos_i_pid(void);
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

    if(config == 0 || config->port_ops.now_ms == 0) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(config->motor_id == 0u || config->motor_min_rad >= config->motor_max_rad ||
       config->motor_zero_rad < config->motor_min_rad ||
       config->motor_zero_rad > config->motor_max_rad ||
       config->motor_speed_rad_s < 0.0f || config->motor_sign == 0.0f ||
       config->motor_pos_i_gain < 0.0f || config->motor_pos_i_limit_rad <= 0.0f ||
       config->acc_sign == 0.0f || config->pitch_sign == 0.0f ||
       config->ball_sign == 0.0f || config->acc_tau_s < 0.0f ||
       config->vision_tau_s < 0.0f || config->calib_samples == 0u) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(config->acc_axis > BALANCE_AXIS_Z ||
       (config->ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) != 0u ||
       !balance_value_valid(config->motor_zero_rad) ||
       !balance_value_valid(config->motor_min_rad) ||
       !balance_value_valid(config->motor_max_rad) ||
       !balance_value_valid(config->motor_speed_rad_s) ||
       !balance_value_valid(config->motor_pos_i_gain) ||
       !balance_value_valid(config->motor_pos_i_limit_rad) ||
       !balance_value_valid(config->motor_sign) ||
       !balance_value_valid(config->acc_sign) ||
       !balance_value_valid(config->pitch_sign) ||
       !balance_value_valid(config->ball_sign) ||
       !balance_value_valid(config->level_gain) ||
       !balance_value_valid(config->acc_gain) ||
       !balance_value_valid(config->acc_tau_s) ||
       !balance_value_valid(config->vision_tau_s) ||
       !balance_value_valid(config->vision_kp) ||
       !balance_value_valid(config->vision_kd)) {
        return BALANCE_STATUS_INVALID_PARAM;
    }
    if(imu_instance == 0 || bus_motor_instance == 0) {
        return BALANCE_STATUS_DEPENDENCY_MISSING;
    }

    s_config = *config;
    memset(&s_status, 0, sizeof(s_status));
    balance_init_vision_pid();
    balance_init_motor_pos_i_pid();

    s_enable_requested = s_config.auto_enable;
    s_status.enable_requested = s_enable_requested;
    s_fault_braked = false;
    s_motor_write_fail_count = 0u;
    s_state_enter_ms = 0u;
    s_last_valid_vision_ms = 0u;
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
    s_vision_prev = 0.0f;
    s_vision_vel = 0.0f;
    s_vis_angle = 0.0f;
    s_vision_filter_ready = false;
    s_motor_rx_ms = 0u;
    s_motor_rx_pos = 0.0f;
    s_motor_rx_spd = 0.0f;
    s_motor_rx_error = 0u;

    s_status.motor_cmd_rad = s_config.motor_zero_rad;
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
    balance_update_motor_feedback(now_ms);
    balance_update_imu(dt_s);

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
                pid_reset(&s_motor_pos_i_pid);
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
        pid_reset(&s_motor_pos_i_pid);
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
            balance_send_motor(now_ms);
            return;
        }
        s_status.motor_cmd_rad = s_status.motor_pos_rad;
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

    if(!s_initialized || sample == 0) {
        return;
    }

    arrival_ms = balance_now_ms();
    s_status.vision_online = true;
    s_status.vision_offset_raw_m = sample->offset_m;
    s_status.vision_capture_ms = sample->capture_ms;
    s_status.vision_frame_seq = sample->frame_seq;

    if(!sample->valid || !balance_value_valid(sample->offset_m)) {
        s_status.vision_valid = false;
        s_vis_angle = 0.0f;
        return;
    }

    reset_filter = !s_vision_filter_ready;
    offset = s_config.ball_sign * sample->offset_m;

    if(reset_filter) {
        s_vision_filt = offset;
        s_vision_prev = offset;
        s_vision_vel = 0.0f;
        s_vision_filter_ready = true;
        dt_s = BALANCE_VISION_DEFAULT_DT_S;
        pid_reset(&s_vision_pid);
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
    offset_vel = (s_vision_filt - s_vision_prev) / dt_s;
    s_vision_vel = offset_vel;
    s_vision_prev = s_vision_filt;

    s_status.vision_offset_m = s_vision_filt;
    s_status.vision_offset_vel_m_s = s_vision_vel;
    s_status.vision_valid = true;
    if((s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u) {
        s_vis_angle = pid_calculate(&s_vision_pid, 0.0f, s_vision_filt, dt_s);
    }
    else {
        pid_reset(&s_vision_pid);
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
    config->level_gain = s_config.level_gain;
    config->acc_gain = s_config.acc_gain;
    config->acc_tau_s = s_config.acc_tau_s;
    config->vision_tau_s = s_config.vision_tau_s;
    config->vision_kp = s_config.vision_kp;
    config->vision_kd = s_config.vision_kd;
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_apply_tune(const BalanceTuneConfig* config) {
    if(!s_initialized || !balance_tune_valid(config)) {
        return BALANCE_STATUS_INVALID_PARAM;
    }

    s_config.motor_zero_rad = config->motor_zero_rad;
    s_config.motor_min_rad = config->motor_min_rad;
    s_config.motor_max_rad = config->motor_max_rad;
    s_config.motor_speed_rad_s = config->motor_speed_rad_s;
    s_config.level_gain = config->level_gain;
    s_config.acc_gain = config->acc_gain;
    s_config.acc_tau_s = config->acc_tau_s;
    s_config.vision_tau_s = config->vision_tau_s;
    s_config.vision_kp = config->vision_kp;
    s_config.vision_kd = config->vision_kd;
    s_status.motor_cmd_rad = balance_clamp(s_status.motor_cmd_rad,
                                           s_config.motor_min_rad,
                                           s_config.motor_max_rad);
    balance_init_vision_pid();
    s_vis_angle = 0.0f;
    return BALANCE_STATUS_OK;
}

BalanceStatusCode balance_set_ctrl_mask(uint8_t ctrl_mask) {
    if(!s_initialized || (ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) != 0u) {
        return BALANCE_STATUS_INVALID_PARAM;
    }

    s_config.ctrl_mask = ctrl_mask;
    s_status.ctrl_mask = ctrl_mask;
    if((ctrl_mask & BALANCE_CTRL_VISION) == 0u) {
        pid_reset(&s_vision_pid);
        s_vis_angle = 0.0f;
    }

    log_info("TUNE CTRL mask=0x%02X level=%u acc=%u vision=%u",
             ctrl_mask,
             (ctrl_mask & BALANCE_CTRL_LEVEL) != 0u ? 1u : 0u,
             (ctrl_mask & BALANCE_CTRL_ACC) != 0u ? 1u : 0u,
             (ctrl_mask & BALANCE_CTRL_VISION) != 0u ? 1u : 0u);
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
           balance_value_valid(config->motor_speed_rad_s) &&
           balance_value_valid(config->level_gain) &&
           balance_value_valid(config->acc_gain) &&
           balance_value_valid(config->acc_tau_s) &&
           balance_value_valid(config->vision_tau_s) &&
           balance_value_valid(config->vision_kp) &&
           balance_value_valid(config->vision_kd) &&
           config->motor_min_rad < config->motor_max_rad &&
           config->motor_zero_rad >= config->motor_min_rad &&
           config->motor_zero_rad <= config->motor_max_rad &&
           config->motor_speed_rad_s >= 0.0f &&
           config->acc_tau_s >= 0.0f &&
           config->vision_tau_s >= 0.0f;
}

static void balance_init_vision_pid(void) {
    PidConfig config = { 0 };

    config.mode = PID_MODE_PD;
    config.features = PID_FEAT_DIFF_ON_MEAS;
    config.kp = s_config.vision_kp;
    config.ki = 0.0f;
    config.kd = s_config.vision_kd;
    pid_init_cfg(&s_vision_pid, &config);
}

/**
 * @brief 初始化电机位置外环积分补偿
 */
static void balance_init_motor_pos_i_pid(void) {
    PidConfig config = { 0 };

    config.mode = PID_MODE_I;
    config.features = PID_FEAT_OUTPUT_LIMIT | PID_FEAT_ANTI_WINDUP;
    config.ki = s_config.motor_pos_i_gain;
    config.max_out = s_config.motor_pos_i_limit_rad;
    pid_init_cfg(&s_motor_pos_i_pid, &config);
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
    balance_init_vision_pid();
    pid_reset(&s_motor_pos_i_pid);
    s_vis_angle = 0.0f;
    balance_set_state(s_enable_requested ? BALANCE_STATE_ACTIVE : BALANCE_STATE_READY);
}

static void balance_update_motor_feedback(uint32_t now_ms) {
    (void)now_ms;

    if(s_motor_rx_ms == 0u) {
        s_status.motor_online = false;
        pid_reset(&s_motor_pos_i_pid);
        return;
    }

    s_status.motor_pos_rad = s_motor_rx_pos;
    s_status.motor_spd_rad_s = s_motor_rx_spd;
    s_status.motor_error = s_motor_rx_error;
    s_status.motor_enabled = s_motor_rx_error == 0x01u;
    if(!s_status.motor_enabled) {
        s_motor_cmd_ready = false;
        pid_reset(&s_motor_pos_i_pid);
    }
    s_status.motor_online = true;
}

static void balance_update_imu(float dt_s) {
    ImuStatus imu_status;
    ImuAcc acc;
    ImuAngle angle;
    float pitch;
    float acc_axis;
    float pitch_rel;
    float acc_linear;

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

static void balance_update_control(float dt_s) {
    float pitch_rel = s_status.pitch_rad;
    float level_angle;
    float acc_angle;
    float vision_angle;
    float target_angle;

    (void)dt_s;
    level_angle = (s_config.ctrl_mask & BALANCE_CTRL_LEVEL) != 0u
                      ? s_config.level_gain * pitch_rel
                      : 0.0f;
    acc_angle = (s_config.ctrl_mask & BALANCE_CTRL_ACC) != 0u
                    ? s_config.acc_gain * atan2f(s_acc_filt, BALANCE_GRAVITY_M_S2)
                    : 0.0f;
    vision_angle = (s_status.vision_valid &&
                    (s_config.ctrl_mask & BALANCE_CTRL_VISION) != 0u)
                       ? s_vis_angle
                       : 0.0f;

    s_status.level_angle_rad = level_angle;
    s_status.acc_angle_rad = acc_angle;
    s_status.vision_angle_rad = vision_angle;

    target_angle = s_config.motor_zero_rad + s_config.motor_sign *
                   (level_angle + acc_angle + vision_angle);
    if(s_status.motor_online && s_status.motor_enabled &&
       s_config.motor_pos_i_gain > 0.0f) {
        target_angle += pid_calculate(&s_motor_pos_i_pid,
                                      target_angle,
                                      s_status.motor_pos_rad,
                                      dt_s);
    }
    else {
        pid_reset(&s_motor_pos_i_pid);
    }
    s_status.motor_cmd_rad = balance_clamp(target_angle,
                                           s_config.motor_min_rad,
                                           s_config.motor_max_rad);
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

    (void)now_ms;
    status = bus_motor.set_pos_vel(s_config.motor_id,
                                   s_status.motor_cmd_rad,
                                   s_config.motor_speed_rad_s);
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
