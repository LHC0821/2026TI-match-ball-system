/**
 * @file app_tune.c
 * @brief USART1 无线调参应用实现
 */

#include "app_tune.h"

#include "app_config.h"
#include "balance.h"
#include "delay.h"
#include "stm32_hal_uart.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define APP_TUNE_ARG_MAX 4u
#define APP_TUNE_RESULT_NONE 0u
#define APP_TUNE_RESULT_OK 1u
#define APP_TUNE_RESULT_INVALID 2u
#define APP_TUNE_RESULT_STATE 3u

// ! ========================= 变 量 声 明 ========================= ! //

static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;
static uint8_t s_rx_buf[APP_TUNE_RX_SIZE];
static uint8_t s_rx_byte = 0u;
static volatile uint32_t s_rx_overflow = 0u;
static char s_line[APP_TUNE_LINE_SIZE];
static uint16_t s_line_len = 0u;
static bool s_line_drop = false;
static bool s_initialized = false;
static BalanceTuneConfig s_pending = { 0 };
static BalanceTuneConfig s_apply = { 0 };
static AppTuneStatus s_status = { 0 };
static uint16_t s_apply_seq = 0u;
static uint16_t s_cmd_seq = 0u;
static uint8_t s_result = APP_TUNE_RESULT_NONE;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief app_tune_start_rx 内部辅助函数
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_start_rx(void);
/**
 * @brief app_tune_rx_complete 内部辅助函数
 */
static void app_tune_rx_complete(void);
/**
 * @brief app_tune_rx_error 内部辅助函数
 */
static void app_tune_rx_error(void);
/**
 * @brief app_tune_pop 内部辅助函数
 * @param data 数据缓冲区指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_pop(uint8_t* data);
/**
 * @brief app_tune_reply 内部辅助函数
 * @param format 格式字符串
 */
static void app_tune_reply(const char* format, ...);
/**
 * @brief app_tune_split 内部辅助函数
 * @param line 文本缓冲区指针
 * @param argv 参数数组
 * @return 参数数量
 */
static uint8_t app_tune_split(char* line, char* argv[]);
/**
 * @brief app_tune_eq 内部辅助函数
 * @param left 左侧字符串
 * @param right 右侧字符串
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_eq(const char* left, const char* right);
/**
 * @brief app_tune_parse_float 内部辅助函数
 * @param text 文本字符串
 * @param value 数值指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_parse_float(const char* text, float* value);
/**
 * @brief app_tune_parse_u32 内部辅助函数
 * @param text 文本字符串
 * @param value 数值指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_parse_u32(const char* text, uint32_t* value);
/**
 * @brief app_tune_parse_bool 内部辅助函数
 * @param text 文本字符串
 * @param value 数值指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_parse_bool(const char* text, bool* value);
/**
 * @brief app_tune_parse_axis 内部辅助函数
 * @param text 文本字符串
 * @param axis 参数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_parse_axis(const char* text, BalanceAxis* axis);
/**
 * @brief app_tune_range 内部辅助函数
 * @param value 数值
 * @param min 参数值
 * @param max 参数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_range(float value, float min, float max);
/**
 * @brief app_tune_sign 内部辅助函数
 * @param value 数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_sign(float value);
/**
 * @brief app_tune_defaults 内部辅助函数
 * @param config 配置结构体指针
 */
static void app_tune_defaults(BalanceTuneConfig* config);
/**
 * @brief app_tune_valid 内部辅助函数
 * @param config 配置结构体指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_valid(const BalanceTuneConfig* config);
/**
 * @brief app_tune_mark_dirty 内部辅助函数
 */
static void app_tune_mark_dirty(void);
/**
 * @brief app_tune_handle_line 内部辅助函数
 * @param line 文本缓冲区指针
 */
static void app_tune_handle_line(char* line);
/**
 * @brief app_tune_set_param 内部辅助函数
 * @param name 参数名称
 * @param text 参数文本
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_set_param(const char* name, const char* text);
/**
 * @brief app_tune_get_param 内部辅助函数
 * @param name 参数名称
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_get_param(const char* name);
/**
 * @brief app_tune_get_all 内部辅助函数
 */
static void app_tune_get_all(void);
/**
 * @brief app_tune_set_ctrl 内部辅助函数
 * @param name 参数名称
 * @param enable 开关值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_set_ctrl(const char* name, bool enable);
/**
 * @brief app_tune_send_cmd 内部辅助函数
 * @param enable 使能值
 * @param clear_fault 清除故障值
 * @param mode 工作模式
 * @param target_m 目标位置，单位 m
 */
static void app_tune_send_cmd(bool enable,
                              bool clear_fault,
                              BalanceMode mode,
                              float target_m);
/**
 * @brief app_tune_print_status 内部辅助函数
 */
static void app_tune_print_status(void);
/**
 * @brief app_tune_print_help 内部辅助函数
 */
static void app_tune_print_help(void);
/**
 * @brief app_tune_report_result 内部辅助函数
 */
static void app_tune_report_result(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool app_tune_init(void) {
    memset(&s_status, 0, sizeof(s_status));
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_line_len = 0u;
    s_line_drop = false;
    s_rx_overflow = 0u;
    s_apply_seq = 0u;
    s_cmd_seq = 0u;
    s_result = APP_TUNE_RESULT_NONE;

    if(balance_get_tune(&s_pending) != BALANCE_STATUS_OK) {
        return false;
    }

    s_status.state = APP_TUNE_IDLE;
    uart_register_rx_complete_callback(&huart1, app_tune_rx_complete);
    uart_register_error_callback(&huart1, app_tune_rx_error);
    if(!app_tune_start_rx()) {
        return false;
    }

    s_initialized = true;
    app_tune_reply("READY baud=921600 type=@HELP");
    return true;
}

void app_tune_process(void) {
    uint8_t data;

    if(!s_initialized) {
        return;
    }

    s_status.overflow_count = s_rx_overflow;
    app_tune_report_result();
    while(app_tune_pop(&data)) {
        if(data == '\r' || data == '\n') {
            if(s_line_drop) {
                s_line_drop = false;
                s_line_len = 0u;
                s_status.reject_count++;
                s_status.state = APP_TUNE_REJECTED;
                app_tune_reply("REJECT line too long");
                continue;
            }
            if(s_line_len == 0u) {
                continue;
            }

            s_line[s_line_len] = '\0';
            s_status.rx_line_count++;
            s_status.last_rx_ms = delay_now_ms();
            app_tune_handle_line(s_line);
            s_line_len = 0u;
            continue;
        }

        if(s_line_drop) {
            continue;
        }
        if(s_line_len >= (APP_TUNE_LINE_SIZE - 1u)) {
            s_line_drop = true;
            continue;
        }

        if(isprint((int)data) != 0 || data == '\t') {
            s_line[s_line_len++] = (char)data;
        }
    }
}

void app_tune_apply_pending(void) {
    BalanceStatusCode result;

    if(!s_initialized || !s_status.apply_pending) {
        return;
    }

    s_status.apply_pending = false;
    result = balance_apply_tune(&s_apply);
    if(result == BALANCE_STATUS_OK) {
        s_status.apply_count++;
        s_status.dirty = s_status.session_seq != s_apply_seq;
        s_status.state = s_status.dirty ? APP_TUNE_EDIT : APP_TUNE_APPLIED;
        s_result = APP_TUNE_RESULT_OK;
    }
    else {
        s_status.reject_count++;
        s_status.state = APP_TUNE_REJECTED;
        s_result = result == BALANCE_STATUS_STATE_DENIED
                       ? APP_TUNE_RESULT_STATE
                       : APP_TUNE_RESULT_INVALID;
    }
}

const AppTuneStatus* app_tune_get_status(void) {
    return &s_status;
}

const char* app_tune_state_str(AppTuneState state) {
    switch(state) {
        case APP_TUNE_IDLE:
            return "IDLE";
        case APP_TUNE_EDIT:
            return "EDIT";
        case APP_TUNE_APPLY_PENDING:
            return "PENDING";
        case APP_TUNE_APPLIED:
            return "APPLIED";
        case APP_TUNE_REJECTED:
            return "REJECTED";
        default:
            return "UNKNOWN";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool app_tune_start_rx(void) {
    return uart_receive_it(&huart1, &s_rx_byte, 1u);
}

static void app_tune_rx_complete(void) {
    uint16_t next = (uint16_t)((s_rx_head + 1u) % APP_TUNE_RX_SIZE);

    if(next == s_rx_tail) {
        s_rx_overflow++;
    }
    else {
        s_rx_buf[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }

    (void)app_tune_start_rx();
}

static void app_tune_rx_error(void) {
    (void)app_tune_start_rx();
}

static bool app_tune_pop(uint8_t* data) {
    if(data == 0 || s_rx_tail == s_rx_head) {
        return false;
    }

    *data = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) % APP_TUNE_RX_SIZE);
    return true;
}

static void app_tune_reply(const char* format, ...) {
    char buffer[APP_TUNE_REPLY_SIZE];
    va_list args;
    int head_len;
    int body_len;
    uint32_t len;

    if(format == 0) {
        return;
    }

    head_len = snprintf(buffer, sizeof(buffer), "[TUNE] ");
    if(head_len < 0 || (uint32_t)head_len >= sizeof(buffer)) {
        return;
    }

    va_start(args, format);
    body_len = vsnprintf(&buffer[head_len], sizeof(buffer) - (uint32_t)head_len, format, args);
    va_end(args);
    if(body_len < 0) {
        return;
    }

    len = (uint32_t)head_len + (uint32_t)body_len;
    if(len > sizeof(buffer) - 3u) {
        len = sizeof(buffer) - 3u;
    }
    buffer[len++] = '\r';
    buffer[len++] = '\n';
    buffer[len] = '\0';
    (void)uart1_write_blocking(buffer, len);
}

static uint8_t app_tune_split(char* line, char* argv[]) {
    uint8_t argc = 0u;
    char* cursor = line;

    while(*cursor != '\0' && argc < APP_TUNE_ARG_MAX) {
        while(isspace((int)(unsigned char)*cursor) != 0) {
            cursor++;
        }
        if(*cursor == '\0') {
            break;
        }

        argv[argc++] = cursor;
        while(*cursor != '\0' && isspace((int)(unsigned char)*cursor) == 0) {
            cursor++;
        }
        if(*cursor != '\0') {
            *cursor++ = '\0';
        }
    }

    return argc;
}

static bool app_tune_eq(const char* left, const char* right) {
    if(left == 0 || right == 0) {
        return false;
    }

    while(*left != '\0' && *right != '\0') {
        if(toupper((int)(unsigned char)*left) != toupper((int)(unsigned char)*right)) {
            return false;
        }
        left++;
        right++;
    }

    return *left == '\0' && *right == '\0';
}

static bool app_tune_parse_float(const char* text, float* value) {
    char* end;
    float parsed;

    if(text == 0 || value == 0) {
        return false;
    }

    parsed = strtof(text, &end);
    if(end == text || *end != '\0' || isfinite(parsed) == 0) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool app_tune_parse_u32(const char* text, uint32_t* value) {
    char* end;
    unsigned long parsed;

    if(text == 0 || value == 0 || *text == '-') {
        return false;
    }

    parsed = strtoul(text, &end, 0);
    if(end == text || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static bool app_tune_parse_bool(const char* text, bool* value) {
    if(text == 0 || value == 0) {
        return false;
    }

    if(app_tune_eq(text, "1") || app_tune_eq(text, "ON") || app_tune_eq(text, "TRUE")) {
        *value = true;
        return true;
    }
    if(app_tune_eq(text, "0") || app_tune_eq(text, "OFF") || app_tune_eq(text, "FALSE")) {
        *value = false;
        return true;
    }

    return false;
}

static bool app_tune_parse_axis(const char* text, BalanceAxis* axis) {
    uint32_t value;

    if(text == 0 || axis == 0) {
        return false;
    }

    if(app_tune_eq(text, "X")) {
        *axis = BALANCE_AXIS_X;
        return true;
    }
    if(app_tune_eq(text, "Y")) {
        *axis = BALANCE_AXIS_Y;
        return true;
    }
    if(app_tune_eq(text, "Z")) {
        *axis = BALANCE_AXIS_Z;
        return true;
    }
    if(app_tune_parse_u32(text, &value) && value <= (uint32_t)BALANCE_AXIS_Z) {
        *axis = (BalanceAxis)value;
        return true;
    }

    return false;
}

static bool app_tune_range(float value, float min, float max) {
    return isfinite(value) != 0 && value >= min && value <= max;
}

static bool app_tune_sign(float value) {
    return value == 1.0f || value == -1.0f;
}

static void app_tune_defaults(BalanceTuneConfig* config) {
    if(config == 0) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->motor_zero_rad = APP_CFG_MOTOR_ZERO_RAD;
    config->motor_min_rad = APP_CFG_MOTOR_MIN_RAD;
    config->motor_max_rad = APP_CFG_MOTOR_MAX_RAD;
    config->motor_speed_rad_s = APP_CFG_MOTOR_SPEED_RAD_S;
    config->motor_rate_rad_s = APP_CFG_MOTOR_RATE_RAD_S;
    config->motor_sign = APP_CFG_MOTOR_SIGN;
    config->acc_axis = APP_CFG_ACC_AXIS;
    config->acc_sign = APP_CFG_ACC_SIGN;
    config->pitch_sign = APP_CFG_PITCH_SIGN;
    config->ball_sign = APP_CFG_BALL_SIGN;
    config->level_gain = APP_CFG_LEVEL_GAIN;
    config->acc_gain = APP_CFG_ACC_GAIN;
    config->acc_tau_s = APP_CFG_ACC_TAU_S;
    config->pos_tau_s = APP_CFG_POS_TAU_S;
    config->vel_tau_s = APP_CFG_VEL_TAU_S;
    config->pos_pid.mode = APP_CFG_POS_MODE;
    config->pos_pid.features = APP_CFG_POS_FEATURES;
    config->pos_pid.kp = APP_CFG_POS_KP;
    config->pos_pid.ki = APP_CFG_POS_KI;
    config->pos_pid.kd = APP_CFG_POS_KD;
    config->pos_pid.max_out = APP_CFG_POS_MAX_OUT_RAD;
    config->pos_pid.integral_separation = APP_CFG_POS_INT_SEP_M;
    config->pos_pid.dead_band = APP_CFG_POS_DEAD_BAND_M;
    config->pos_pid.diff_filter_alpha = APP_CFG_POS_DIFF_ALPHA;
    config->pos_pid.output_max_rate = APP_CFG_POS_OUT_RATE_RAD_S;
    config->vision_angle_max_rad = APP_CFG_VISION_ANGLE_MAX_RAD;
    config->vision_decay_tau_s = APP_CFG_VISION_DECAY_TAU_S;
    config->target_limit_m = APP_CFG_TARGET_LIMIT_M;
    config->vision_quality_min = APP_CFG_VISION_QUALITY_MIN;
    config->vision_timeout_ms = APP_CFG_VISION_TIMEOUT_MS;
    config->sweep_pos_m = APP_CFG_SWEEP_POS_M;
    config->sweep_err_m = APP_CFG_SWEEP_ERR_M;
    config->sweep_vel_m_s = APP_CFG_SWEEP_VEL_M_S;
    config->sweep_stable_ms = APP_CFG_SWEEP_STABLE_MS;
    config->ctrl_mask = APP_CFG_CTRL_MASK;
}

static bool app_tune_valid(const BalanceTuneConfig* config) {
    if(config == 0) {
        return false;
    }

    return app_tune_range(config->motor_zero_rad,
                          APP_TUNE_MOTOR_POS_MIN_RAD,
                          APP_TUNE_MOTOR_POS_MAX_RAD) &&
           app_tune_range(config->motor_min_rad,
                          APP_TUNE_MOTOR_POS_MIN_RAD,
                          APP_TUNE_MOTOR_POS_MAX_RAD) &&
           app_tune_range(config->motor_max_rad,
                          APP_TUNE_MOTOR_POS_MIN_RAD,
                          APP_TUNE_MOTOR_POS_MAX_RAD) &&
           config->motor_min_rad < config->motor_zero_rad &&
           config->motor_zero_rad < config->motor_max_rad &&
           app_tune_range(config->motor_speed_rad_s,
                          APP_TUNE_RATE_MIN_RAD_S,
                          APP_TUNE_RATE_MAX_RAD_S) &&
           app_tune_range(config->motor_rate_rad_s,
                          APP_TUNE_RATE_MIN_RAD_S,
                          APP_TUNE_RATE_MAX_RAD_S) &&
           app_tune_sign(config->motor_sign) &&
           app_tune_sign(config->acc_sign) &&
           app_tune_sign(config->pitch_sign) &&
           app_tune_sign(config->ball_sign) &&
           config->acc_axis <= BALANCE_AXIS_Z &&
           app_tune_range(config->level_gain, APP_TUNE_GAIN_MIN, APP_TUNE_GAIN_MAX) &&
           app_tune_range(config->acc_gain, APP_TUNE_GAIN_MIN, APP_TUNE_GAIN_MAX) &&
           app_tune_range(config->acc_tau_s, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S) &&
           app_tune_range(config->pos_tau_s, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S) &&
           app_tune_range(config->vel_tau_s, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S) &&
           app_tune_range(config->pos_pid.kp, APP_TUNE_KP_MIN, APP_TUNE_KP_MAX) &&
           app_tune_range(config->pos_pid.ki, APP_TUNE_KI_MIN, APP_TUNE_KI_MAX) &&
           app_tune_range(config->pos_pid.kd, APP_TUNE_KD_MIN, APP_TUNE_KD_MAX) &&
           app_tune_range(config->pos_pid.max_out,
                          APP_TUNE_ANGLE_MIN_RAD,
                          APP_TUNE_ANGLE_MAX_RAD) &&
           app_tune_range(config->pos_pid.integral_separation,
                          0.0f,
                          APP_TUNE_POS_LIMIT_MAX_M) &&
           app_tune_range(config->pos_pid.dead_band, 0.0f, APP_TUNE_POS_LIMIT_MAX_M) &&
           app_tune_range(config->pos_pid.diff_filter_alpha, 0.0f, 1.0f) &&
           app_tune_range(config->pos_pid.output_max_rate,
                          APP_TUNE_RATE_MIN_RAD_S,
                          APP_TUNE_RATE_MAX_RAD_S) &&
           app_tune_range(config->vision_angle_max_rad,
                          APP_TUNE_ANGLE_MIN_RAD,
                          APP_TUNE_ANGLE_MAX_RAD) &&
           app_tune_range(config->vision_decay_tau_s,
                          APP_TUNE_TAU_MIN_S,
                          APP_TUNE_TAU_MAX_S) &&
           app_tune_range(config->target_limit_m, 0.001f, APP_TUNE_TARGET_MAX_M) &&
           config->vision_quality_min <= APP_TUNE_QUALITY_MAX &&
           config->vision_timeout_ms >= APP_TUNE_TIMEOUT_MIN_MS &&
           config->vision_timeout_ms <= APP_TUNE_TIMEOUT_MAX_MS &&
           app_tune_range(config->sweep_pos_m, 0.001f, config->target_limit_m) &&
           app_tune_range(config->sweep_err_m, 0.0005f, config->sweep_pos_m) &&
           app_tune_range(config->sweep_vel_m_s, 0.001f, 1.0f) &&
           config->sweep_stable_ms > 0u &&
           config->sweep_stable_ms <= APP_TUNE_SWEEP_STABLE_MAX_MS &&
           (config->ctrl_mask & (uint8_t)(~BALANCE_CTRL_ALL)) == 0u;
}

static void app_tune_mark_dirty(void) {
    s_status.session_seq++;
    s_status.dirty = true;
    s_status.state = APP_TUNE_EDIT;
}

static void app_tune_handle_line(char* line) {
    char* argv[APP_TUNE_ARG_MAX] = { 0 };
    uint8_t argc;
    bool enable;
    float value;
    const BalanceStatus* balance_status;

    while(isspace((int)(unsigned char)*line) != 0) {
        line++;
    }
    if(*line == '@') {
        line++;
    }

    argc = app_tune_split(line, argv);
    if(argc == 0u) {
        return;
    }

    if(app_tune_eq(argv[0], "HELP")) {
        app_tune_print_help();
        return;
    }
    if(app_tune_eq(argv[0], "STATUS")) {
        app_tune_print_status();
        return;
    }
    if(app_tune_eq(argv[0], "GET") && argc == 2u) {
        if(app_tune_eq(argv[1], "ALL")) {
            app_tune_get_all();
        }
        else if(!app_tune_get_param(argv[1])) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT unknown param=%s", argv[1]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "SET") && argc == 3u) {
        if(app_tune_set_param(argv[1], argv[2])) {
            app_tune_mark_dirty();
            app_tune_reply("SET %s=%s PENDING", argv[1], argv[2]);
        }
        else {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT param=%s value=%s", argv[1], argv[2]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "CTRL") && argc == 3u) {
        if(app_tune_parse_bool(argv[2], &enable) && app_tune_set_ctrl(argv[1], enable)) {
            app_tune_mark_dirty();
            app_tune_reply("CTRL %s=%u PENDING", argv[1], enable ? 1u : 0u);
        }
        else {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT ctrl=%s value=%s", argv[1], argv[2]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "APPLY") && argc == 1u) {
        if(!s_status.dirty) {
            app_tune_reply("APPLY SKIP no pending change");
            return;
        }
        if(!app_tune_valid(&s_pending)) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT pending config invalid");
            return;
        }

        s_apply = s_pending;
        s_apply_seq = s_status.session_seq;
        s_status.apply_pending = true;
        s_status.state = APP_TUNE_APPLY_PENDING;
        app_tune_reply("APPLY PENDING seq=%u", s_apply_seq);
        return;
    }
    if(app_tune_eq(argv[0], "CANCEL") && argc == 1u) {
        if(balance_get_tune(&s_pending) == BALANCE_STATUS_OK) {
            s_status.dirty = false;
            s_status.apply_pending = false;
            s_status.state = APP_TUNE_IDLE;
            app_tune_reply("CANCEL OK");
        }
        else {
            app_tune_reply("CANCEL FAILED");
        }
        return;
    }
    if(app_tune_eq(argv[0], "DEFAULT") && argc == 1u) {
        app_tune_defaults(&s_pending);
        app_tune_mark_dirty();
        app_tune_reply("DEFAULT PENDING");
        return;
    }
    if(app_tune_eq(argv[0], "ENABLE") && argc == 2u) {
        if(!app_tune_parse_bool(argv[1], &enable)) {
            app_tune_reply("REJECT enable=%s", argv[1]);
            return;
        }
        balance_status = balance_get_status();
        app_tune_send_cmd(enable,
                          false,
                          balance_status->mode,
                          balance_status->target_m);
        app_tune_reply("ENABLE %u", enable ? 1u : 0u);
        return;
    }
    if(app_tune_eq(argv[0], "MODE") && argc == 2u) {
        balance_status = balance_get_status();
        if(app_tune_eq(argv[1], "CENTER")) {
            app_tune_send_cmd(balance_status->enable_requested,
                              false,
                              BALANCE_MODE_HOLD_CENTER,
                              0.0f);
        }
        else if(app_tune_eq(argv[1], "POS")) {
            app_tune_send_cmd(balance_status->enable_requested,
                              false,
                              BALANCE_MODE_HOLD_POS,
                              balance_status->target_m);
        }
        else if(app_tune_eq(argv[1], "SWEEP")) {
            app_tune_send_cmd(balance_status->enable_requested,
                              false,
                              BALANCE_MODE_SWEEP_TEST,
                              0.0f);
        }
        else {
            app_tune_reply("REJECT mode=%s", argv[1]);
            return;
        }
        app_tune_reply("MODE %s", argv[1]);
        return;
    }
    if(app_tune_eq(argv[0], "TARGET") && argc == 2u) {
        if(!app_tune_parse_float(argv[1], &value) ||
           !app_tune_range(value, -s_pending.target_limit_m, s_pending.target_limit_m)) {
            app_tune_reply("REJECT target=%s", argv[1]);
            return;
        }
        balance_status = balance_get_status();
        app_tune_send_cmd(balance_status->enable_requested,
                          false,
                          BALANCE_MODE_HOLD_POS,
                          value);
        app_tune_reply("TARGET %.6f m", value);
        return;
    }
    if(app_tune_eq(argv[0], "CLEAR") && argc == 1u) {
        balance_status = balance_get_status();
        app_tune_send_cmd(balance_status->enable_requested,
                          true,
                          balance_status->mode,
                          balance_status->target_m);
        app_tune_reply("CLEAR requested");
        return;
    }

    s_status.reject_count++;
    s_status.state = APP_TUNE_REJECTED;
    app_tune_reply("REJECT command type=@HELP");
}

static bool app_tune_set_param(const char* name, const char* text) {
    float value;
    uint32_t u32_value;
    BalanceAxis axis;

    if(app_tune_eq(name, "ACC_AXIS")) {
        if(!app_tune_parse_axis(text, &axis))
            return false;
        s_pending.acc_axis = axis;
        return true;
    }
    if(app_tune_eq(name, "QUALITY_MIN")) {
        if(!app_tune_parse_u32(text, &u32_value) || u32_value > APP_TUNE_QUALITY_MAX)
            return false;
        s_pending.vision_quality_min = (uint8_t)u32_value;
        return true;
    }
    if(app_tune_eq(name, "VIS_TIMEOUT")) {
        if(!app_tune_parse_u32(text, &u32_value) ||
           u32_value < APP_TUNE_TIMEOUT_MIN_MS || u32_value > APP_TUNE_TIMEOUT_MAX_MS)
            return false;
        s_pending.vision_timeout_ms = u32_value;
        return true;
    }
    if(app_tune_eq(name, "SWEEP_STABLE")) {
        if(!app_tune_parse_u32(text, &u32_value) ||
           u32_value == 0u || u32_value > APP_TUNE_SWEEP_STABLE_MAX_MS)
            return false;
        s_pending.sweep_stable_ms = u32_value;
        return true;
    }
    if(!app_tune_parse_float(text, &value)) {
        return false;
    }

    if(app_tune_eq(name, "LEVEL_GAIN") && app_tune_range(value, APP_TUNE_GAIN_MIN, APP_TUNE_GAIN_MAX))
        s_pending.level_gain = value;
    else if(app_tune_eq(name, "ACC_GAIN") && app_tune_range(value, APP_TUNE_GAIN_MIN, APP_TUNE_GAIN_MAX))
        s_pending.acc_gain = value;
    else if(app_tune_eq(name, "ACC_TAU") && app_tune_range(value, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S))
        s_pending.acc_tau_s = value;
    else if(app_tune_eq(name, "POS_TAU") && app_tune_range(value, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S))
        s_pending.pos_tau_s = value;
    else if(app_tune_eq(name, "VEL_TAU") && app_tune_range(value, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S))
        s_pending.vel_tau_s = value;
    else if(app_tune_eq(name, "KP") && app_tune_range(value, APP_TUNE_KP_MIN, APP_TUNE_KP_MAX))
        s_pending.pos_pid.kp = value;
    else if(app_tune_eq(name, "KI") && app_tune_range(value, APP_TUNE_KI_MIN, APP_TUNE_KI_MAX))
        s_pending.pos_pid.ki = value;
    else if(app_tune_eq(name, "KD") && app_tune_range(value, APP_TUNE_KD_MIN, APP_TUNE_KD_MAX))
        s_pending.pos_pid.kd = value;
    else if(app_tune_eq(name, "POS_MAX") && app_tune_range(value, APP_TUNE_ANGLE_MIN_RAD, APP_TUNE_ANGLE_MAX_RAD))
        s_pending.pos_pid.max_out = value;
    else if(app_tune_eq(name, "INT_SEP") && app_tune_range(value, 0.0f, APP_TUNE_POS_LIMIT_MAX_M))
        s_pending.pos_pid.integral_separation = value;
    else if(app_tune_eq(name, "DEAD_BAND") && app_tune_range(value, 0.0f, APP_TUNE_POS_LIMIT_MAX_M))
        s_pending.pos_pid.dead_band = value;
    else if(app_tune_eq(name, "DIFF_ALPHA") && app_tune_range(value, 0.0f, 1.0f))
        s_pending.pos_pid.diff_filter_alpha = value;
    else if(app_tune_eq(name, "POS_RATE") && app_tune_range(value, APP_TUNE_RATE_MIN_RAD_S, APP_TUNE_RATE_MAX_RAD_S))
        s_pending.pos_pid.output_max_rate = value;
    else if(app_tune_eq(name, "VIS_MAX") && app_tune_range(value, APP_TUNE_ANGLE_MIN_RAD, APP_TUNE_ANGLE_MAX_RAD))
        s_pending.vision_angle_max_rad = value;
    else if(app_tune_eq(name, "VIS_FADE") && app_tune_range(value, APP_TUNE_TAU_MIN_S, APP_TUNE_TAU_MAX_S))
        s_pending.vision_decay_tau_s = value;
    else if(app_tune_eq(name, "TARGET_LIMIT") && app_tune_range(value, 0.001f, APP_TUNE_TARGET_MAX_M))
        s_pending.target_limit_m = value;
    else if(app_tune_eq(name, "MOTOR_SPEED") && app_tune_range(value, APP_TUNE_RATE_MIN_RAD_S, APP_TUNE_RATE_MAX_RAD_S))
        s_pending.motor_speed_rad_s = value;
    else if(app_tune_eq(name, "MOTOR_RATE") && app_tune_range(value, APP_TUNE_RATE_MIN_RAD_S, APP_TUNE_RATE_MAX_RAD_S))
        s_pending.motor_rate_rad_s = value;
    else if(app_tune_eq(name, "MOTOR_ZERO") && app_tune_range(value, APP_TUNE_MOTOR_POS_MIN_RAD, APP_TUNE_MOTOR_POS_MAX_RAD))
        s_pending.motor_zero_rad = value;
    else if(app_tune_eq(name, "MOTOR_MIN") && app_tune_range(value, APP_TUNE_MOTOR_POS_MIN_RAD, APP_TUNE_MOTOR_POS_MAX_RAD))
        s_pending.motor_min_rad = value;
    else if(app_tune_eq(name, "MOTOR_MAX") && app_tune_range(value, APP_TUNE_MOTOR_POS_MIN_RAD, APP_TUNE_MOTOR_POS_MAX_RAD))
        s_pending.motor_max_rad = value;
    else if(app_tune_eq(name, "MOTOR_SIGN") && app_tune_sign(value))
        s_pending.motor_sign = value;
    else if(app_tune_eq(name, "PITCH_SIGN") && app_tune_sign(value))
        s_pending.pitch_sign = value;
    else if(app_tune_eq(name, "ACC_SIGN") && app_tune_sign(value))
        s_pending.acc_sign = value;
    else if(app_tune_eq(name, "BALL_SIGN") && app_tune_sign(value))
        s_pending.ball_sign = value;
    else if(app_tune_eq(name, "SWEEP_POS") && app_tune_range(value, 0.001f, APP_TUNE_TARGET_MAX_M))
        s_pending.sweep_pos_m = value;
    else if(app_tune_eq(name, "SWEEP_ERR") && app_tune_range(value, 0.0005f, APP_TUNE_TARGET_MAX_M))
        s_pending.sweep_err_m = value;
    else if(app_tune_eq(name, "SWEEP_VEL") && app_tune_range(value, 0.001f, 1.0f))
        s_pending.sweep_vel_m_s = value;
    else
        return false;

    return true;
}

static bool app_tune_get_param(const char* name) {
    if(app_tune_eq(name, "LEVEL_GAIN"))
        app_tune_reply("LEVEL_GAIN=%.6f", s_pending.level_gain);
    else if(app_tune_eq(name, "ACC_GAIN"))
        app_tune_reply("ACC_GAIN=%.6f", s_pending.acc_gain);
    else if(app_tune_eq(name, "ACC_TAU"))
        app_tune_reply("ACC_TAU=%.6f s", s_pending.acc_tau_s);
    else if(app_tune_eq(name, "POS_TAU"))
        app_tune_reply("POS_TAU=%.6f s", s_pending.pos_tau_s);
    else if(app_tune_eq(name, "VEL_TAU"))
        app_tune_reply("VEL_TAU=%.6f s", s_pending.vel_tau_s);
    else if(app_tune_eq(name, "KP"))
        app_tune_reply("KP=%.6f", s_pending.pos_pid.kp);
    else if(app_tune_eq(name, "KI"))
        app_tune_reply("KI=%.6f", s_pending.pos_pid.ki);
    else if(app_tune_eq(name, "KD"))
        app_tune_reply("KD=%.6f", s_pending.pos_pid.kd);
    else if(app_tune_eq(name, "POS_MAX"))
        app_tune_reply("POS_MAX=%.6f rad", s_pending.pos_pid.max_out);
    else if(app_tune_eq(name, "INT_SEP"))
        app_tune_reply("INT_SEP=%.6f m", s_pending.pos_pid.integral_separation);
    else if(app_tune_eq(name, "DEAD_BAND"))
        app_tune_reply("DEAD_BAND=%.6f m", s_pending.pos_pid.dead_band);
    else if(app_tune_eq(name, "DIFF_ALPHA"))
        app_tune_reply("DIFF_ALPHA=%.6f", s_pending.pos_pid.diff_filter_alpha);
    else if(app_tune_eq(name, "POS_RATE"))
        app_tune_reply("POS_RATE=%.6f rad/s", s_pending.pos_pid.output_max_rate);
    else if(app_tune_eq(name, "VIS_MAX"))
        app_tune_reply("VIS_MAX=%.6f rad", s_pending.vision_angle_max_rad);
    else if(app_tune_eq(name, "VIS_FADE"))
        app_tune_reply("VIS_FADE=%.6f s", s_pending.vision_decay_tau_s);
    else if(app_tune_eq(name, "TARGET_LIMIT"))
        app_tune_reply("TARGET_LIMIT=%.6f m", s_pending.target_limit_m);
    else if(app_tune_eq(name, "QUALITY_MIN"))
        app_tune_reply("QUALITY_MIN=%u", s_pending.vision_quality_min);
    else if(app_tune_eq(name, "VIS_TIMEOUT"))
        app_tune_reply("VIS_TIMEOUT=%lu ms", (unsigned long)s_pending.vision_timeout_ms);
    else if(app_tune_eq(name, "MOTOR_SPEED"))
        app_tune_reply("MOTOR_SPEED=%.6f rad/s", s_pending.motor_speed_rad_s);
    else if(app_tune_eq(name, "MOTOR_RATE"))
        app_tune_reply("MOTOR_RATE=%.6f rad/s", s_pending.motor_rate_rad_s);
    else if(app_tune_eq(name, "MOTOR_ZERO"))
        app_tune_reply("MOTOR_ZERO=%.6f rad", s_pending.motor_zero_rad);
    else if(app_tune_eq(name, "MOTOR_MIN"))
        app_tune_reply("MOTOR_MIN=%.6f rad", s_pending.motor_min_rad);
    else if(app_tune_eq(name, "MOTOR_MAX"))
        app_tune_reply("MOTOR_MAX=%.6f rad", s_pending.motor_max_rad);
    else if(app_tune_eq(name, "MOTOR_SIGN"))
        app_tune_reply("MOTOR_SIGN=%.1f", s_pending.motor_sign);
    else if(app_tune_eq(name, "PITCH_SIGN"))
        app_tune_reply("PITCH_SIGN=%.1f", s_pending.pitch_sign);
    else if(app_tune_eq(name, "ACC_SIGN"))
        app_tune_reply("ACC_SIGN=%.1f", s_pending.acc_sign);
    else if(app_tune_eq(name, "BALL_SIGN"))
        app_tune_reply("BALL_SIGN=%.1f", s_pending.ball_sign);
    else if(app_tune_eq(name, "ACC_AXIS"))
        app_tune_reply("ACC_AXIS=%u", (unsigned int)s_pending.acc_axis);
    else if(app_tune_eq(name, "SWEEP_POS"))
        app_tune_reply("SWEEP_POS=%.6f m", s_pending.sweep_pos_m);
    else if(app_tune_eq(name, "SWEEP_ERR"))
        app_tune_reply("SWEEP_ERR=%.6f m", s_pending.sweep_err_m);
    else if(app_tune_eq(name, "SWEEP_VEL"))
        app_tune_reply("SWEEP_VEL=%.6f m/s", s_pending.sweep_vel_m_s);
    else if(app_tune_eq(name, "SWEEP_STABLE"))
        app_tune_reply("SWEEP_STABLE=%lu ms", (unsigned long)s_pending.sweep_stable_ms);
    else
        return false;

    return true;
}

static void app_tune_get_all(void) {
    app_tune_reply("CTRL level=%u acc=%u vision=%u vis_i=%u dirty=%u",
                   (s_pending.ctrl_mask & BALANCE_CTRL_LEVEL) != 0u,
                   (s_pending.ctrl_mask & BALANCE_CTRL_ACC) != 0u,
                   (s_pending.ctrl_mask & BALANCE_CTRL_VISION) != 0u,
                   (s_pending.ctrl_mask & BALANCE_CTRL_VISION_I) != 0u,
                   s_status.dirty ? 1u : 0u);
    app_tune_reply("GAIN level=%.4f acc=%.4f kp=%.4f ki=%.4f kd=%.4f",
                   s_pending.level_gain,
                   s_pending.acc_gain,
                   s_pending.pos_pid.kp,
                   s_pending.pos_pid.ki,
                   s_pending.pos_pid.kd);
    app_tune_reply("FILTER acc=%.4f pos=%.4f vel=%.4f diff=%.4f",
                   s_pending.acc_tau_s,
                   s_pending.pos_tau_s,
                   s_pending.vel_tau_s,
                   s_pending.pos_pid.diff_filter_alpha);
    app_tune_reply("OUT pos_max=%.4f pos_rate=%.4f vis_max=%.4f vis_fade=%.4f",
                   s_pending.pos_pid.max_out,
                   s_pending.pos_pid.output_max_rate,
                   s_pending.vision_angle_max_rad,
                   s_pending.vision_decay_tau_s);
    app_tune_reply("MOTOR zero=%.4f min=%.4f max=%.4f speed=%.4f rate=%.4f sign=%.1f",
                   s_pending.motor_zero_rad,
                   s_pending.motor_min_rad,
                   s_pending.motor_max_rad,
                   s_pending.motor_speed_rad_s,
                   s_pending.motor_rate_rad_s,
                   s_pending.motor_sign);
    app_tune_reply("DIR axis=%u pitch=%.1f acc=%.1f ball=%.1f",
                   (unsigned int)s_pending.acc_axis,
                   s_pending.pitch_sign,
                   s_pending.acc_sign,
                   s_pending.ball_sign);
    app_tune_reply("VISION quality=%u timeout=%lu target_limit=%.4f",
                   s_pending.vision_quality_min,
                   (unsigned long)s_pending.vision_timeout_ms,
                   s_pending.target_limit_m);
    app_tune_reply("SWEEP pos=%.4f err=%.4f vel=%.4f stable=%lu",
                   s_pending.sweep_pos_m,
                   s_pending.sweep_err_m,
                   s_pending.sweep_vel_m_s,
                   (unsigned long)s_pending.sweep_stable_ms);
}

static bool app_tune_set_ctrl(const char* name, bool enable) {
    uint8_t bit;

    if(app_tune_eq(name, "LEVEL"))
        bit = BALANCE_CTRL_LEVEL;
    else if(app_tune_eq(name, "ACC"))
        bit = BALANCE_CTRL_ACC;
    else if(app_tune_eq(name, "VISION"))
        bit = BALANCE_CTRL_VISION;
    else if(app_tune_eq(name, "VIS_I"))
        bit = BALANCE_CTRL_VISION_I;
    else
        return false;

    if(enable) {
        s_pending.ctrl_mask |= bit;
    }
    else {
        s_pending.ctrl_mask &= (uint8_t)(~bit);
    }
    return true;
}

static void app_tune_send_cmd(bool enable,
                              bool clear_fault,
                              BalanceMode mode,
                              float target_m) {
    BalanceCommand command;

    s_cmd_seq++;
    command.enable = enable;
    command.clear_fault = clear_fault;
    command.mode = mode;
    command.target_m = target_m;
    command.command_seq = s_cmd_seq;
    balance_set_command(&command);
}

static void app_tune_print_status(void) {
    const BalanceStatus* balance_status = balance_get_status();

    app_tune_reply("STATUS tune=%s dirty=%u pending=%u seq=%u rx=%lu apply=%lu reject=%lu overflow=%lu",
                   app_tune_state_str(s_status.state),
                   s_status.dirty ? 1u : 0u,
                   s_status.apply_pending ? 1u : 0u,
                   s_status.session_seq,
                   (unsigned long)s_status.rx_line_count,
                   (unsigned long)s_status.apply_count,
                   (unsigned long)s_status.reject_count,
                   (unsigned long)s_status.overflow_count);
    app_tune_reply("STATUS balance=%s mode=%s enable=%u ctrl=0x%02X ball_loop=%s vis=%u motor=%u fault=0x%08lX",
                   balance_state_str(balance_status->state),
                   balance_mode_str(balance_status->mode),
                   balance_status->enable_requested ? 1u : 0u,
                   balance_status->ctrl_mask,
                   balance_status->ball_loop_closed ? "CLOSED" : "OPEN",
                   balance_status->vision_valid ? 1u : 0u,
                   balance_status->motor_enabled ? 1u : 0u,
                   (unsigned long)balance_status->fault_flags);
}

static void app_tune_print_help(void) {
    app_tune_reply("CMD @GET ALL | @GET name | @SET name value | @APPLY | @CANCEL | @DEFAULT");
    app_tune_reply("CMD @CTRL LEVEL|ACC|VISION|VIS_I 0|1");
    app_tune_reply("CMD @ENABLE 0|1 | @MODE CENTER|POS|SWEEP | @TARGET meter | @CLEAR | @STATUS");
    app_tune_reply("PARAM LEVEL_GAIN ACC_GAIN ACC_TAU POS_TAU VEL_TAU KP KI KD");
    app_tune_reply("PARAM POS_MAX INT_SEP DEAD_BAND DIFF_ALPHA POS_RATE VIS_MAX VIS_FADE");
    app_tune_reply("PARAM MOTOR_SPEED MOTOR_RATE MOTOR_ZERO MOTOR_MIN MOTOR_MAX MOTOR_SIGN");
    app_tune_reply("PARAM PITCH_SIGN ACC_SIGN BALL_SIGN ACC_AXIS QUALITY_MIN VIS_TIMEOUT TARGET_LIMIT");
    app_tune_reply("PARAM SWEEP_POS SWEEP_ERR SWEEP_VEL SWEEP_STABLE");
}

static void app_tune_report_result(void) {
    uint8_t result = s_result;

    if(result == APP_TUNE_RESULT_NONE) {
        return;
    }
    s_result = APP_TUNE_RESULT_NONE;

    if(result == APP_TUNE_RESULT_OK) {
        app_tune_reply("APPLY OK seq=%u dirty=%u",
                       s_apply_seq,
                       s_status.dirty ? 1u : 0u);
    }
    else if(result == APP_TUNE_RESULT_STATE) {
        app_tune_reply("APPLY REJECT mechanical params require ENABLE 0");
    }
    else {
        app_tune_reply("APPLY REJECT invalid config");
    }
}
