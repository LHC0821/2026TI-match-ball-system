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

#define APP_TUNE_ARG_MAX 3u

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
static AppTuneStatus s_status = { 0 };
static uint16_t s_cmd_seq = 0u;

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
 * @brief app_tune_parse_bool 内部辅助函数
 * @param text 文本字符串
 * @param value 数值指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_parse_bool(const char* text, bool* value);
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
 * @brief app_tune_set_ctrl 修改运行中的平衡控制通道
 * @param name 通道名称
 * @param enable 是否启用
 * @return true 表示修改成功，false 表示通道或参数无效
 */
static bool app_tune_set_ctrl(const char* name, bool enable);
/**
 * @brief app_tune_get_param 内部辅助函数
 * @param name 参数名称
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool app_tune_get_param(const char* name);
/**
 * @brief app_tune_reply_ctrl 回复当前平衡控制通道状态
 */
static void app_tune_reply_ctrl(void);
/**
 * @brief app_tune_get_all 内部辅助函数
 */
static void app_tune_get_all(void);
/**
 * @brief app_tune_send_cmd 内部辅助函数
 * @param enable 使能值
 * @param clear_fault 清除故障值
 */
static void app_tune_send_cmd(bool enable, bool clear_fault);
/**
 * @brief app_tune_print_status 内部辅助函数
 */
static void app_tune_print_status(void);
/**
 * @brief app_tune_print_help 内部辅助函数
 */
static void app_tune_print_help(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool app_tune_init(void) {
    memset(&s_status, 0, sizeof(s_status));
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_line_len = 0u;
    s_line_drop = false;
    s_rx_overflow = 0u;
    s_cmd_seq = 0u;

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

const AppTuneStatus* app_tune_get_status(void) {
    return &s_status;
}

const char* app_tune_state_str(AppTuneState state) {
    switch(state) {
        case APP_TUNE_IDLE:
            return "IDLE";
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
    (void)uart1_write_it(buffer, len);
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

static void app_tune_defaults(BalanceTuneConfig* config) {
    if(config == 0) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->motor_zero_rad = APP_CFG_MOTOR_ZERO_RAD;
    config->motor_min_rad = APP_CFG_MOTOR_MIN_RAD;
    config->motor_max_rad = APP_CFG_MOTOR_MAX_RAD;
    config->motor_vel_rad_s = APP_CFG_MOTOR_VEL_RAD_S;
    config->motor_kp = APP_CFG_MOTOR_KP;
    config->motor_kd = APP_CFG_MOTOR_KD;
    config->motor_tor_nm = APP_CFG_MOTOR_TOR_NM;
    config->tor_pitch_gain = APP_CFG_TOR_PITCH_GAIN;
    config->level_gain = APP_CFG_LEVEL_GAIN;
    config->acc_gain = APP_CFG_ACC_GAIN;
    config->acc_tau_s = APP_CFG_ACC_TAU_S;
    config->acc_reverse_tau_s = APP_CFG_ACC_REVERSE_TAU_S;
    config->acc_reverse_threshold_m_s2 = APP_CFG_ACC_REVERSE_THRESHOLD_M_S2;
    config->vision_tau_s = APP_CFG_VISION_TAU_S;
    config->vision_kp = APP_CFG_VISION_KP;
    config->vision_ki = APP_CFG_VISION_KI;
    config->vision_kd = APP_CFG_VISION_KD;
    config->vision_max_angle_rad = APP_CFG_VISION_MAX_ANGLE_RAD;
}

static bool app_tune_valid(const BalanceTuneConfig* config) {
    if(config == 0) {
        return false;
    }

    return isfinite(config->motor_zero_rad) != 0 &&
           isfinite(config->motor_min_rad) != 0 &&
           isfinite(config->motor_max_rad) != 0 &&
           isfinite(config->motor_vel_rad_s) != 0 &&
           isfinite(config->motor_kp) != 0 &&
           isfinite(config->motor_kd) != 0 &&
           isfinite(config->motor_tor_nm) != 0 &&
           isfinite(config->tor_pitch_gain) != 0 &&
           isfinite(config->level_gain) != 0 &&
           isfinite(config->acc_gain) != 0 &&
           isfinite(config->acc_tau_s) != 0 &&
           isfinite(config->acc_reverse_tau_s) != 0 &&
           isfinite(config->acc_reverse_threshold_m_s2) != 0 &&
           isfinite(config->vision_tau_s) != 0 &&
           isfinite(config->vision_kp) != 0 &&
           isfinite(config->vision_ki) != 0 &&
           isfinite(config->vision_kd) != 0 &&
           isfinite(config->vision_max_angle_rad) != 0 &&
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
           config->acc_reverse_tau_s >= 0.0f &&
           config->acc_reverse_threshold_m_s2 >= 0.0f &&
           config->vision_tau_s >= 0.0f &&
           config->vision_max_angle_rad >= 0.0f;
}

static void app_tune_handle_line(char* line) {
    char* argv[APP_TUNE_ARG_MAX] = { 0 };
    uint8_t argc;
    bool enable;
    float angle_rad;
    BalanceTuneConfig previous;
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
        else if(app_tune_eq(argv[1], "CTRL")) {
            app_tune_reply_ctrl();
        }
        else if(!app_tune_get_param(argv[1])) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT unknown param=%s", argv[1]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "SET") && argc == 3u) {
        previous = s_pending;
        if(app_tune_set_param(argv[1], argv[2]) &&
           app_tune_valid(&s_pending) &&
           balance_apply_tune(&s_pending) == BALANCE_STATUS_OK) {
            s_status.session_seq++;
            s_status.apply_count++;
            s_status.state = APP_TUNE_APPLIED;
            app_tune_reply("SET %s=%s OK", argv[1], argv[2]);
        }
        else {
            s_pending = previous;
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT param=%s value=%s", argv[1], argv[2]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "CTRL") && argc == 3u) {
        if(!app_tune_parse_bool(argv[2], &enable)) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT ctrl=%s value=%s", argv[1], argv[2]);
            return;
        }
        if(app_tune_set_ctrl(argv[1], enable)) {
            s_status.state = APP_TUNE_APPLIED;
            app_tune_reply_ctrl();
        }
        else {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT ctrl=%s value=%s", argv[1], argv[2]);
        }
        return;
    }
    if(app_tune_eq(argv[0], "DEFAULT") && argc == 1u) {
        previous = s_pending;
        app_tune_defaults(&s_pending);
        if(app_tune_valid(&s_pending) &&
           balance_apply_tune(&s_pending) == BALANCE_STATUS_OK) {
            s_status.session_seq++;
            s_status.apply_count++;
            s_status.state = APP_TUNE_APPLIED;
            app_tune_reply("DEFAULT OK");
        }
        else {
            s_pending = previous;
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("DEFAULT REJECT");
        }
        return;
    }
    if(app_tune_eq(argv[0], "ENABLE") && argc == 2u) {
        if(!app_tune_parse_bool(argv[1], &enable)) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT enable=%s", argv[1]);
            return;
        }
        app_tune_send_cmd(enable, false);
        app_tune_reply("CONTROL %s motor stays enabled", enable ? "ENABLED" : "DISABLED");
        return;
    }
    if(app_tune_eq(argv[0], "MOTOR") && argc == 2u) {
        if(!app_tune_parse_float(argv[1], &angle_rad)) {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT motor angle=%s", argv[1]);
            return;
        }
        if(balance_set_motor_angle(angle_rad) == BALANCE_STATUS_OK) {
            app_tune_reply("MOTOR angle=%.6f rad OK", angle_rad);
        }
        else {
            s_status.reject_count++;
            s_status.state = APP_TUNE_REJECTED;
            app_tune_reply("REJECT disable control first or angle out of range");
        }
        return;
    }
    if(app_tune_eq(argv[0], "CLEAR") && argc == 1u) {
        balance_status = balance_get_status();
        app_tune_send_cmd(balance_status->enable_requested, true);
        app_tune_reply("CLEAR requested");
        return;
    }

    s_status.reject_count++;
    s_status.state = APP_TUNE_REJECTED;
    app_tune_reply("REJECT command type=@HELP");
}

static bool app_tune_set_param(const char* name, const char* text) {
    float value;

    if(!app_tune_parse_float(text, &value)) {
        return false;
    }

    if(app_tune_eq(name, "LEVEL_GAIN"))
        s_pending.level_gain = value;
    else if(app_tune_eq(name, "ACC_GAIN"))
        s_pending.acc_gain = value;
    else if(app_tune_eq(name, "ACC_TAU"))
        s_pending.acc_tau_s = value;
    else if(app_tune_eq(name, "ACC_REVERSE_TAU"))
        s_pending.acc_reverse_tau_s = value;
    else if(app_tune_eq(name, "ACC_REVERSE_THRESHOLD"))
        s_pending.acc_reverse_threshold_m_s2 = value;
    else if(app_tune_eq(name, "VISION_TAU"))
        s_pending.vision_tau_s = value;
    else if(app_tune_eq(name, "VISION_KP"))
        s_pending.vision_kp = value;
    else if(app_tune_eq(name, "VISION_KI"))
        s_pending.vision_ki = value;
    else if(app_tune_eq(name, "VISION_KD"))
        s_pending.vision_kd = value;
    else if(app_tune_eq(name, "VISION_MAX_ANGLE"))
        s_pending.vision_max_angle_rad = value;
    else if(app_tune_eq(name, "MOTOR_VEL"))
        s_pending.motor_vel_rad_s = value;
    else if(app_tune_eq(name, "MOTOR_KP"))
        s_pending.motor_kp = value;
    else if(app_tune_eq(name, "MOTOR_KD"))
        s_pending.motor_kd = value;
    else if(app_tune_eq(name, "MOTOR_TOR"))
        s_pending.motor_tor_nm = value;
    else if(app_tune_eq(name, "TOR_PITCH_GAIN"))
        s_pending.tor_pitch_gain = value;
    else if(app_tune_eq(name, "MOTOR_ZERO"))
        s_pending.motor_zero_rad = value;
    else if(app_tune_eq(name, "MOTOR_MIN"))
        s_pending.motor_min_rad = value;
    else if(app_tune_eq(name, "MOTOR_MAX"))
        s_pending.motor_max_rad = value;
    else
        return false;

    return true;
}

static bool app_tune_set_ctrl(const char* name, bool enable) {
    const BalanceStatus* balance_status = balance_get_status();
    uint8_t bit;
    uint8_t ctrl_mask;

    if(app_tune_eq(name, "LEVEL")) {
        bit = BALANCE_CTRL_LEVEL;
    }
    else if(app_tune_eq(name, "ACC")) {
        bit = BALANCE_CTRL_ACC;
    }
    else if(app_tune_eq(name, "VISION")) {
        bit = BALANCE_CTRL_VISION;
    }
    else {
        return false;
    }

    ctrl_mask = balance_status->ctrl_mask;
    if(enable) {
        ctrl_mask |= bit;
    }
    else {
        ctrl_mask &= (uint8_t)(~bit);
    }
    return balance_set_ctrl_mask(ctrl_mask) == BALANCE_STATUS_OK;
}

static bool app_tune_get_param(const char* name) {
    if(app_tune_eq(name, "LEVEL_GAIN"))
        app_tune_reply("LEVEL_GAIN=%.6f", s_pending.level_gain);
    else if(app_tune_eq(name, "ACC_GAIN"))
        app_tune_reply("ACC_GAIN=%.6f", s_pending.acc_gain);
    else if(app_tune_eq(name, "ACC_TAU"))
        app_tune_reply("ACC_TAU=%.6f s", s_pending.acc_tau_s);
    else if(app_tune_eq(name, "ACC_REVERSE_TAU"))
        app_tune_reply("ACC_REVERSE_TAU=%.6f s", s_pending.acc_reverse_tau_s);
    else if(app_tune_eq(name, "ACC_REVERSE_THRESHOLD"))
        app_tune_reply("ACC_REVERSE_THRESHOLD=%.6f m/s2", s_pending.acc_reverse_threshold_m_s2);
    else if(app_tune_eq(name, "VISION_TAU"))
        app_tune_reply("VISION_TAU=%.6f s", s_pending.vision_tau_s);
    else if(app_tune_eq(name, "VISION_KP"))
        app_tune_reply("VISION_KP=%.6f rad/px", s_pending.vision_kp);
    else if(app_tune_eq(name, "VISION_KI"))
        app_tune_reply("VISION_KI=%.6f rad/(px*s)", s_pending.vision_ki);
    else if(app_tune_eq(name, "VISION_KD"))
        app_tune_reply("VISION_KD=%.6f rad*s/px", s_pending.vision_kd);
    else if(app_tune_eq(name, "VISION_MAX_ANGLE"))
        app_tune_reply("VISION_MAX_ANGLE=%.6f rad", s_pending.vision_max_angle_rad);
    else if(app_tune_eq(name, "MOTOR_VEL"))
        app_tune_reply("MOTOR_VEL=%.6f rad/s", s_pending.motor_vel_rad_s);
    else if(app_tune_eq(name, "MOTOR_KP"))
        app_tune_reply("MOTOR_KP=%.6f", s_pending.motor_kp);
    else if(app_tune_eq(name, "MOTOR_KD"))
        app_tune_reply("MOTOR_KD=%.6f", s_pending.motor_kd);
    else if(app_tune_eq(name, "MOTOR_TOR"))
        app_tune_reply("MOTOR_TOR=%.6f N*m", s_pending.motor_tor_nm);
    else if(app_tune_eq(name, "TOR_PITCH_GAIN"))
        app_tune_reply("TOR_PITCH_GAIN=%.6f N*m/rad", s_pending.tor_pitch_gain);
    else if(app_tune_eq(name, "MOTOR_ZERO"))
        app_tune_reply("MOTOR_ZERO=%.6f rad", s_pending.motor_zero_rad);
    else if(app_tune_eq(name, "MOTOR_MIN"))
        app_tune_reply("MOTOR_MIN=%.6f rad", s_pending.motor_min_rad);
    else if(app_tune_eq(name, "MOTOR_MAX"))
        app_tune_reply("MOTOR_MAX=%.6f rad", s_pending.motor_max_rad);
    else
        return false;

    return true;
}

static void app_tune_get_all(void) {
    app_tune_reply_ctrl();
    app_tune_reply("MOTOR zero=%.4f min=%.4f max=%.4f",
                   s_pending.motor_zero_rad,
                   s_pending.motor_min_rad,
                   s_pending.motor_max_rad);
    app_tune_reply("MIT vel=%.4f kp=%.4f kd=%.4f tor=%.4f pitch_gain=%.4f",
                   s_pending.motor_vel_rad_s,
                   s_pending.motor_kp,
                   s_pending.motor_kd,
                   s_pending.motor_tor_nm,
                   s_pending.tor_pitch_gain);
    app_tune_reply("GAIN level=%.4f acc=%.4f vision_kp=%.4f vision_ki=%.4f vision_kd=%.4f",
                   s_pending.level_gain,
                   s_pending.acc_gain,
                   s_pending.vision_kp,
                   s_pending.vision_ki,
                   s_pending.vision_kd);
    app_tune_reply("FILTER acc_tau=%.4f reverse_tau=%.4f reverse_threshold=%.4f vision_tau=%.4f",
                   s_pending.acc_tau_s,
                   s_pending.acc_reverse_tau_s,
                   s_pending.acc_reverse_threshold_m_s2,
                   s_pending.vision_tau_s);
    app_tune_reply("LIMIT vision_max_angle=%.5f rad",
                   s_pending.vision_max_angle_rad);
}

static void app_tune_reply_ctrl(void) {
    const BalanceStatus* balance_status = balance_get_status();

    app_tune_reply("CTRL mask=0x%02X level=%u acc=%u vision=%u",
                   balance_status->ctrl_mask,
                   (balance_status->ctrl_mask & BALANCE_CTRL_LEVEL) != 0u ? 1u : 0u,
                   (balance_status->ctrl_mask & BALANCE_CTRL_ACC) != 0u ? 1u : 0u,
                   (balance_status->ctrl_mask & BALANCE_CTRL_VISION) != 0u ? 1u : 0u);
}

static void app_tune_send_cmd(bool enable, bool clear_fault) {
    BalanceCommand command;

    s_cmd_seq++;
    command.enable = enable;
    command.clear_fault = clear_fault;
    command.command_seq = s_cmd_seq;
    balance_set_command(&command);
}

static void app_tune_print_status(void) {
    const BalanceStatus* balance_status = balance_get_status();

    app_tune_reply("STATUS tune=%s seq=%u rx=%lu apply=%lu reject=%lu overflow=%lu",
                   app_tune_state_str(s_status.state),
                   s_status.session_seq,
                   (unsigned long)s_status.rx_line_count,
                   (unsigned long)s_status.apply_count,
                   (unsigned long)s_status.reject_count,
                   (unsigned long)s_status.overflow_count);
    app_tune_reply("STATUS balance=%s enable=%u loop=%s vis=%u motor=%u fault=0x%08lX",
                   balance_state_str(balance_status->state),
                   balance_status->enable_requested ? 1u : 0u,
                   balance_status->ball_loop_closed ? "CLOSED" : "OPEN",
                   balance_status->vision_valid ? 1u : 0u,
                   balance_status->motor_enabled ? 1u : 0u,
                   (unsigned long)balance_status->fault_flags);
    app_tune_reply_ctrl();
}

static void app_tune_print_help(void) {
    app_tune_reply("CMD @GET ALL | @GET name | @SET name value | @DEFAULT");
    app_tune_reply("CMD @CTRL LEVEL|ACC|VISION 0|1");
    app_tune_reply("CMD @ENABLE 0|1 | @MOTOR angle_rad | @CLEAR | @STATUS");
    app_tune_reply("PARAM MOTOR_ZERO MOTOR_MIN MOTOR_MAX MOTOR_VEL MOTOR_KP MOTOR_KD MOTOR_TOR TOR_PITCH_GAIN");
    app_tune_reply("PARAM LEVEL_GAIN ACC_GAIN ACC_TAU ACC_REVERSE_TAU ACC_REVERSE_THRESHOLD");
    app_tune_reply("PARAM VISION_TAU VISION_KP VISION_KI VISION_KD VISION_MAX_ANGLE");
    app_tune_reply("VISION input unit px and px/s");
    app_tune_reply("DIR and IMU axis stay in app_config.h");
}
