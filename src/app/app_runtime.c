/**
 * @file app_runtime.c
 * @brief 滚球平衡应用超级循环实现
 */

#include "app_runtime.h"

#include "binary_frame.h"
#include "log.h"
#include "rgb_led/rgb_led.h"
#include "vision_comms.h"

#include <stdint.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define APP_RUNTIME_DT_S 0.002f
#define APP_RUNTIME_STATUS_DIV 50u
#define APP_RUNTIME_RGB_DIV 50u
#define APP_RUNTIME_LOG_DIV 500u

// ! ========================= 变 量 声 明 ========================= ! //

static uint16_t s_status_tick = 0u;
static uint16_t s_rgb_tick = 0u;
static uint16_t s_log_tick = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief app_runtime_send_status 内部辅助函数
 */
static void app_runtime_send_status(void);
/**
 * @brief app_runtime_update_rgb 内部辅助函数
 */
static void app_runtime_update_rgb(void);
/**
 * @brief app_runtime_log_status 内部辅助函数
 */
static void app_runtime_log_status(void);
/**
 * @brief app_runtime_make_status_flags 内部辅助函数
 * @param status 状态值
 * @return 返回值
 */
static uint8_t app_runtime_make_status_flags(const BalanceStatus* status);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_runtime_init(void) {
    s_status_tick = 0u;
    s_rgb_tick = 0u;
    s_log_tick = 0u;
}

void app_runtime_process(void) {
    balance_process(APP_RUNTIME_DT_S);

    s_status_tick++;
    if(s_status_tick >= APP_RUNTIME_STATUS_DIV) {
        s_status_tick = 0u;
        app_runtime_send_status();
    }

    s_rgb_tick++;
    if(s_rgb_tick >= APP_RUNTIME_RGB_DIV) {
        s_rgb_tick = 0u;
        app_runtime_update_rgb();
    }

    s_log_tick++;
    if(s_log_tick >= APP_RUNTIME_LOG_DIV) {
        s_log_tick = 0u;
        app_runtime_log_status();
    }
}

void app_runtime_background(void) {
    BalanceVisionSample sample;
    BalanceCommand command;

    vision_comms_process();

    while(vision_comms_take_ball(&sample)) {
        balance_set_vision(&sample);
    }

    while(vision_comms_take_command(&command)) {
        balance_set_command(&command);
    }
}

const BalanceStatus* app_runtime_get_status(void) {
    return balance_get_status();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void app_runtime_send_status(void) {
    const BalanceStatus* status = balance_get_status();
    VisionMcuStatus packet;

    packet.stamp_ms = status->stamp_ms;
    packet.ball_mm = binary_frame_m_to_mm_i16(status->ball_pos_m);
    packet.target_mm = binary_frame_m_to_mm_i16(status->target_m);
    packet.motor_angle_mrad = binary_frame_rad_to_mrad_i16(status->motor_cmd_rad);
    packet.pitch_mrad = binary_frame_rad_to_mrad_i16(status->pitch_rad);
    packet.acc_mm_s2 = binary_frame_m_to_mm_i16(status->acc_m_s2);
    packet.state = (uint8_t)status->state;
    packet.mode = (uint8_t)status->mode;
    packet.flags = app_runtime_make_status_flags(status);
    packet.motor_error = status->motor_error;
    packet.fault_flags = (uint16_t)(status->fault_flags & 0xFFFFu);
    (void)vision_comms_send_status(&packet);
}

static void app_runtime_update_rgb(void) {
    const BalanceStatus* status = balance_get_status();
    uint8_t r = 0u;
    uint8_t g = 0u;
    uint8_t b = 0u;

    switch(status->state) {
        case BALANCE_STATE_CALIB:
            b = 32u;
            break;

        case BALANCE_STATE_READY:
            r = 32u;
            g = 20u;
            break;

        case BALANCE_STATE_ACTIVE:
            if(status->vision_valid) {
                g = 32u;
            }
            else {
                g = 20u;
                b = 32u;
            }
            break;

        case BALANCE_STATE_FAULT:
            r = 48u;
            break;

        case BALANCE_STATE_INIT:
        default:
            r = 16u;
            b = 16u;
            break;
    }

    if(rgb_led.fill(r, g, b) == RGB_LED_STATUS_OK) {
        (void)rgb_led.show();
    }
}

static void app_runtime_log_status(void) {
    const BalanceStatus* status = balance_get_status();
    VisionCommsStats stats;

    vision_comms_get_stats(&stats);
    log_info("BAL state=%s mode=%s ball=%.3f target=%.3f pitch=%.3f acc=%.3f mcmd=%.3f mpos=%.3f mon=%u men=%u merr=0x%X vis=%u fault=0x%08lX rx=%lu crc=%lu",
             balance_state_str(status->state),
             balance_mode_str(status->mode),
             status->ball_pos_m,
             status->target_m,
             status->pitch_rad,
             status->acc_m_s2,
             status->motor_cmd_rad,
             status->motor_pos_rad,
             status->motor_online ? 1u : 0u,
             status->motor_enabled ? 1u : 0u,
             status->motor_error,
             status->vision_valid ? 1u : 0u,
             (unsigned long)status->fault_flags,
             (unsigned long)stats.rx_frame_count,
             (unsigned long)stats.rx_bad_crc_count);
}

static uint8_t app_runtime_make_status_flags(const BalanceStatus* status) {
    uint8_t flags = 0u;

    if(status->imu_online) {
        flags |= VISION_STATUS_FLAG_IMU_ONLINE;
    }
    if(status->motor_online) {
        flags |= VISION_STATUS_FLAG_MOTOR_ONLINE;
    }
    if(vision_comms_is_online()) {
        flags |= VISION_STATUS_FLAG_VISION_ONLINE;
    }
    if(status->vision_valid) {
        flags |= VISION_STATUS_FLAG_VISION_VALID;
    }
    if(status->state == BALANCE_STATE_FAULT) {
        flags |= VISION_STATUS_FLAG_FAULT;
    }

    return flags;
}
