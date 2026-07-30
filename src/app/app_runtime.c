/**
 * @file app_runtime.c
 * @brief 滚球平衡应用超级循环实现
 */

#include "app_runtime.h"

#include "app_tune.h"
#include "binary_frame.h"
#include "delay.h"
#include "log.h"
#include "rgb_led/rgb_led.h"
#include "vision_comms.h"

#include <stdint.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define APP_RUNTIME_DEFAULT_DT_S 0.002f
#define APP_RUNTIME_MAX_DT_S 0.020f
#define APP_RUNTIME_STATUS_DIV 50u
#define APP_RUNTIME_RGB_DIV 50u
#define APP_RUNTIME_LOG_DIV 500u

// ! ========================= 变 量 声 明 ========================= ! //

static uint16_t s_status_tick = 0u;
static uint16_t s_rgb_tick = 0u;
static uint16_t s_log_tick = 0u;
static uint32_t s_process_last_us = 0u;

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

bool app_runtime_init(void) {
    s_status_tick = 0u;
    s_rgb_tick = 0u;
    s_log_tick = 0u;
    s_process_last_us = 0u;
    return app_tune_init();
}

void app_runtime_process(void) {
    uint32_t now_us;
    uint32_t elapsed_us;
    float dt_s;

    now_us = delay_now_us();
    if(s_process_last_us == 0u) {
        dt_s = APP_RUNTIME_DEFAULT_DT_S;
    }
    else {
        elapsed_us = now_us - s_process_last_us;
        dt_s = (float)elapsed_us * 0.000001f;
        if(dt_s <= 0.0f || dt_s > APP_RUNTIME_MAX_DT_S) {
            dt_s = APP_RUNTIME_DEFAULT_DT_S;
        }
    }
    s_process_last_us = now_us;
    balance_process(dt_s);

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

    app_tune_process();
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
    packet.offset_mm = binary_frame_m_to_mm_i16(status->vision_offset_m);
    packet.offset_vel_mm_s = binary_frame_m_to_mm_i16(status->vision_offset_vel_m_s);
    packet.motor_angle_mrad = binary_frame_rad_to_mrad_i16(status->motor_cmd_rad);
    packet.pitch_mrad = binary_frame_rad_to_mrad_i16(status->pitch_rad);
    packet.acc_mm_s2 = binary_frame_m_to_mm_i16(status->acc_m_s2);
    packet.state = (uint8_t)status->state;
    packet.reserved = 0u;
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
    const AppTuneStatus* tune = app_tune_get_status();
    BalanceTuneConfig config = { 0 };
    VisionCommsStats stats;

    (void)balance_get_tune(&config);
    vision_comms_get_stats(&stats);
    log_info("--------------------");
    log_info("[01 状态 STATE] 模式=%s 总控=%s 闭环=%s MERR=0x%X FAULT=0x%08lX",
             balance_state_str(status->state),
             status->enable_requested ? "开" : "关",
             status->ball_loop_closed ? "闭合" : "断开",
             status->motor_error,
             (unsigned long)status->fault_flags);
    log_info("[02 在线 ONLINE] IMU=%s MOTOR=%s/%s VISION=%s/%s RX=%lu CRC=%lu",
             status->imu_online ? "在线" : "离线",
             status->motor_online ? "在线" : "离线",
             status->motor_enabled ? "使能" : "失能",
             status->vision_online ? "在线" : "离线",
             status->vision_valid ? "有效" : "无效",
             (unsigned long)stats.rx_frame_count,
             (unsigned long)stats.rx_bad_crc_count);
    log_info("[03 通道 CTRL] LEVEL=%s ACC=%s VISION=%s 掩码=0x%02X",
             (status->ctrl_mask & BALANCE_CTRL_LEVEL) != 0u ? "开" : "关",
             (status->ctrl_mask & BALANCE_CTRL_ACC) != 0u ? "开" : "关",
             (status->ctrl_mask & BALANCE_CTRL_VISION) != 0u ? "开" : "关",
             status->ctrl_mask);
    log_info("[04 姿态 IMU] pitch原=%.4f rel=%.4f acc原=%.4f lin=%.4f filt=%.4f",
             status->pitch_raw_rad,
             status->pitch_rad,
             status->acc_raw_m_s2,
             status->acc_linear_m_s2,
             status->acc_m_s2);
    log_info("[05 视觉 VISION] offset原=%.4f filt=%.4f vel=%.4f P=%.4f I=%.4f D=%.4f out=%.4f",
             status->vision_offset_raw_m,
             status->vision_offset_m,
             status->vision_offset_vel_m_s,
             status->vision_p_rad,
             status->vision_i_rad,
             status->vision_d_rad,
             status->vision_angle_rad);
    log_info("[06 输出 OUTPUT] LEVEL=%.4f ACC=%.4f VIS=%.4f raw=%.4f cmd=%.4f SAT=%s",
             status->level_angle_rad,
             status->acc_angle_rad,
             status->vision_angle_rad,
             status->motor_cmd_raw_rad,
             status->motor_cmd_rad,
             status->motor_cmd_saturated ? "是" : "否");
    log_info("[07 电机 MIT] pos=%.4f err=%.4f vel目标=%.4f 实际=%.4f KP=%.4f KD=%.4f tor=%.4f",
             status->motor_cmd_rad,
             status->motor_cmd_rad - status->motor_pos_rad,
             config.motor_vel_rad_s,
             status->motor_spd_rad_s,
             config.motor_kp,
             config.motor_kd,
             status->motor_tor_ff_nm);
    log_info("[08 调参 TUNE] 状态=%s 序号=%u RX=%lu 应用=%lu 拒绝=%lu 溢出=%lu",
             app_tune_state_str(tune->state),
             tune->session_seq,
             (unsigned long)tune->rx_line_count,
             (unsigned long)tune->apply_count,
             (unsigned long)tune->reject_count,
             (unsigned long)tune->overflow_count);
}

static uint8_t app_runtime_make_status_flags(const BalanceStatus* status) {
    uint8_t flags = 0u;

    if(status->imu_online) {
        flags |= VISION_STATUS_FLAG_IMU_ONLINE;
    }
    if(status->motor_online) {
        flags |= VISION_STATUS_FLAG_MOTOR_ONLINE;
    }
    if(status->vision_online) {
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
