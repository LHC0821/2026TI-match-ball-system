/**
 * @file assemble_balance.c
 * @brief 滚球平衡控制参数装配
 */

#include "assemble.h"

#include "balance.h"
#include "delay.h"
#include "log.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_balance(void) {
    BalanceConfig config = {
        .port_ops = {
            .now_ms = delay_now_ms,
        },
        .motor_id = 0x01u,
        .motor_zero_rad = 0.0f,
        .motor_min_rad = -0.35f,
        .motor_max_rad = 0.35f,
        .motor_speed_rad_s = 1.0f,
        .motor_rate_rad_s = 1.5f,
        .motor_sign = 1.0f,
        .acc_axis = BALANCE_AXIS_X,
        .acc_sign = 1.0f,
        .pitch_sign = 1.0f,
        .ball_sign = 1.0f,
        .level_gain = -1.0f,
        .acc_gain = -1.0f,
        .acc_tau_s = 0.08f,
        .pos_tau_s = 0.03f,
        .vel_tau_s = 0.08f,
        .pos_pid = {
            .mode = PID_MODE_PID,
            .features = PID_FEAT_OUTPUT_LIMIT | PID_FEAT_INTEGRAL_SEP | PID_FEAT_DEADBAND | PID_FEAT_DIFF_FILTER | PID_FEAT_DIFF_ON_MEAS | PID_FEAT_ANTI_WINDUP | PID_FEAT_OUTPUT_RATE_LIMIT,
            .kp = 1.8f,
            .ki = 0.05f,
            .kd = 0.35f,
            .max_out = 0.12f,
            .integral_separation = 0.02f,
            .dead_band = 0.002f,
            .diff_filter_alpha = 0.20f,
            .output_max_rate = 1.0f,
        },
        .vision_angle_max_rad = 0.12f,
        .target_limit_m = 0.10f,
        .vision_quality_min = 40u,
        .vision_timeout_ms = 200u,
        .imu_timeout_ms = 50u,
        .motor_timeout_ms = 300u,
        .calib_settle_ms = 4000u,
        .calib_samples = 250u,
        .auto_enable = true,
    };

    if(balance_init(&config) != BALANCE_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("BALANCE control ready");
    return SYSTEM_STATUS_OK;
}
