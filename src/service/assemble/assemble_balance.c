/**
 * @file assemble_balance.c
 * @brief 滚球平衡控制参数装配
 */

#include "assemble.h"

#include "../../app/app_config.h"
#include "balance.h"
#include "delay.h"
#include "log.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_balance(void) {
    BalanceConfig config = {
        .port_ops = {
            .now_ms = delay_now_ms,
        },
        .motor_id = APP_CFG_MOTOR_ID,
        .motor_zero_rad = APP_CFG_MOTOR_ZERO_RAD,
        .motor_min_rad = APP_CFG_MOTOR_MIN_RAD,
        .motor_max_rad = APP_CFG_MOTOR_MAX_RAD,
        .motor_speed_rad_s = APP_CFG_MOTOR_SPEED_RAD_S,
        .motor_rate_rad_s = APP_CFG_MOTOR_RATE_RAD_S,
        .motor_sign = APP_CFG_MOTOR_SIGN,
        .acc_axis = APP_CFG_ACC_AXIS,
        .acc_sign = APP_CFG_ACC_SIGN,
        .pitch_sign = APP_CFG_PITCH_SIGN,
        .ball_sign = APP_CFG_BALL_SIGN,
        .level_gain = APP_CFG_LEVEL_GAIN,
        .acc_gain = APP_CFG_ACC_GAIN,
        .acc_tau_s = APP_CFG_ACC_TAU_S,
        .pos_tau_s = APP_CFG_POS_TAU_S,
        .vel_tau_s = APP_CFG_VEL_TAU_S,
        .pos_pid = {
            .mode = APP_CFG_POS_MODE,
            .features = APP_CFG_POS_FEATURES,
            .kp = APP_CFG_POS_KP,
            .ki = APP_CFG_POS_KI,
            .kd = APP_CFG_POS_KD,
            .max_out = APP_CFG_POS_MAX_OUT_RAD,
            .integral_separation = APP_CFG_POS_INT_SEP_M,
            .dead_band = APP_CFG_POS_DEAD_BAND_M,
            .diff_filter_alpha = APP_CFG_POS_DIFF_ALPHA,
            .output_max_rate = APP_CFG_POS_OUT_RATE_RAD_S,
        },
        .vision_angle_max_rad = APP_CFG_VISION_ANGLE_MAX_RAD,
        .vision_decay_tau_s = APP_CFG_VISION_DECAY_TAU_S,
        .target_limit_m = APP_CFG_TARGET_LIMIT_M,
        .vision_quality_min = APP_CFG_VISION_QUALITY_MIN,
        .vision_timeout_ms = APP_CFG_VISION_TIMEOUT_MS,
        .imu_timeout_ms = APP_CFG_IMU_TIMEOUT_MS,
        .motor_timeout_ms = APP_CFG_MOTOR_TIMEOUT_MS,
        .sweep_pos_m = APP_CFG_SWEEP_POS_M,
        .sweep_err_m = APP_CFG_SWEEP_ERR_M,
        .sweep_vel_m_s = APP_CFG_SWEEP_VEL_M_S,
        .sweep_stable_ms = APP_CFG_SWEEP_STABLE_MS,
        .calib_settle_ms = APP_CFG_CALIB_SETTLE_MS,
        .calib_samples = APP_CFG_CALIB_SAMPLES,
        .ctrl_mask = APP_CFG_CTRL_MASK,
        .auto_enable = APP_CFG_AUTO_ENABLE,
    };

    if(balance_init(&config) != BALANCE_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("BALANCE control ready");
    return SYSTEM_STATUS_OK;
}
