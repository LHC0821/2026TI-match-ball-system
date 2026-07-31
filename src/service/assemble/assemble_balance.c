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
            .prepare_motor = assemble_motor_prepare,
        },
        .motor_id = APP_CFG_MOTOR_ID,
        .motor_zero_rad = APP_CFG_MOTOR_ZERO_RAD,
        .motor_min_rad = APP_CFG_MOTOR_MIN_RAD,
        .motor_max_rad = APP_CFG_MOTOR_MAX_RAD,
        .motor_vel_rad_s = APP_CFG_MOTOR_VEL_RAD_S,
        .motor_kp = APP_CFG_MOTOR_KP,
        .motor_kd = APP_CFG_MOTOR_KD,
        .motor_tor_nm = APP_CFG_MOTOR_TOR_NM,
        .tor_pitch_gain = APP_CFG_TOR_PITCH_GAIN,
        .motor_sign = APP_CFG_MOTOR_SIGN,
        .acc_axis = APP_CFG_ACC_AXIS,
        .acc_sign = APP_CFG_ACC_SIGN,
        .pitch_sign = APP_CFG_PITCH_SIGN,
        .ball_sign = APP_CFG_BALL_SIGN,
        .level_gain = APP_CFG_LEVEL_GAIN,
        .acc_gain = APP_CFG_ACC_GAIN,
        .acc_tau_s = APP_CFG_ACC_TAU_S,
        .acc_fast_tau_s = APP_CFG_ACC_FAST_TAU_S,
        .acc_fast_enter_m_s2 = APP_CFG_ACC_FAST_ENTER_M_S2,
        .acc_fast_exit_m_s2 = APP_CFG_ACC_FAST_EXIT_M_S2,
        .vision_tau_s = APP_CFG_VISION_TAU_S,
        .vision_kp = APP_CFG_VISION_KP,
        .vision_ki = APP_CFG_VISION_KI,
        .vision_kd = APP_CFG_VISION_KD,
        .vision_max_angle_rad = APP_CFG_VISION_MAX_ANGLE_RAD,
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
