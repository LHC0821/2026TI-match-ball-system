#ifndef _balance_h_
#define _balance_h_

/**
 * @file balance.h
 * @brief 车载滚球摆杆平衡控制服务
 */

#include "bus_motor/bus_motor.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    BALANCE_STATUS_OK = 0,
    BALANCE_STATUS_INVALID_PARAM,
    BALANCE_STATUS_DEPENDENCY_MISSING,
} BalanceStatusCode;

typedef enum {
    BALANCE_STATE_INIT = 0,
    BALANCE_STATE_CALIB,
    BALANCE_STATE_READY,
    BALANCE_STATE_ACTIVE,
    BALANCE_STATE_FAULT,
} BalanceState;

typedef enum {
    BALANCE_MODE_HOLD_CENTER = 0,
    BALANCE_MODE_HOLD_POS,
    BALANCE_MODE_SWEEP_TEST,
} BalanceMode;

typedef enum {
    BALANCE_FAULT_NONE = 0u,
    BALANCE_FAULT_IMU_TIMEOUT = (1u << 0),
    BALANCE_FAULT_MOTOR_TIMEOUT = (1u << 1),
    BALANCE_FAULT_MOTOR_ERROR = (1u << 2),
    BALANCE_FAULT_MOTOR_WRITE = (1u << 3),
    BALANCE_FAULT_DATA_INVALID = (1u << 4),
} BalanceFault;

typedef enum {
    BALANCE_AXIS_X = 0,
    BALANCE_AXIS_Y,
    BALANCE_AXIS_Z,
} BalanceAxis;

typedef struct {
    uint32_t stamp_ms;
    uint32_t capture_ms;
    uint16_t frame_seq;
    float pos_m;
    uint8_t quality;
    bool valid;
} BalanceVisionSample;

typedef struct {
    bool enable;
    bool clear_fault;
    BalanceMode mode;
    float target_m;
    uint16_t command_seq;
} BalanceCommand;

typedef struct {
    uint32_t (*now_ms)(void);
} BalancePortOps;

typedef struct {
    BalancePortOps port_ops;

    uint16_t motor_id;
    float motor_zero_rad;
    float motor_min_rad;
    float motor_max_rad;
    float motor_speed_rad_s;
    float motor_rate_rad_s;
    float motor_sign;

    BalanceAxis acc_axis;
    float acc_sign;
    float pitch_sign;
    float ball_sign;
    float level_gain;
    float acc_gain;

    float acc_tau_s;
    float pos_tau_s;
    float vel_tau_s;
    PidConfig pos_pid;

    float vision_angle_max_rad;
    float target_limit_m;
    uint8_t vision_quality_min;
    uint32_t vision_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t motor_timeout_ms;

    uint32_t calib_settle_ms;
    uint16_t calib_samples;
    bool auto_enable;
} BalanceConfig;

typedef struct {
    uint32_t stamp_ms;
    BalanceState state;
    BalanceMode mode;
    uint32_t fault_flags;

    float target_m;
    float ball_raw_m;
    float ball_pos_m;
    float ball_vel_m_s;
    float pitch_rad;
    float acc_m_s2;

    float level_angle_rad;
    float acc_angle_rad;
    float vision_angle_rad;
    float motor_cmd_rad;
    float motor_pos_rad;
    float motor_spd_rad_s;
    uint32_t vision_capture_ms;
    uint16_t vision_frame_seq;
    uint16_t command_seq;
    uint8_t motor_error;

    bool imu_online;
    bool motor_online;
    bool motor_enabled;
    bool vision_online;
    bool vision_valid;
} BalanceStatus;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BalanceStatusCode balance_init(const BalanceConfig* config);
void balance_process(float dt_s);
void balance_set_vision(const BalanceVisionSample* sample);
void balance_set_command(const BalanceCommand* command);
void balance_on_motor_feedback(const BusMotorFeedback* feedback, uint32_t stamp_ms);
const BalanceStatus* balance_get_status(void);
const char* balance_state_str(BalanceState state);
const char* balance_mode_str(BalanceMode mode);

#endif
