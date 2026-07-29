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

/**
 * @brief BalanceStatusCode 枚举类型
 */
typedef enum {
    BALANCE_STATUS_OK = 0,
    BALANCE_STATUS_INVALID_PARAM,
    BALANCE_STATUS_DEPENDENCY_MISSING,
} BalanceStatusCode;

/**
 * @brief BalanceState 枚举类型
 */
typedef enum {
    BALANCE_STATE_INIT = 0,
    BALANCE_STATE_CALIB,
    BALANCE_STATE_READY,
    BALANCE_STATE_ACTIVE,
    BALANCE_STATE_FAULT,
} BalanceState;

/**
 * @brief BalanceMode 枚举类型
 */
typedef enum {
    BALANCE_MODE_HOLD_CENTER = 0,
    BALANCE_MODE_HOLD_POS,
    BALANCE_MODE_SWEEP_TEST,
} BalanceMode;

/**
 * @brief BalanceFault 枚举类型
 */
typedef enum {
    BALANCE_FAULT_NONE = 0u,
    BALANCE_FAULT_IMU_TIMEOUT = (1u << 0),
    BALANCE_FAULT_MOTOR_TIMEOUT = (1u << 1),
    BALANCE_FAULT_MOTOR_ERROR = (1u << 2),
    BALANCE_FAULT_MOTOR_WRITE = (1u << 3),
    BALANCE_FAULT_DATA_INVALID = (1u << 4),
} BalanceFault;

/**
 * @brief BalanceAxis 枚举类型
 */
typedef enum {
    BALANCE_AXIS_X = 0,
    BALANCE_AXIS_Y,
    BALANCE_AXIS_Z,
} BalanceAxis;

/**
 * @brief BalanceVisionSample 结构体类型
 */
typedef struct {
    uint32_t stamp_ms;
    uint32_t capture_ms;
    uint16_t frame_seq;
    float pos_m;
    uint8_t quality;
    bool valid;
} BalanceVisionSample;

/**
 * @brief BalanceCommand 结构体类型
 */
typedef struct {
    bool enable;
    bool clear_fault;
    BalanceMode mode;
    float target_m;
    uint16_t command_seq;
} BalanceCommand;

/**
 * @brief BalancePortOps 结构体类型
 */
typedef struct {
    uint32_t (*now_ms)(void);
} BalancePortOps;

/**
 * @brief BalanceConfig 结构体类型
 */
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

/**
 * @brief BalanceStatus 结构体类型
 */
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

/**
 * @brief balance_init 接口函数
 * @param config 配置结构体指针
 * @return 状态码
 */
BalanceStatusCode balance_init(const BalanceConfig* config);
/**
 * @brief balance_process 接口函数
 * @param dt_s 周期时间，单位 s
 */
void balance_process(float dt_s);
/**
 * @brief balance_set_vision 接口函数
 * @param sample 样本数据指针
 */
void balance_set_vision(const BalanceVisionSample* sample);
/**
 * @brief balance_set_command 接口函数
 * @param command 命令数据指针
 */
void balance_set_command(const BalanceCommand* command);
/**
 * @brief balance_on_motor_feedback 接口函数
 * @param feedback 反馈数据指针
 * @param stamp_ms 时间戳，单位 ms
 */
void balance_on_motor_feedback(const BusMotorFeedback* feedback, uint32_t stamp_ms);
/**
 * @brief balance_get_status 接口函数
 * @return 结果指针
 */
const BalanceStatus* balance_get_status(void);
/**
 * @brief balance_state_str 接口函数
 * @param state 状态枚举值
 * @return 字符串指针
 */
const char* balance_state_str(BalanceState state);
/**
 * @brief balance_mode_str 接口函数
 * @param mode 工作模式
 * @return 字符串指针
 */
const char* balance_mode_str(BalanceMode mode);

#endif
