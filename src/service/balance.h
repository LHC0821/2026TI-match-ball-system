#ifndef _balance_h_
#define _balance_h_

/**
 * @file balance.h
 * @brief 车载滚球摆杆平衡控制服务
 */

#include "bus_motor/bus_motor.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief DM-4340 V3 位置速度指令允许的最高配速，单位 rad/s
 */
#define BALANCE_MOTOR_SPEED_MAX_RAD_S 10.0f

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief BalanceStatusCode 枚举类型
 */
typedef enum {
    BALANCE_STATUS_OK = 0,
    BALANCE_STATUS_INVALID_PARAM,
    BALANCE_STATUS_DEPENDENCY_MISSING,
    BALANCE_STATUS_STATE_DENIED,
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
 * @brief BalanceFault 枚举类型
 */
typedef enum {
    BALANCE_FAULT_NONE = 0u,
    BALANCE_FAULT_MOTOR_ERROR = (1u << 0),
    BALANCE_FAULT_MOTOR_WRITE = (1u << 1),
    BALANCE_FAULT_DATA_INVALID = (1u << 2),
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
 * @brief BalanceCtrl 平衡控制通道位
 */
typedef enum {
    BALANCE_CTRL_NONE = 0u,
    BALANCE_CTRL_LEVEL = (1u << 0),
    BALANCE_CTRL_ACC = (1u << 1),
    BALANCE_CTRL_VISION = (1u << 2),
    BALANCE_CTRL_ALL = 0x07u,
} BalanceCtrl;

/**
 * @brief BalanceVisionSample 结构体类型
 *
 * @details
 * 视觉端发送的是球相对于期望位置的偏移量；MCU 只把零偏移作为控制目标
 */
typedef struct {
    uint32_t stamp_ms;
    uint32_t capture_ms;
    uint16_t frame_seq;
    float offset_m;
    uint8_t quality;
    bool valid;
} BalanceVisionSample;

/**
 * @brief BalanceCommand 结构体类型
 */
typedef struct {
    bool enable;
    bool clear_fault;
    uint16_t command_seq;
} BalanceCommand;

/**
 * @brief BalancePortOps 结构体类型
 */
typedef struct {
    uint32_t (*now_ms)(void);
    /** @brief IMU 校准完成后执行一次电机协议准备 */
    bool (*prepare_motor)(uint16_t id);
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
    /** @brief 每帧位置速度指令固定使用的全程配速，单位 rad/s */
    float motor_speed_rad_s;
    float motor_sign;

    BalanceAxis acc_axis;
    float acc_sign;
    float pitch_sign;
    float ball_sign;
    float level_gain;
    float acc_gain;

    float acc_tau_s;
    float vision_tau_s;
    float vision_kp;
    float vision_kd;

    uint32_t calib_settle_ms;
    uint16_t calib_samples;
    uint8_t ctrl_mask;
    bool auto_enable;
} BalanceConfig;

/**
 * @brief BalanceTuneConfig 结构体类型
 *
 * @details
 * 方向和 IMU 轴属于编译期安装配置，不通过无线调参修改
 */
typedef struct {
    float motor_zero_rad;
    float motor_min_rad;
    float motor_max_rad;
    /** @brief 无线调参设置的位置速度指令全程配速，单位 rad/s */
    float motor_speed_rad_s;
    float level_gain;
    float acc_gain;
    float acc_tau_s;
    float vision_tau_s;
    float vision_kp;
    float vision_kd;
} BalanceTuneConfig;

/**
 * @brief BalanceStatus 结构体类型
 */
typedef struct {
    uint32_t stamp_ms;
    BalanceState state;
    uint32_t fault_flags;

    float vision_offset_raw_m;
    float vision_offset_m;
    float vision_offset_vel_m_s;
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
    uint8_t ctrl_mask;

    bool enable_requested;
    bool imu_online;
    bool motor_online;
    bool motor_enabled;
    bool vision_online;
    bool vision_valid;
    bool ball_loop_closed;
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
 * @brief balance_get_tune 接口函数
 * @param config 配置结构体指针
 * @return 状态码
 */
BalanceStatusCode balance_get_tune(BalanceTuneConfig* config);
/**
 * @brief balance_apply_tune 接口函数
 * @param config 配置结构体指针
 * @return 状态码
 */
BalanceStatusCode balance_apply_tune(const BalanceTuneConfig* config);
/**
 * @brief balance_set_ctrl_mask 接口函数
 * @param ctrl_mask 平衡控制通道位掩码
 * @return 状态码
 */
BalanceStatusCode balance_set_ctrl_mask(uint8_t ctrl_mask);
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

#endif
