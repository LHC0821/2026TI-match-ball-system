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
 * @brief DM-4340 V3 MIT 参数范围
 */
#define BALANCE_MOTOR_VEL_MAX_RAD_S 10.0f
#define BALANCE_MOTOR_KP_MAX 500.0f
#define BALANCE_MOTOR_KD_MAX 5.0f
#define BALANCE_MOTOR_TOR_MAX_NM 28.0f

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
 * 视觉端发送的是球相对于期望位置的像素偏移量，MCU 只把零偏移作为控制目标
 */
typedef struct {
    uint32_t stamp_ms;
    uint32_t capture_ms;
    uint16_t frame_seq;
    float offset_px;
    float velocity_px_s;
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
    float motor_vel_rad_s;
    float motor_kp;
    float motor_kd;
    float motor_tor_nm;
    float tor_pitch_gain;
    float motor_sign;

    BalanceAxis acc_axis;
    float acc_sign;
    float pitch_sign;
    float ball_sign;
    float level_gain;
    float acc_gain;

    float acc_tau_s;
    float acc_reverse_tau_s;
    float acc_reverse_threshold_m_s2;
    float vision_tau_s;
    float vision_kp;
    float vision_ki;
    float vision_kd;
    float vision_max_angle_rad;

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
    float motor_vel_rad_s;
    float motor_kp;
    float motor_kd;
    float motor_tor_nm;
    float tor_pitch_gain;
    float level_gain;
    float acc_gain;
    float acc_tau_s;
    float acc_reverse_tau_s;
    float acc_reverse_threshold_m_s2;
    float vision_tau_s;
    float vision_kp;
    float vision_ki;
    float vision_kd;
    float vision_max_angle_rad;
} BalanceTuneConfig;

/**
 * @brief BalanceStatus 结构体类型
 */
typedef struct {
    uint32_t stamp_ms;
    BalanceState state;
    uint32_t fault_flags;

    float vision_offset_raw_px;
    float vision_offset_px;
    float vision_offset_vel_px_s;
    float pitch_raw_rad;
    float pitch_rad;
    float acc_raw_m_s2;
    float acc_linear_m_s2;
    float acc_m_s2;
    float acc_tau_s;

    float level_angle_rad;
    float acc_angle_rad;
    float vision_raw_angle_rad;
    float vision_angle_rad;
    float vision_limit_angle_rad;
    float vision_p_rad;
    float vision_i_rad;
    float vision_d_rad;
    float motor_cmd_raw_rad;
    float motor_cmd_rad;
    float motor_pos_rad;
    float motor_spd_rad_s;
    float motor_tor_ff_nm;
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
    bool acc_reverse_filter;
    bool vision_angle_saturated;
    bool motor_cmd_saturated;
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
 * @brief 在滚球控制失能时直接设置电机 MIT 目标角度
 * @param angle_rad 目标角度，单位 rad
 * @return 状态码
 */
BalanceStatusCode balance_set_motor_angle(float angle_rad);
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
