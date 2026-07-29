#ifndef _vision_comms_h_
#define _vision_comms_h_

/**
 * @file vision_comms.h
 * @brief 视觉相机与 MCU 的二进制通信服务
 */

#include "balance.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define VISION_MSG_HEARTBEAT 0x50u
#define VISION_MSG_BALL 0x51u
#define VISION_MSG_COMMAND 0x52u
#define VISION_MSG_MCU_STATUS 0x60u
#define VISION_MSG_MCU_ACK 0x61u

#define VISION_BALL_FLAG_VALID 0x01u
#define VISION_BALL_FLAG_OCCLUDED 0x02u
#define VISION_CMD_FLAG_CLEAR_FAULT 0x0001u

#define VISION_STATUS_FLAG_IMU_ONLINE 0x01u
#define VISION_STATUS_FLAG_MOTOR_ONLINE 0x02u
#define VISION_STATUS_FLAG_VISION_ONLINE 0x04u
#define VISION_STATUS_FLAG_VISION_VALID 0x08u
#define VISION_STATUS_FLAG_FAULT 0x10u

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief VisionCommsStatus 枚举类型
 */
typedef enum {
    VISION_COMMS_STATUS_OK = 0,
    VISION_COMMS_STATUS_INVALID_PARAM,
    VISION_COMMS_STATUS_DEPENDENCY_MISSING,
} VisionCommsStatus;

/**
 * @brief VisionCommsPortOps 结构体类型
 */
typedef struct {
    bool (*write)(const char* data, uint32_t len);
    uint32_t (*now_ms)(void);
} VisionCommsPortOps;

/**
 * @brief VisionCommsConfig 结构体类型
 */
typedef struct {
    VisionCommsPortOps port_ops;
    uint32_t online_timeout_ms;
} VisionCommsConfig;

/**
 * @brief VisionHeartbeat 结构体类型
 */
typedef struct {
    uint32_t uptime_ms;
    uint16_t fps_x10;
    uint16_t status;
} VisionHeartbeat;

/**
 * @brief VisionMcuStatus 结构体类型
 */
typedef struct {
    uint32_t stamp_ms;
    int16_t ball_mm;
    int16_t target_mm;
    int16_t motor_angle_mrad;
    int16_t pitch_mrad;
    int16_t acc_mm_s2;
    uint8_t state;
    uint8_t mode;
    uint8_t flags;
    uint8_t motor_error;
    uint16_t fault_flags;
} VisionMcuStatus;

/**
 * @brief VisionCommsStats 结构体类型
 */
typedef struct {
    uint32_t rx_frame_count;
    uint32_t rx_bad_crc_count;
    uint32_t rx_bad_len_count;
    uint32_t rx_overflow_count;
    uint32_t rx_unknown_count;
    uint32_t tx_frame_count;
    uint32_t tx_fail_count;
    uint8_t last_msg_id;
    uint8_t last_seq;
} VisionCommsStats;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief vision_comms_init 接口函数
 * @param config 配置结构体指针
 * @return 状态码
 */
VisionCommsStatus vision_comms_init(const VisionCommsConfig* config);
/**
 * @brief vision_comms_on_rx_byte 接口函数
 * @param data 数据缓冲区指针
 */
void vision_comms_on_rx_byte(uint8_t data);
/**
 * @brief vision_comms_process 接口函数
 */
void vision_comms_process(void);
/**
 * @brief vision_comms_take_ball 接口函数
 * @param sample 样本数据指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool vision_comms_take_ball(BalanceVisionSample* sample);
/**
 * @brief vision_comms_take_command 接口函数
 * @param command 命令数据指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool vision_comms_take_command(BalanceCommand* command);
/**
 * @brief vision_comms_get_heartbeat 接口函数
 * @param heartbeat 参数值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool vision_comms_get_heartbeat(VisionHeartbeat* heartbeat);
/**
 * @brief vision_comms_is_online 接口函数
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool vision_comms_is_online(void);
/**
 * @brief vision_comms_send_status 接口函数
 * @param status 状态值
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool vision_comms_send_status(const VisionMcuStatus* status);
/**
 * @brief vision_comms_get_stats 接口函数
 * @param stats 参数值
 */
void vision_comms_get_stats(VisionCommsStats* stats);

#endif
