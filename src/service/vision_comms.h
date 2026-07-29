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

typedef enum {
    VISION_COMMS_STATUS_OK = 0,
    VISION_COMMS_STATUS_INVALID_PARAM,
    VISION_COMMS_STATUS_DEPENDENCY_MISSING,
} VisionCommsStatus;

typedef struct {
    bool (*write)(const char* data, uint32_t len);
    uint32_t (*now_ms)(void);
} VisionCommsPortOps;

typedef struct {
    VisionCommsPortOps port_ops;
    uint32_t online_timeout_ms;
} VisionCommsConfig;

typedef struct {
    uint32_t uptime_ms;
    uint16_t fps_x10;
    uint16_t status;
} VisionHeartbeat;

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

VisionCommsStatus vision_comms_init(const VisionCommsConfig* config);
void vision_comms_on_rx_byte(uint8_t data);
void vision_comms_process(void);
bool vision_comms_take_ball(BalanceVisionSample* sample);
bool vision_comms_take_command(BalanceCommand* command);
bool vision_comms_get_heartbeat(VisionHeartbeat* heartbeat);
bool vision_comms_is_online(void);
bool vision_comms_send_status(const VisionMcuStatus* status);
void vision_comms_get_stats(VisionCommsStats* stats);

#endif
