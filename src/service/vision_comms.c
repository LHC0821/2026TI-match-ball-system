/**
 * @file vision_comms.c
 * @brief 视觉相机与 MCU 的二进制通信服务实现
 */

#include "vision_comms.h"

#include "binary_frame.h"
#include "protocol_parser.h"

#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define VISION_RX_QUEUE_SIZE 256u
#define VISION_PARSER_RING_SIZE 256u
#define VISION_FRAME_BUF_SIZE 64u
#define VISION_TX_BUF_SIZE 64u
#define VISION_ONLINE_TIMEOUT_MS 500u

#define VISION_HEARTBEAT_PAYLOAD_LEN 8u
#define VISION_BALL_PAYLOAD_LEN 12u
#define VISION_COMMAND_PAYLOAD_LEN 8u
#define VISION_STATUS_PAYLOAD_LEN 20u
#define VISION_ACK_PAYLOAD_LEN 4u

// ! ========================= 变 量 声 明 ========================= ! //

static VisionCommsConfig s_config = { 0 };
static bool s_initialized = false;
static volatile uint16_t s_rx_write = 0u;
static volatile uint16_t s_rx_read = 0u;
static uint8_t s_rx_queue[VISION_RX_QUEUE_SIZE] = { 0 };
static uint8_t s_parser_ring_buf[VISION_PARSER_RING_SIZE] = { 0 };
static RingBuf s_parser_ring = { 0 };
static uint8_t s_frame_buf[VISION_FRAME_BUF_SIZE] = { 0 };
static const uint8_t s_frame_header[2] = { BINARY_FRAME_SOF0, BINARY_FRAME_SOF1 };
static FrameParser s_frame_parser = { 0 };
static uint8_t s_tx_buf[VISION_TX_BUF_SIZE] = { 0 };
static uint8_t s_tx_seq = 0u;
static uint32_t s_last_rx_ms = 0u;
static BalanceVisionSample s_ball = { 0 };
static BalanceCommand s_command = { 0 };
static VisionHeartbeat s_heartbeat = { 0 };
static bool s_ball_pending = false;
static bool s_command_pending = false;
static bool s_heartbeat_valid = false;
static VisionCommsStats s_stats = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief vision_comms_now_ms 内部辅助函数
 * @return 返回值
 */
static uint32_t vision_comms_now_ms(void);
/**
 * @brief vision_comms_next_seq 内部辅助函数
 * @return 返回值
 */
static uint8_t vision_comms_next_seq(void);
/**
 * @brief vision_comms_queue_read 内部辅助函数
 * @param data 数据缓冲区指针
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool vision_comms_queue_read(uint8_t* data);
/**
 * @brief vision_comms_send_frame 内部辅助函数
 * @param msg_id 消息 ID
 * @param seq 帧序号
 * @param flags 帧标志位
 * @param payload 负载数据指针
 * @param payload_len 负载长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool vision_comms_send_frame(uint8_t msg_id, uint8_t seq, uint8_t flags,
                                    const uint8_t* payload, uint16_t payload_len);
/**
 * @brief vision_comms_send_ack 内部辅助函数
 * @param msg_id 消息 ID
 * @param seq 帧序号
 * @param status 状态值
 */
static void vision_comms_send_ack(uint8_t msg_id, uint8_t seq, uint8_t status);
/**
 * @brief vision_comms_process_parser 内部辅助函数
 */
static void vision_comms_process_parser(void);
/**
 * @brief vision_comms_process_frame 内部辅助函数
 * @param body 帧体缓冲区
 * @param body_len 帧体长度
 */
static void vision_comms_process_frame(const uint8_t* body, uint16_t body_len);
/**
 * @brief vision_comms_handle_heartbeat 内部辅助函数
 * @param frame 参数值
 */
static void vision_comms_handle_heartbeat(const BinaryFrameView* frame);
/**
 * @brief vision_comms_handle_ball 内部辅助函数
 * @param frame 参数值
 */
static void vision_comms_handle_ball(const BinaryFrameView* frame);
/**
 * @brief vision_comms_handle_command 内部辅助函数
 * @param frame 参数值
 */
static void vision_comms_handle_command(const BinaryFrameView* frame);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

VisionCommsStatus vision_comms_init(const VisionCommsConfig* config) {
    s_initialized = false;

    if(config == 0 || config->port_ops.write == 0 || config->port_ops.now_ms == 0) {
        return VISION_COMMS_STATUS_INVALID_PARAM;
    }

    s_config = *config;
    s_rx_write = 0u;
    s_rx_read = 0u;
    s_tx_seq = 0u;
    s_last_rx_ms = 0u;
    s_ball_pending = false;
    s_command_pending = false;
    s_heartbeat_valid = false;
    memset(&s_ball, 0, sizeof(s_ball));
    memset(&s_command, 0, sizeof(s_command));
    memset(&s_heartbeat, 0, sizeof(s_heartbeat));
    memset(&s_stats, 0, sizeof(s_stats));

    if(ring_buf_create(&s_parser_ring,
                       s_parser_ring_buf,
                       VISION_PARSER_RING_SIZE,
                       false) != RING_BUF_SUCCESS) {
        return VISION_COMMS_STATUS_DEPENDENCY_MISSING;
    }

    if(frame_parser_create(&s_frame_parser,
                           &s_parser_ring,
                           s_frame_header,
                           sizeof(s_frame_header),
                           s_frame_buf,
                           VISION_FRAME_BUF_SIZE,
                           true) != FRAME_PARSER_SUCCESS) {
        return VISION_COMMS_STATUS_DEPENDENCY_MISSING;
    }

    s_initialized = true;
    return VISION_COMMS_STATUS_OK;
}

void vision_comms_on_rx_byte(uint8_t data) {
    uint16_t next;

    if(!s_initialized) {
        return;
    }

    next = (uint16_t)((s_rx_write + 1u) % VISION_RX_QUEUE_SIZE);
    if(next == s_rx_read) {
        s_stats.rx_overflow_count++;
        return;
    }

    s_rx_queue[s_rx_write] = data;
    s_rx_write = next;
}

void vision_comms_process(void) {
    uint8_t data;

    if(!s_initialized) {
        return;
    }

    while(vision_comms_queue_read(&data)) {
        if(frame_parser_write(&s_frame_parser, data) != FRAME_PARSER_SUCCESS) {
            s_stats.rx_overflow_count++;
        }
    }

    vision_comms_process_parser();
}

bool vision_comms_take_ball(BalanceVisionSample* sample) {
    if(sample == 0 || !s_ball_pending) {
        return false;
    }

    *sample = s_ball;
    s_ball_pending = false;
    return true;
}

bool vision_comms_take_command(BalanceCommand* command) {
    if(command == 0 || !s_command_pending) {
        return false;
    }

    *command = s_command;
    s_command_pending = false;
    return true;
}

bool vision_comms_get_heartbeat(VisionHeartbeat* heartbeat) {
    if(heartbeat == 0 || !s_heartbeat_valid) {
        return false;
    }

    *heartbeat = s_heartbeat;
    return true;
}

bool vision_comms_is_online(void) {
    if(!s_initialized || s_last_rx_ms == 0u) {
        return false;
    }

    return (vision_comms_now_ms() - s_last_rx_ms) <= VISION_ONLINE_TIMEOUT_MS;
}

bool vision_comms_send_status(const VisionMcuStatus* status) {
    uint8_t payload[VISION_STATUS_PAYLOAD_LEN] = { 0 };

    if(status == 0) {
        return false;
    }

    binary_frame_write_u32_le(&payload[0], status->stamp_ms);
    binary_frame_write_i16_le(&payload[4], status->offset_mm);
    binary_frame_write_i16_le(&payload[6], status->offset_vel_mm_s);
    binary_frame_write_i16_le(&payload[8], status->motor_angle_mrad);
    binary_frame_write_i16_le(&payload[10], status->pitch_mrad);
    binary_frame_write_i16_le(&payload[12], status->acc_mm_s2);
    payload[14] = status->state;
    payload[15] = status->reserved;
    payload[16] = status->flags;
    payload[17] = status->motor_error;
    binary_frame_write_u16_le(&payload[18], status->fault_flags);

    return vision_comms_send_frame(VISION_MSG_MCU_STATUS,
                                   vision_comms_next_seq(),
                                   0u,
                                   payload,
                                   sizeof(payload));
}

void vision_comms_get_stats(VisionCommsStats* stats) {
    if(stats != 0) {
        *stats = s_stats;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t vision_comms_now_ms(void) {
    return s_config.port_ops.now_ms();
}

static uint8_t vision_comms_next_seq(void) {
    uint8_t seq = s_tx_seq;
    s_tx_seq++;
    return seq;
}

static bool vision_comms_queue_read(uint8_t* data) {
    if(data == 0 || s_rx_read == s_rx_write) {
        return false;
    }

    *data = s_rx_queue[s_rx_read];
    s_rx_read = (uint16_t)((s_rx_read + 1u) % VISION_RX_QUEUE_SIZE);
    return true;
}

static bool vision_comms_send_frame(uint8_t msg_id, uint8_t seq, uint8_t flags,
                                    const uint8_t* payload, uint16_t payload_len) {
    uint16_t frame_len = 0u;

    if(!binary_frame_pack(msg_id,
                          seq,
                          flags,
                          payload,
                          payload_len,
                          s_tx_buf,
                          sizeof(s_tx_buf),
                          &frame_len)) {
        s_stats.tx_fail_count++;
        return false;
    }

    if(!s_config.port_ops.write((const char*)s_tx_buf, frame_len)) {
        s_stats.tx_fail_count++;
        return false;
    }

    s_stats.tx_frame_count++;
    return true;
}

static void vision_comms_send_ack(uint8_t msg_id, uint8_t seq, uint8_t status) {
    uint8_t payload[VISION_ACK_PAYLOAD_LEN] = { msg_id, seq, status, 0u };

    (void)vision_comms_send_frame(VISION_MSG_MCU_ACK,
                                  vision_comms_next_seq(),
                                  0u,
                                  payload,
                                  sizeof(payload));
}

static void vision_comms_process_parser(void) {
    for(;;) {
        uint8_t* body = 0;
        uint16_t body_len = 0u;
        FrameParserErrorCode result = frame_parser_process(&s_frame_parser);

        if(result == FRAME_PARSER_PROCESSING) {
            break;
        }

        if(result == FRAME_PARSER_ERR_CRC_MISMATCH) {
            s_stats.rx_bad_crc_count++;
            continue;
        }

        if(result == FRAME_PARSER_ERR_LENGTH_EXCEED) {
            s_stats.rx_bad_len_count++;
            continue;
        }

        if(result != FRAME_PARSER_SUCCESS) {
            (void)frame_parser_finish(&s_frame_parser);
            continue;
        }

        if(frame_parser_get_frame(&s_frame_parser, &body, &body_len) == FRAME_PARSER_SUCCESS) {
            vision_comms_process_frame(body, body_len);
        }
        (void)frame_parser_finish(&s_frame_parser);
    }
}

static void vision_comms_process_frame(const uint8_t* body, uint16_t body_len) {
    BinaryFrameView frame;

    if(!binary_frame_parse_body(body, body_len, &frame) ||
       frame.version != BINARY_FRAME_PROTOCOL_VERSION) {
        s_stats.rx_bad_len_count++;
        return;
    }

    s_stats.rx_frame_count++;
    s_stats.last_msg_id = frame.msg_id;
    s_stats.last_seq = frame.seq;
    s_last_rx_ms = vision_comms_now_ms();

    switch(frame.msg_id) {
        case VISION_MSG_HEARTBEAT:
            vision_comms_handle_heartbeat(&frame);
            break;

        case VISION_MSG_BALL:
            vision_comms_handle_ball(&frame);
            break;

        case VISION_MSG_COMMAND:
            vision_comms_handle_command(&frame);
            break;

        default:
            s_stats.rx_unknown_count++;
            break;
    }
}

static void vision_comms_handle_heartbeat(const BinaryFrameView* frame) {
    if(frame == 0 || frame->payload_len != VISION_HEARTBEAT_PAYLOAD_LEN) {
        s_stats.rx_bad_len_count++;
        return;
    }

    s_heartbeat.uptime_ms = binary_frame_read_u32_le(&frame->payload[0]);
    s_heartbeat.fps_x10 = binary_frame_read_u16_le(&frame->payload[4]);
    s_heartbeat.status = binary_frame_read_u16_le(&frame->payload[6]);
    s_heartbeat_valid = true;
}

static void vision_comms_handle_ball(const BinaryFrameView* frame) {
    uint8_t flags;

    if(frame == 0 || frame->payload_len != VISION_BALL_PAYLOAD_LEN) {
        s_stats.rx_bad_len_count++;
        return;
    }

    flags = frame->payload[9];
    s_ball.stamp_ms = vision_comms_now_ms();
    s_ball.capture_ms = binary_frame_read_u32_le(&frame->payload[0]);
    s_ball.frame_seq = binary_frame_read_u16_le(&frame->payload[4]);
    s_ball.offset_m = binary_frame_mm_to_m(binary_frame_read_i16_le(&frame->payload[6]));
    s_ball.quality = frame->payload[8];
    s_ball.valid = (flags & VISION_BALL_FLAG_VALID) != 0u;
    s_ball.velocity_m_s = binary_frame_mm_to_m(binary_frame_read_i16_le(&frame->payload[10]));
    s_ball_pending = true;
}

static void vision_comms_handle_command(const BinaryFrameView* frame) {
    uint16_t command_flags;
    uint8_t ack_status = 0u;

    if(frame == 0 || frame->payload_len != VISION_COMMAND_PAYLOAD_LEN) {
        s_stats.rx_bad_len_count++;
        if(frame != 0 && (frame->flags & BINARY_FRAME_FLAG_NEED_ACK) != 0u) {
            vision_comms_send_ack(VISION_MSG_COMMAND, frame->seq, 1u);
        }
        return;
    }

    command_flags = binary_frame_read_u16_le(&frame->payload[6]);
    s_command.enable = frame->payload[0] != 0u;
    s_command.command_seq = binary_frame_read_u16_le(&frame->payload[4]);
    s_command.clear_fault = (command_flags & VISION_CMD_FLAG_CLEAR_FAULT) != 0u;
    s_command_pending = true;

    if((frame->flags & BINARY_FRAME_FLAG_NEED_ACK) != 0u) {
        vision_comms_send_ack(VISION_MSG_COMMAND, frame->seq, ack_status);
    }
}
