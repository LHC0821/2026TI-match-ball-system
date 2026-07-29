#ifndef _binary_frame_h_
#define _binary_frame_h_

/**
 * @file binary_frame.h
 * @brief 通用二进制帧协议辅助接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BINARY_FRAME_SOF0 0xA5u
#define BINARY_FRAME_SOF1 0x5Au
#define BINARY_FRAME_PROTOCOL_VERSION 0x01u
#define BINARY_FRAME_HEADER_SIZE 2u
#define BINARY_FRAME_LENGTH_FIELD_SIZE 2u
#define BINARY_FRAME_BODY_PREFIX_SIZE 4u
#define BINARY_FRAME_CRC_SIZE 2u
#define BINARY_FRAME_FIXED_OVERHEAD (BINARY_FRAME_HEADER_SIZE + BINARY_FRAME_LENGTH_FIELD_SIZE + BINARY_FRAME_CRC_SIZE)

#define BINARY_FRAME_FLAG_NEED_ACK 0x01u

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    uint8_t version;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t flags;
    const uint8_t* payload;
    uint16_t payload_len;
} BinaryFrameView;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool binary_frame_pack(uint8_t msg_id,
                       uint8_t seq,
                       uint8_t flags,
                       const uint8_t* payload,
                       uint16_t payload_len,
                       uint8_t* out,
                       uint16_t out_size,
                       uint16_t* out_len);

bool binary_frame_parse_body(const uint8_t* body,
                             uint16_t body_len,
                             BinaryFrameView* view);

uint16_t binary_frame_crc16_ccitt(const uint8_t* data, uint16_t len);

uint16_t binary_frame_read_u16_le(const uint8_t* data);
int16_t binary_frame_read_i16_le(const uint8_t* data);
uint32_t binary_frame_read_u32_le(const uint8_t* data);
int32_t binary_frame_read_i32_le(const uint8_t* data);

void binary_frame_write_u16_le(uint8_t* data, uint16_t value);
void binary_frame_write_i16_le(uint8_t* data, int16_t value);
void binary_frame_write_u32_le(uint8_t* data, uint32_t value);
void binary_frame_write_i32_le(uint8_t* data, int32_t value);

float binary_frame_urad_to_rad(int32_t urad);
int32_t binary_frame_rad_to_urad(float rad);
float binary_frame_mrad_to_rad(int32_t mrad);
int16_t binary_frame_rad_to_mrad_i16(float rad);
float binary_frame_mm_to_m(int32_t mm);
int16_t binary_frame_m_to_mm_i16(float m);
int32_t binary_frame_m_to_mm_i32(float m);
int32_t binary_frame_unit_to_e6_i32(float value);
int16_t binary_frame_unit_to_q15_i16(float value);

#endif
