#include "binary_frame.h"

#include <math.h>
#include <stddef.h>
#include <limits.h>

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint16_t binary_frame_crc16_update(uint16_t crc, uint8_t data);
static int32_t binary_frame_round_clamp_i32(float value);
static int16_t binary_frame_round_clamp_i16(float value);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool binary_frame_pack(uint8_t msg_id,
                       uint8_t seq,
                       uint8_t flags,
                       const uint8_t* payload,
                       uint16_t payload_len,
                       uint8_t* out,
                       uint16_t out_size,
                       uint16_t* out_len) {
    const uint16_t body_len = (uint16_t)(BINARY_FRAME_BODY_PREFIX_SIZE + payload_len);
    const uint16_t frame_len = (uint16_t)(BINARY_FRAME_FIXED_OVERHEAD + body_len);
    uint16_t crc;

    if(out == NULL || out_len == NULL) {
        return false;
    }

    if((payload_len > 0u && payload == NULL) || frame_len > out_size) {
        return false;
    }

    out[0] = BINARY_FRAME_SOF0;
    out[1] = BINARY_FRAME_SOF1;
    out[2] = (uint8_t)(body_len >> 8);
    out[3] = (uint8_t)(body_len & 0xFFu);
    out[4] = BINARY_FRAME_PROTOCOL_VERSION;
    out[5] = msg_id;
    out[6] = seq;
    out[7] = flags;

    if(payload_len > 0u) {
        for(uint16_t i = 0u; i < payload_len; i++) {
            out[8u + i] = payload[i];
        }
    }

    crc = binary_frame_crc16_ccitt(out, (uint16_t)(BINARY_FRAME_HEADER_SIZE + BINARY_FRAME_LENGTH_FIELD_SIZE + body_len));
    out[8u + payload_len] = (uint8_t)(crc >> 8);
    out[9u + payload_len] = (uint8_t)(crc & 0xFFu);
    *out_len = frame_len;
    return true;
}

bool binary_frame_parse_body(const uint8_t* body,
                             uint16_t body_len,
                             BinaryFrameView* view) {
    if(body == NULL || view == NULL || body_len < BINARY_FRAME_BODY_PREFIX_SIZE) {
        return false;
    }

    view->version = body[0];
    view->msg_id = body[1];
    view->seq = body[2];
    view->flags = body[3];
    view->payload = &body[BINARY_FRAME_BODY_PREFIX_SIZE];
    view->payload_len = (uint16_t)(body_len - BINARY_FRAME_BODY_PREFIX_SIZE);
    return true;
}

uint16_t binary_frame_crc16_ccitt(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFFu;

    if(data == NULL) {
        return 0u;
    }

    for(uint16_t i = 0u; i < len; i++) {
        crc = binary_frame_crc16_update(crc, data[i]);
    }

    return crc;
}

uint16_t binary_frame_read_u16_le(const uint8_t* data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

int16_t binary_frame_read_i16_le(const uint8_t* data) {
    return (int16_t)binary_frame_read_u16_le(data);
}

uint32_t binary_frame_read_u32_le(const uint8_t* data) {
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

int32_t binary_frame_read_i32_le(const uint8_t* data) {
    return (int32_t)binary_frame_read_u32_le(data);
}

void binary_frame_write_u16_le(uint8_t* data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)(value >> 8);
}

void binary_frame_write_i16_le(uint8_t* data, int16_t value) {
    binary_frame_write_u16_le(data, (uint16_t)value);
}

void binary_frame_write_u32_le(uint8_t* data, uint32_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

void binary_frame_write_i32_le(uint8_t* data, int32_t value) {
    binary_frame_write_u32_le(data, (uint32_t)value);
}

float binary_frame_urad_to_rad(int32_t urad) {
    return (float)urad / 1000000.0f;
}

int32_t binary_frame_rad_to_urad(float rad) {
    return binary_frame_round_clamp_i32(rad * 1000000.0f);
}

float binary_frame_mrad_to_rad(int32_t mrad) {
    return (float)mrad / 1000.0f;
}

int16_t binary_frame_rad_to_mrad_i16(float rad) {
    return binary_frame_round_clamp_i16(rad * 1000.0f);
}

float binary_frame_mm_to_m(int32_t mm) {
    return (float)mm / 1000.0f;
}

int16_t binary_frame_m_to_mm_i16(float m) {
    return binary_frame_round_clamp_i16(m * 1000.0f);
}

int32_t binary_frame_m_to_mm_i32(float m) {
    return binary_frame_round_clamp_i32(m * 1000.0f);
}

int32_t binary_frame_unit_to_e6_i32(float value) {
    return binary_frame_round_clamp_i32(value * 1000000.0f);
}

int16_t binary_frame_unit_to_q15_i16(float value) {
    return binary_frame_round_clamp_i16(value * 32767.0f);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint16_t binary_frame_crc16_update(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for(uint8_t i = 0u; i < 8u; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static int32_t binary_frame_round_clamp_i32(float value) {
    if(!isfinite(value)) {
        return 0;
    }
    if(value >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if(value <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(value);
}

static int16_t binary_frame_round_clamp_i16(float value) {
    if(!isfinite(value)) {
        return 0;
    }
    if(value >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if(value <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)lroundf(value);
}
