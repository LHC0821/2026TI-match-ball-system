#include "protocol_parser.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

static const ProtocolParserCriticalOps* s_default_ops = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void rb_enter(const RingBuf* self);
static void rb_exit(const RingBuf* self);
static uint16_t crc16_update(uint16_t crc, uint8_t data);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void protocol_parser_register_critical_ops(const ProtocolParserCriticalOps* ops) {
    s_default_ops = ops;
}

RingBufErrorCode ring_buf_create(RingBuf* const self,
                                 uint8_t* const buf,
                                 const uint16_t capacity,
                                 const bool overwrite) {
    return ring_buf_create_with_ops(self, buf, capacity, overwrite, s_default_ops);
}

RingBufErrorCode ring_buf_create_with_ops(RingBuf* const self,
                                          uint8_t* const buf,
                                          const uint16_t capacity,
                                          const bool overwrite,
                                          const ProtocolParserCriticalOps* ops) {
    if(self == NULL || buf == NULL) {
        return RING_BUF_ERR_NULL_PTR;
    }
    if(capacity == 0u) {
        return RING_BUF_ERR_FULL;
    }

    self->_ops_ = ops;
    self->_buf_ = buf;
    self->_size_ = 0u;
    self->_capacity_ = capacity;
    self->_write_idx_ = 0u;
    self->_read_idx_ = 0u;
    self->_overwrite_ = overwrite;
    self->_initialized_ = true;

    return RING_BUF_SUCCESS;
}

RingBufErrorCode ring_buf_write(RingBuf* const self, const uint8_t data) {
    if(self == NULL) {
        return RING_BUF_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return RING_BUF_ERR_NOT_INITIALIZE;
    }

    rb_enter(self);

    if(self->_size_ >= self->_capacity_) {
        if(self->_overwrite_ == false) {
            rb_exit(self);
            return RING_BUF_ERR_FULL;
        }
        self->_buf_[self->_write_idx_] = data;
        self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1u) % self->_capacity_);
        self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1u) % self->_capacity_);
        rb_exit(self);
        return RING_BUF_SUCCESS;
    }

    self->_buf_[self->_write_idx_] = data;
    self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1u) % self->_capacity_);
    self->_size_++;

    rb_exit(self);
    return RING_BUF_SUCCESS;
}

RingBufErrorCode ring_buf_read(RingBuf* const self, uint8_t* const data) {
    if(self == NULL || data == NULL) {
        return RING_BUF_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return RING_BUF_ERR_NOT_INITIALIZE;
    }

    rb_enter(self);

    if(self->_size_ == 0u) {
        rb_exit(self);
        return RING_BUF_ERR_EMPTY;
    }

    *data = self->_buf_[self->_read_idx_];
    self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1u) % self->_capacity_);
    self->_size_--;

    rb_exit(self);
    return RING_BUF_SUCCESS;
}

RingBufErrorCode ring_buf_clear(RingBuf* const self) {
    if(self == NULL) {
        return RING_BUF_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return RING_BUF_ERR_NOT_INITIALIZE;
    }

    rb_enter(self);
    self->_size_ = 0u;
    self->_write_idx_ = 0u;
    self->_read_idx_ = 0u;
    rb_exit(self);

    return RING_BUF_SUCCESS;
}

bool ring_buf_is_full(RingBuf* const self) {
    if(self == NULL || self->_initialized_ == false) {
        return false;
    }

    return self->_size_ >= self->_capacity_;
}

bool ring_buf_is_empty(RingBuf* const self) {
    if(self == NULL || self->_initialized_ == false) {
        return false;
    }

    return self->_size_ == 0u;
}

int ring_buf_get_size(RingBuf* const self) {
    if(self == NULL || self->_initialized_ == false) {
        return -1;
    }

    return (int)self->_size_;
}

int ring_buf_get_capacity(RingBuf* const self) {
    if(self == NULL || self->_initialized_ == false) {
        return -1;
    }

    return (int)self->_capacity_;
}

FrameParserErrorCode frame_parser_create(FrameParser* const self,
                                         RingBuf* const ring_buf,
                                         const uint8_t* const header,
                                         const uint8_t header_length,
                                         uint8_t* const frame_buf,
                                         const uint16_t frame_buf_capacity,
                                         const bool crc_enabled) {
    if(self == NULL || ring_buf == NULL || header == NULL || frame_buf == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(ring_buf->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }
    if(header_length < 2u) {
        return FRAME_PARSER_ERR_HEADER_TOO_SHORT;
    }
    if(frame_buf_capacity == 0u || (crc_enabled && frame_buf_capacity <= 2u)) {
        return FRAME_PARSER_ERR_BUF_TOO_SMALL;
    }

    self->_ring_buf_ = ring_buf;
    self->_state_ = STATE_IDLE;
    self->_header_ = header;
    self->_header_length_ = header_length;
    self->_header_match_idx_ = 0u;
    self->_expected_length_ = 0u;
    self->_received_length_ = 0u;
    self->_frame_buf_ = frame_buf;
    self->_frame_buf_capacity_ = (uint16_t)(frame_buf_capacity - (crc_enabled ? 2u : 0u));
    self->_crc_enabled_ = crc_enabled;
    self->_crc_accum_ = 0u;
    self->_received_crc_ = 0u;
    self->_initialized_ = true;

    return FRAME_PARSER_SUCCESS;
}

FrameParserErrorCode frame_parser_write(FrameParser* const self, const uint8_t data) {
    RingBufErrorCode ret;

    if(self == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }

    ret = ring_buf_write(self->_ring_buf_, data);
    switch(ret) {
        case RING_BUF_SUCCESS:
            return FRAME_PARSER_SUCCESS;
        case RING_BUF_ERR_NULL_PTR:
            return FRAME_PARSER_ERR_NULL_PTR;
        case RING_BUF_ERR_FULL:
            return FRAME_PARSER_ERR_BUFFER_FULL;
        case RING_BUF_ERR_NOT_INITIALIZE:
            return FRAME_PARSER_ERR_NOT_INITIALIZE;
        default:
            return FRAME_PARSER_ERR_INVALID_STATE;
    }
}

FrameParserErrorCode frame_parser_process(FrameParser* const self) {
    uint8_t byte;

    if(self == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }

    while(ring_buf_read(self->_ring_buf_, &byte) == RING_BUF_SUCCESS) {
        switch(self->_state_) {
            case STATE_IDLE:
                if(byte == self->_header_[0]) {
                    self->_crc_accum_ = crc16_update(0xFFFFu, byte);
                    self->_header_match_idx_ = 1u;
                    self->_state_ = STATE_HEADER_MATCHING;
                }
                break;

            case STATE_HEADER_MATCHING:
                if(byte == self->_header_[self->_header_match_idx_]) {
                    self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                    self->_header_match_idx_++;
                    if(self->_header_match_idx_ >= self->_header_length_) {
                        self->_state_ = STATE_READ_LENGTH;
                        self->_received_length_ = 0u;
                    }
                }
                else {
                    self->_state_ = (byte == self->_header_[0]) ? STATE_HEADER_MATCHING : STATE_IDLE;
                    self->_crc_accum_ = (byte == self->_header_[0]) ? crc16_update(0xFFFFu, byte) : 0u;
                    self->_header_match_idx_ = (byte == self->_header_[0]) ? 1u : 0u;
                }
                break;

            case STATE_READ_LENGTH:
                self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                if(self->_received_length_ == 0u) {
                    self->_expected_length_ = (uint16_t)((uint16_t)byte << 8);
                    self->_received_length_ = 1u;
                }
                else {
                    self->_expected_length_ |= (uint16_t)byte;
                    self->_received_length_ = 0u;

                    if(self->_expected_length_ > self->_frame_buf_capacity_) {
                        (void)frame_parser_finish(self);
                        return FRAME_PARSER_ERR_LENGTH_EXCEED;
                    }
                    if(self->_expected_length_ == 0u) {
                        self->_state_ = self->_crc_enabled_ ? STATE_READ_CRC : STATE_FRAME_COMPLETE;
                        if(self->_state_ == STATE_FRAME_COMPLETE) {
                            return FRAME_PARSER_SUCCESS;
                        }
                    }
                    else {
                        self->_state_ = STATE_READ_PAYLOAD;
                    }
                }
                break;

            case STATE_READ_PAYLOAD:
                self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                self->_frame_buf_[self->_received_length_] = byte;
                self->_received_length_++;

                if(self->_received_length_ >= self->_expected_length_) {
                    self->_received_length_ = 0u;
                    self->_received_crc_ = 0u;
                    self->_state_ = self->_crc_enabled_ ? STATE_READ_CRC : STATE_FRAME_COMPLETE;
                    if(self->_state_ == STATE_FRAME_COMPLETE) {
                        return FRAME_PARSER_SUCCESS;
                    }
                }
                break;

            case STATE_READ_CRC:
                if(self->_received_length_ == 0u) {
                    self->_received_crc_ = (uint16_t)((uint16_t)byte << 8);
                    self->_received_length_ = 1u;
                }
                else {
                    self->_received_crc_ |= (uint16_t)byte;
                    self->_received_length_ = 0u;

                    if(self->_received_crc_ == self->_crc_accum_) {
                        self->_state_ = STATE_FRAME_COMPLETE;
                        return FRAME_PARSER_SUCCESS;
                    }

                    (void)frame_parser_finish(self);
                    return FRAME_PARSER_ERR_CRC_MISMATCH;
                }
                break;

            case STATE_FRAME_COMPLETE:
                return FRAME_PARSER_SUCCESS;

            default:
                return FRAME_PARSER_ERR_INVALID_STATE;
        }
    }

    return FRAME_PARSER_PROCESSING;
}

FrameParserErrorCode frame_parser_get_frame(FrameParser* const self,
                                            uint8_t** const frame_buffer,
                                            uint16_t* const frame_length) {
    if(self == NULL || frame_buffer == NULL || frame_length == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }
    if(self->_state_ != STATE_FRAME_COMPLETE) {
        return FRAME_PARSER_ERR_NO_FRAME;
    }

    *frame_buffer = self->_frame_buf_;
    *frame_length = self->_expected_length_;
    return FRAME_PARSER_SUCCESS;
}

FrameParserErrorCode frame_parser_finish(FrameParser* const self) {
    if(self == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }

    self->_state_ = STATE_IDLE;
    self->_header_match_idx_ = 0u;
    self->_expected_length_ = 0u;
    self->_received_length_ = 0u;
    self->_crc_accum_ = 0u;
    self->_received_crc_ = 0u;

    return FRAME_PARSER_SUCCESS;
}

FrameParserErrorCode frame_parser_reset(FrameParser* const self) {
    if(self == NULL) {
        return FRAME_PARSER_ERR_NULL_PTR;
    }
    if(self->_initialized_ == false) {
        return FRAME_PARSER_ERR_NOT_INITIALIZE;
    }

    (void)ring_buf_clear(self->_ring_buf_);
    return frame_parser_finish(self);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void rb_enter(const RingBuf* self) {
    if(self != NULL && self->_ops_ != NULL && self->_ops_->enter != NULL) {
        self->_ops_->enter();
    }
}

static void rb_exit(const RingBuf* self) {
    if(self != NULL && self->_ops_ != NULL && self->_ops_->exit != NULL) {
        self->_ops_->exit();
    }
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for(uint8_t i = 0u; i < 8u; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}
