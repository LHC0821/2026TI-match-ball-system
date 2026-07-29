#ifndef _protocol_parser_h_
#define _protocol_parser_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 环形缓冲区临界区钩子
 */
typedef struct {
    void (*enter)(void);
    void (*exit)(void);
} ProtocolParserCriticalOps;

typedef enum {
    RING_BUF_SUCCESS = 0,
    RING_BUF_ERR_NULL_PTR,
    RING_BUF_ERR_IN_USE,
    RING_BUF_ERR_FULL,
    RING_BUF_ERR_EMPTY,
    RING_BUF_ERR_NOT_INITIALIZE,
} RingBufErrorCode;

typedef enum {
    FRAME_PARSER_SUCCESS = 0,
    FRAME_PARSER_PROCESSING,
    FRAME_PARSER_ERR_HEADER_TOO_SHORT,
    FRAME_PARSER_ERR_NULL_PTR,
    FRAME_PARSER_ERR_BUF_TOO_SMALL,
    FRAME_PARSER_ERR_INVALID_STATE,
    FRAME_PARSER_ERR_BUFFER_FULL,
    FRAME_PARSER_ERR_LENGTH_EXCEED,
    FRAME_PARSER_ERR_NO_FRAME,
    FRAME_PARSER_ERR_CRC_MISMATCH,
    FRAME_PARSER_ERR_NOT_INITIALIZE,
} FrameParserErrorCode;

typedef enum {
    STATE_IDLE = 0,
    STATE_HEADER_MATCHING,
    STATE_READ_LENGTH,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC,
    STATE_FRAME_COMPLETE,
} FrameParserState;

/**
 * @brief 字节环形缓冲区
 *
 * 缓冲区本体不分配内存，调用方通过 ring_buf_create() 传入存储区
 * 可选临界区钩子来自全局注册默认值，或通过 ring_buf_create_with_ops() 单独指定
 */
typedef struct {
    const ProtocolParserCriticalOps* _ops_;
    uint8_t* _buf_;
    uint16_t _size_;
    uint16_t _capacity_;
    uint16_t _write_idx_;
    uint16_t _read_idx_;
    bool _overwrite_;
    bool _initialized_;
} RingBuf;

/**
 * @brief 流式帧解析器
 *
 * 帧格式：
 * [帧头][长度高字节][长度低字节][载荷][CRC 高字节][CRC 低字节]
 *
 * CRC 使用 CRC-16/CCITT，覆盖帧头、长度字段和有效载荷；是否启用由 crc_enabled 控制
 * 解析器本体不分配内存
 */
typedef struct {
    RingBuf* _ring_buf_;
    FrameParserState _state_;
    const uint8_t* _header_;
    uint8_t _header_length_;
    uint8_t _header_match_idx_;
    uint16_t _expected_length_;
    uint16_t _received_length_;
    uint8_t* _frame_buf_;
    uint16_t _frame_buf_capacity_;
    bool _crc_enabled_;
    uint16_t _crc_accum_;
    uint16_t _received_crc_;
    bool _initialized_;
} FrameParser;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 注册默认环形缓冲区临界区钩子
 *
 * 这里只保存指针，不拷贝结构体内容；所有通过 ring_buf_create() 创建的缓冲区都会引用它
 * 传入 NULL 表示恢复为空操作
 */
void protocol_parser_register_critical_ops(const ProtocolParserCriticalOps* ops);

RingBufErrorCode ring_buf_create(RingBuf* const self,
                                 uint8_t* const buf,
                                 const uint16_t capacity,
                                 const bool overwrite);
RingBufErrorCode ring_buf_create_with_ops(RingBuf* const self,
                                          uint8_t* const buf,
                                          const uint16_t capacity,
                                          const bool overwrite,
                                          const ProtocolParserCriticalOps* ops);
RingBufErrorCode ring_buf_write(RingBuf* const self, const uint8_t data);
RingBufErrorCode ring_buf_read(RingBuf* const self, uint8_t* const data);
RingBufErrorCode ring_buf_clear(RingBuf* const self);
bool ring_buf_is_full(RingBuf* const self);
bool ring_buf_is_empty(RingBuf* const self);
int ring_buf_get_size(RingBuf* const self);
int ring_buf_get_capacity(RingBuf* const self);

FrameParserErrorCode frame_parser_create(FrameParser* const self,
                                         RingBuf* const ring_buf,
                                         const uint8_t* const header,
                                         const uint8_t header_length,
                                         uint8_t* const frame_buf,
                                         const uint16_t frame_buf_capacity,
                                         const bool crc_enabled);
FrameParserErrorCode frame_parser_write(FrameParser* const self, const uint8_t data);
FrameParserErrorCode frame_parser_process(FrameParser* const self);
FrameParserErrorCode frame_parser_get_frame(FrameParser* const self,
                                            uint8_t** const frame_buffer,
                                            uint16_t* const frame_length);
FrameParserErrorCode frame_parser_finish(FrameParser* const self);
FrameParserErrorCode frame_parser_reset(FrameParser* const self);

#endif
