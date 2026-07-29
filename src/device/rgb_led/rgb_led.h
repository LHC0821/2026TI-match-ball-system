#ifndef _rgb_led_h_
#define _rgb_led_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file rgb_led.h
 * @brief RGB LED 统一抽象接口
 */

/**
 * @brief 当前 RGB LED 实例的便捷访问宏
 */
#define rgb_led (*rgb_led_instance)

/**
 * @brief 单个 RGB 颜色占用字节数
 */
#define RGB_LED_COLOR_BYTES 3u

/**
 * @brief RGB LED 状态码表
 */
#define RGB_LED_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(INVALID_PARAM, 2) \
    X(PORT_ERROR, 3) \
    X(NO_INSTANCE, 4) \
    X(NOT_INITIALIZE, 5) \
    X(BUFFER_TOO_SMALL, 6) \
    X(INDEX_OUT_OF_RANGE, 7) \
    X(BUSY, 8)

#define X(name, value) RGB_LED_STATUS_##name = value,
/**
 * @brief RGB LED 通用状态码
 */
typedef enum {
    RGB_LED_STATUS_TABLE
} RgbLedStatus;
#undef X

/**
 * @brief RGB 颜色值
 */
typedef struct {
    uint8_t r; /**< 红色通道，范围 0~255 */
    uint8_t g; /**< 绿色通道，范围 0~255 */
    uint8_t b; /**< 蓝色通道，范围 0~255 */
} RgbLedColor;

/**
 * @brief RGB LED 底层端口函数表
 */
typedef struct {
    bool (*write)(const uint8_t* data, uint32_t len); /**< 写出已编码的灯带时序数据 */
} RgbLedPortOps;

/**
 * @brief RGB LED 初始化配置
 */
typedef struct {
    const RgbLedPortOps* ops; /**< 底层端口函数表，不能为空 */
    uint16_t pixel_count;     /**< 灯珠数量，单位 pixel */
    uint8_t* color_buffer;    /**< 颜色缓存，容量至少为 `pixel_count * RGB_LED_COLOR_BYTES` */
    uint32_t color_buffer_size; /**< 颜色缓存容量，单位 byte */
    uint8_t* tx_buffer;       /**< 发送缓存，容量由具体 LED 实例决定 */
    uint32_t tx_buffer_size;  /**< 发送缓存容量，单位 byte */
    uint16_t reset_bytes;     /**< reset 低电平填充字节数，具体含义由实例决定 */
    bool async_write;         /**< true 表示 write 返回后底层仍可能继续读取 tx_buffer */
} RgbLedConfig;

/**
 * @brief RGB LED 统一接口表
 */
typedef struct {
    RgbLedStatus(*init)(const RgbLedConfig* config); /**< 初始化具体 RGB LED 实例 */
    const char* (*status_str)(RgbLedStatus status);  /**< 状态码转字符串 */
    RgbLedStatus(*set_rgb)(uint16_t index, uint8_t r, uint8_t g, uint8_t b); /**< 设置单个灯珠颜色 */
    RgbLedStatus(*fill)(uint8_t r, uint8_t g, uint8_t b); /**< 设置全部灯珠颜色 */
    RgbLedStatus(*clear)(void);                          /**< 清空全部灯珠颜色缓存 */
    RgbLedStatus(*show)(void);                           /**< 编码并发送颜色缓存 */
    RgbLedStatus(*write_complete)(void);                 /**< 通知异步写入完成 */
    RgbLedStatus(*get_color)(uint16_t index, RgbLedColor* out); /**< 获取单个灯珠缓存颜色 */
} RgbLedInterface;

/**
 * @brief 当前绑定的 RGB LED 实例
 */
extern const RgbLedInterface* rgb_led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 绑定具体 RGB LED 实例
 * @param instance RGB LED 接口表，例如 `&ws2812_rgb_led_instance`
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_set_instance(const RgbLedInterface* instance);

/**
 * @brief 初始化当前绑定的 RGB LED 实例
 * @param config 初始化配置
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_init(const RgbLedConfig* config);

/**
 * @brief 状态码转字符串
 * @param status RGB LED 状态码
 * @return 状态码名称字符串
 */
const char* rgb_led_status_str(RgbLedStatus status);

/**
 * @brief 设置单个灯珠颜色，只更新缓存，不立即发送
 * @param index 灯珠索引，从 0 开始
 * @param r 红色通道，范围 0~255
 * @param g 绿色通道，范围 0~255
 * @param b 蓝色通道，范围 0~255
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置全部灯珠颜色，只更新缓存，不立即发送
 * @param r 红色通道，范围 0~255
 * @param g 绿色通道，范围 0~255
 * @param b 蓝色通道，范围 0~255
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_fill(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清空全部灯珠颜色缓存，不立即发送
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_clear(void);

/**
 * @brief 编码颜色缓存并发送到真实灯带
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_show(void);

/**
 * @brief 通知当前实例上一段异步写入已经完成
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_write_complete(void);

/**
 * @brief 获取单个灯珠当前缓存颜色
 * @param index 灯珠索引，从 0 开始
 * @param out 输出颜色
 * @return RGB LED 状态码
 */
RgbLedStatus rgb_led_get_color(uint16_t index, RgbLedColor* out);

#endif
