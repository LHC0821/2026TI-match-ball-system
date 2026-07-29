#ifndef _ws2812_rgb_led_h_
#define _ws2812_rgb_led_h_

#include "rgb_led.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file ws2812_rgb_led.h
 * @brief WS2812 RGB LED 具体实例接口
 */

/**
 * @brief WS2812 单个 RGB 灯珠编码后的 bit 数
 */
#define WS2812_RGB_LED_BITS_PER_PIXEL 24u

/**
 * @brief WS2812 RGB LED 默认像素数量
 */
#define WS2812_RGB_LED_DEFAULT_PIXEL_COUNT 8u

/**
 * @brief WS2812 RGB LED 默认复位时间对应的填充字节数
 *
 * 以 800 kHz 速率发送时，复位低电平通常需要大于 50 us
 */
#define WS2812_RGB_LED_DEFAULT_RESET_BYTES 80u

/**
 * @brief WS2812 RGB LED 统一接口实例
 */
extern const RgbLedInterface ws2812_rgb_led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 生成 WS2812 默认配置
 * @param out_config 输出 RGB LED 配置
 * @param ops 底层端口函数表
 * @param color_buffer 颜色缓存
 * @param color_buffer_size 颜色缓存容量，单位 byte
 * @param tx_buffer 发送缓存
 * @param tx_buffer_size 发送缓存容量，单位 byte
 * @return RGB LED 状态码
 */
RgbLedStatus ws2812_rgb_led_make_config(RgbLedConfig* out_config, const RgbLedPortOps* ops,
    uint8_t* color_buffer, uint32_t color_buffer_size,
    uint8_t* tx_buffer, uint32_t tx_buffer_size);

#endif
