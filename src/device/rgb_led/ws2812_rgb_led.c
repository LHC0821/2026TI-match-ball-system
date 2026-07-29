#include "ws2812_rgb_led.h"

#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define WS2812_LOW_LEVEL 0x60u
#define WS2812_HIGH_LEVEL 0x78u

static const RgbLedPortOps* s_ops;
static uint16_t s_pixel_count;
static uint8_t* s_color_buffer;
static uint32_t s_color_buffer_size;
static uint8_t* s_tx_buffer;
static uint32_t s_tx_buffer_size;
static uint16_t s_reset_bytes;
static bool s_async_write;
static volatile bool s_write_busy;
static bool s_initialized;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static RgbLedStatus ws2812_rgb_led_init(const RgbLedConfig* config);
static const char* ws2812_rgb_led_status_str(RgbLedStatus status);
static RgbLedStatus ws2812_rgb_led_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
static RgbLedStatus ws2812_rgb_led_fill(uint8_t r, uint8_t g, uint8_t b);
static RgbLedStatus ws2812_rgb_led_clear(void);
static RgbLedStatus ws2812_rgb_led_show(void);
static RgbLedStatus ws2812_rgb_led_write_complete(void);
static RgbLedStatus ws2812_rgb_led_get_color(uint16_t index, RgbLedColor* out);
static void ws2812_rgb_led_encode_byte(uint8_t value, uint8_t* out);
static uint32_t ws2812_rgb_led_color_offset(uint16_t index);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const RgbLedInterface ws2812_rgb_led_instance = {
    .init = ws2812_rgb_led_init,
    .status_str = ws2812_rgb_led_status_str,
    .set_rgb = ws2812_rgb_led_set_rgb,
    .fill = ws2812_rgb_led_fill,
    .clear = ws2812_rgb_led_clear,
    .show = ws2812_rgb_led_show,
    .write_complete = ws2812_rgb_led_write_complete,
    .get_color = ws2812_rgb_led_get_color,
};

static RgbLedStatus ws2812_rgb_led_init(const RgbLedConfig* config) {
    uint32_t need_color_size;
    uint32_t need_tx_size;

    if(config == 0 || config->ops == 0 || config->ops->write == 0 ||
        config->pixel_count == 0u || config->color_buffer == 0 || config->tx_buffer == 0) {
        return RGB_LED_STATUS_INVALID_PARAM;
    }

    need_color_size = (uint32_t)config->pixel_count * RGB_LED_COLOR_BYTES;
    need_tx_size = ((uint32_t)config->pixel_count * WS2812_RGB_LED_BITS_PER_PIXEL) + config->reset_bytes;

    if(config->color_buffer_size < need_color_size || config->tx_buffer_size < need_tx_size) {
        return RGB_LED_STATUS_BUFFER_TOO_SMALL;
    }

    s_ops = config->ops;
    s_pixel_count = config->pixel_count;
    s_color_buffer = config->color_buffer;
    s_color_buffer_size = config->color_buffer_size;
    s_tx_buffer = config->tx_buffer;
    s_tx_buffer_size = config->tx_buffer_size;
    s_reset_bytes = config->reset_bytes;
    s_async_write = config->async_write;
    s_write_busy = false;
    s_initialized = true;

    memset(s_color_buffer, 0, s_color_buffer_size);
    memset(s_tx_buffer, 0, s_tx_buffer_size);

    return RGB_LED_STATUS_OK;
}

static const char* ws2812_rgb_led_status_str(RgbLedStatus status) {
    switch(status) {
#define X(name, value) case RGB_LED_STATUS_##name: return #name;
        RGB_LED_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

static RgbLedStatus ws2812_rgb_led_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t offset;

    if(s_initialized == false) {
        return RGB_LED_STATUS_NOT_INITIALIZE;
    }

    if(index >= s_pixel_count) {
        return RGB_LED_STATUS_INDEX_OUT_OF_RANGE;
    }

    offset = ws2812_rgb_led_color_offset(index);

    s_color_buffer[offset + 0u] = g;
    s_color_buffer[offset + 1u] = r;
    s_color_buffer[offset + 2u] = b;

    return RGB_LED_STATUS_OK;
}

static RgbLedStatus ws2812_rgb_led_fill(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t i;
    RgbLedStatus status;

    if(s_initialized == false) {
        return RGB_LED_STATUS_NOT_INITIALIZE;
    }

    for(i = 0; i < s_pixel_count; i++) {
        status = ws2812_rgb_led_set_rgb(i, r, g, b);
        if(status != RGB_LED_STATUS_OK) {
            return status;
        }
    }

    return RGB_LED_STATUS_OK;
}

static RgbLedStatus ws2812_rgb_led_clear(void) {
    return ws2812_rgb_led_fill(0u, 0u, 0u);
}

static RgbLedStatus ws2812_rgb_led_show(void) {
    uint16_t pixel;
    uint32_t color_base;
    uint32_t tx_base;
    uint32_t send_len;

    if(s_initialized == false) {
        return RGB_LED_STATUS_NOT_INITIALIZE;
    }

    if(s_ops == 0 || s_ops->write == 0) {
        return RGB_LED_STATUS_PORT_ERROR;
    }

    if(s_async_write && s_write_busy) {
        return RGB_LED_STATUS_BUSY;
    }

    send_len = ((uint32_t)s_pixel_count * WS2812_RGB_LED_BITS_PER_PIXEL) + s_reset_bytes;

    if(send_len > s_tx_buffer_size) {
        return RGB_LED_STATUS_BUFFER_TOO_SMALL;
    }

    for(pixel = 0; pixel < s_pixel_count; pixel++) {
        color_base = ws2812_rgb_led_color_offset(pixel);
        tx_base = (uint32_t)pixel * WS2812_RGB_LED_BITS_PER_PIXEL;
        ws2812_rgb_led_encode_byte(s_color_buffer[color_base + 0u], &s_tx_buffer[tx_base + 0u]);
        ws2812_rgb_led_encode_byte(s_color_buffer[color_base + 1u], &s_tx_buffer[tx_base + 8u]);
        ws2812_rgb_led_encode_byte(s_color_buffer[color_base + 2u], &s_tx_buffer[tx_base + 16u]);
    }

    if(s_reset_bytes > 0u) {
        memset(&s_tx_buffer[(uint32_t)s_pixel_count * WS2812_RGB_LED_BITS_PER_PIXEL], 0, s_reset_bytes);
    }

    if(s_async_write) {
        s_write_busy = true;
    }

    if(s_ops->write(s_tx_buffer, send_len) == false) {
        s_write_busy = false;
        return RGB_LED_STATUS_PORT_ERROR;
    }

    return RGB_LED_STATUS_OK;
}

static RgbLedStatus ws2812_rgb_led_write_complete(void) {
    s_write_busy = false;
    return RGB_LED_STATUS_OK;
}

static RgbLedStatus ws2812_rgb_led_get_color(uint16_t index, RgbLedColor* out) {
    uint32_t offset;

    if(s_initialized == false) {
        return RGB_LED_STATUS_NOT_INITIALIZE;
    }

    if(out == 0) {
        return RGB_LED_STATUS_INVALID_PARAM;
    }

    if(index >= s_pixel_count) {
        return RGB_LED_STATUS_INDEX_OUT_OF_RANGE;
    }

    offset = ws2812_rgb_led_color_offset(index);
    out->g = s_color_buffer[offset + 0u];
    out->r = s_color_buffer[offset + 1u];
    out->b = s_color_buffer[offset + 2u];

    return RGB_LED_STATUS_OK;
}

RgbLedStatus ws2812_rgb_led_make_config(RgbLedConfig* out_config, const RgbLedPortOps* ops,
    uint8_t* color_buffer, uint32_t color_buffer_size,
    uint8_t* tx_buffer, uint32_t tx_buffer_size) {
    if(out_config == 0 || ops == 0 || ops->write == 0 || color_buffer == 0 || tx_buffer == 0) return RGB_LED_STATUS_INVALID_PARAM;

    out_config->ops = ops;
    out_config->pixel_count = 8;
    out_config->color_buffer = color_buffer;
    out_config->color_buffer_size = color_buffer_size;
    out_config->tx_buffer = tx_buffer;
    out_config->tx_buffer_size = tx_buffer_size;
    out_config->reset_bytes = 80;
    out_config->async_write = false;

    return RGB_LED_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void ws2812_rgb_led_encode_byte(uint8_t value, uint8_t* out) {
    uint8_t i;

    for(i = 0; i < 8u; i++) {
        out[i] = ((value & (uint8_t)(0x80u >> i)) != 0u) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL;
    }
}

static uint32_t ws2812_rgb_led_color_offset(uint16_t index) {
    return (uint32_t)index * RGB_LED_COLOR_BYTES;
}
