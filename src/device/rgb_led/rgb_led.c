#include "rgb_led.h"

// ! ========================= 变 量 声 明 ========================= ! //

const RgbLedInterface* rgb_led_instance = 0;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

RgbLedStatus rgb_led_set_instance(const RgbLedInterface* instance) {
    if(instance == 0) {
        return RGB_LED_STATUS_INVALID_PARAM;
    }

    rgb_led_instance = instance;
    return RGB_LED_STATUS_OK;
}

RgbLedStatus rgb_led_init(const RgbLedConfig* config) {
    if(rgb_led_instance == 0 || rgb_led_instance->init == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->init(config);
}

const char* rgb_led_status_str(RgbLedStatus status) {
    if(rgb_led_instance != 0 && rgb_led_instance->status_str != 0) {
        return rgb_led_instance->status_str(status);
    }

    switch(status) {
        #define X(name, value) case RGB_LED_STATUS_##name: return #name;
        RGB_LED_STATUS_TABLE
        #undef X
        default: return "UNKNOWN";
    }
}

RgbLedStatus rgb_led_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if(rgb_led_instance == 0 || rgb_led_instance->set_rgb == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->set_rgb(index, r, g, b);
}

RgbLedStatus rgb_led_fill(uint8_t r, uint8_t g, uint8_t b) {
    if(rgb_led_instance == 0 || rgb_led_instance->fill == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->fill(r, g, b);
}

RgbLedStatus rgb_led_clear(void) {
    if(rgb_led_instance == 0 || rgb_led_instance->clear == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->clear();
}

RgbLedStatus rgb_led_show(void) {
    if(rgb_led_instance == 0 || rgb_led_instance->show == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->show();
}

RgbLedStatus rgb_led_write_complete(void) {
    if(rgb_led_instance == 0 || rgb_led_instance->write_complete == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->write_complete();
}

RgbLedStatus rgb_led_get_color(uint16_t index, RgbLedColor* out) {
    if(rgb_led_instance == 0 || rgb_led_instance->get_color == 0) {
        return RGB_LED_STATUS_NO_INSTANCE;
    }

    return rgb_led_instance->get_color(index, out);
}
