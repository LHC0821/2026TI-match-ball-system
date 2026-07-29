/**
 * @file assemble_vision.c
 * @brief USART10 视觉通信装配
 */

#include "assemble.h"

#include "delay.h"
#include "log.h"
#include "stm32_hal_uart.h"
#include "vision_comms.h"

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_rx_byte = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool assemble_vision_write(const char* data, uint32_t len);
static uint32_t assemble_vision_now_ms(void);
static void assemble_vision_rx_complete(void);
static void assemble_vision_error(void);
static void assemble_vision_start_rx(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_vision(void) {
    VisionCommsConfig config = {
        .port_ops = {
            .write = assemble_vision_write,
            .now_ms = assemble_vision_now_ms,
        },
        .online_timeout_ms = 500u,
    };

    if(vision_comms_init(&config) != VISION_COMMS_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(&huart10, assemble_vision_rx_complete);
    uart_register_error_callback(&huart10, assemble_vision_error);
    assemble_vision_start_rx();
    log_info("VISION USART10 ready: 1000000 8N1");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool assemble_vision_write(const char* data, uint32_t len) {
    return uart10_write_blocking(data, len);
}

static uint32_t assemble_vision_now_ms(void) {
    return delay_now_ms();
}

static void assemble_vision_rx_complete(void) {
    vision_comms_on_rx_byte(s_rx_byte);
    assemble_vision_start_rx();
}

static void assemble_vision_error(void) {
    assemble_vision_start_rx();
}

static void assemble_vision_start_rx(void) {
    (void)uart_receive_it(&huart10, &s_rx_byte, 1u);
}
