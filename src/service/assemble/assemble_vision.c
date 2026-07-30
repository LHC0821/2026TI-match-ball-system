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

/**
 * @brief assemble_vision_write 内部辅助函数
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool assemble_vision_write(const char* data, uint32_t len);
/**
 * @brief assemble_vision_now_ms 内部辅助函数
 * @return 返回值
 */
static uint32_t assemble_vision_now_ms(void);
/**
 * @brief assemble_vision_rx_complete 内部辅助函数
 */
static void assemble_vision_rx_complete(void);
/**
 * @brief assemble_vision_error 内部辅助函数
 */
static void assemble_vision_error(void);
/**
 * @brief assemble_vision_start_rx 内部辅助函数
 */
static void assemble_vision_start_rx(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_vision(void) {
    VisionCommsConfig config = {
        .port_ops = {
            .write = assemble_vision_write,
            .now_ms = assemble_vision_now_ms,
        },
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
    return uart10_write_it(data, len);
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
