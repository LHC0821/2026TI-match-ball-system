#include "stm32_hal_uart.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

static void (*s_uart1_rx_complete_callback)(void) = NULL;
static void (*s_uart1_error_callback)(void) = NULL;
static void (*s_uart10_rx_complete_callback)(void) = NULL;
static void (*s_uart10_error_callback)(void) = NULL;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool uart1_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 10u) == HAL_OK;
}

bool uart10_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart10, (uint8_t*)data, (uint16_t)len, 10u) == HAL_OK;
}

bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    return HAL_UART_Receive_IT(huart, data, len) == HAL_OK;
}

void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        s_uart1_rx_complete_callback = callback;
    }
    else if(huart == &huart10) {
        s_uart10_rx_complete_callback = callback;
    }
}

void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        s_uart1_error_callback = callback;
    }
    else if(huart == &huart10) {
        s_uart10_error_callback = callback;
    }
}

// ! ========================= HAL 回 调 实 现 ========================= ! //

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && s_uart1_rx_complete_callback != NULL) {
        s_uart1_rx_complete_callback();
    }
    else if(huart == &huart10 && s_uart10_rx_complete_callback != NULL) {
        s_uart10_rx_complete_callback();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && s_uart1_error_callback != NULL) {
        s_uart1_error_callback();
    }
    else if(huart == &huart10 && s_uart10_error_callback != NULL) {
        s_uart10_error_callback();
    }
}
