#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include "main.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 声 明 ========================= ! //

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart10;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool uart1_write_blocking(const char* data, uint32_t len);
bool uart10_write_blocking(const char* data, uint32_t len);
bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void));
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void));

#endif
