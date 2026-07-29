#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include "main.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 声 明 ========================= ! //

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart10;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief uart1_write_blocking 接口函数
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool uart1_write_blocking(const char* data, uint32_t len);
/**
 * @brief uart10_write_blocking 接口函数
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool uart10_write_blocking(const char* data, uint32_t len);
/**
 * @brief uart_receive_it 接口函数
 * @param huart UART 句柄指针
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);
/**
 * @brief uart_register_rx_complete_callback 接口函数
 * @param huart UART 句柄指针
 * @param callback 回调函数指针
 */
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void));
/**
 * @brief uart_register_error_callback 接口函数
 * @param huart UART 句柄指针
 * @param callback 回调函数指针
 */
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void));

#endif
