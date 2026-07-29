#ifndef _stm32_hal_bmi088_h_
#define _stm32_hal_bmi088_h_

#include <stdint.h>

#include "imu/bmi088.h"
#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief stm32_bmi088_get_ops 接口函数
 * @return 结果指针
 */
const Bmi088PortOps* stm32_bmi088_get_ops(void);
/**
 * @brief stm32_bmi088_get_accel_int_pin 接口函数
 * @return 返回值
 */
uint16_t stm32_bmi088_get_accel_int_pin(void);
/**
 * @brief stm32_bmi088_get_gyro_int_pin 接口函数
 * @return 返回值
 */
uint16_t stm32_bmi088_get_gyro_int_pin(void);

/**
 * @brief 注册 BMI088 所需的 EXTI 和 SPI 回调
 *
 * 将板级 wiring 下沉到平台层，避免组装层直接依赖底层回调注册细节
 */
void stm32_bmi088_register_callbacks(void);
/**
 * @brief stm32_bmi088_spi_txrx_complete_callback 接口函数
 * @param hspi SPI 句柄指针
 */
void stm32_bmi088_spi_txrx_complete_callback(SPI_HandleTypeDef* hspi);
/**
 * @brief stm32_bmi088_spi_error_callback 接口函数
 * @param hspi SPI 句柄指针
 */
void stm32_bmi088_spi_error_callback(SPI_HandleTypeDef* hspi);

#endif
