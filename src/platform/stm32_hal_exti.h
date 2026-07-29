#ifndef _stm32_hal_exti_h_
#define _stm32_hal_exti_h_

#include <stdint.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief STM32 HAL EXTI 抽象层状态码表
 */
#define STM32_HAL_EXTI_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(CALLBACK_OCCUPIED, "Callback Occupied")

/**
 * @brief STM32 HAL EXTI 抽象层状态码
 */
#define X(name, str) STM32_HAL_EXTI_##name,
typedef enum {
    STM32_HAL_EXTI_STATUS_TABLE
} STM32HalExtiStatus;
#undef X

/**
 * @brief EXTI 回调函数类型
 * @param gpio_pin 触发中断的 GPIO 引脚掩码
 * @param user 注册时绑定的用户指针
 */
typedef void (*STM32HalExtiCallback)(uint16_t gpio_pin, void* user);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 为指定 EXTI 引脚注册回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @param callback 回调函数
 * @param user 用户指针
 * @return STM32HalExtiStatus 状态码
 */
STM32HalExtiStatus exti_register_callback(uint16_t gpio_pin, STM32HalExtiCallback callback, void* user);

/**
 * @brief 注销指定 EXTI 引脚的回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @return STM32HalExtiStatus 状态码
 */
STM32HalExtiStatus exti_unregister_callback(uint16_t gpio_pin);

/**
 * @brief 清空所有 EXTI 回调函数
 */
void exti_clear_callbacks(void);

/**
 * @brief 将 EXTI 状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
const char* exti_error_code_to_str(STM32HalExtiStatus status);

#endif
