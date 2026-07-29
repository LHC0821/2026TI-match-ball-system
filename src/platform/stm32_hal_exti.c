#include "stm32_hal_exti.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief EXTI 0~15 号引脚的回调槽
 */
typedef struct {
    STM32HalExtiCallback callback;
    void* user;
} ExtiCallbackSlot;

/**
 * @brief EXTI 0~15 号引脚的回调函数表
 */
static ExtiCallbackSlot exti_callbacks[16] = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将 GPIO 引脚掩码转换为 EXTI 线索引
 * @param gpio_pin GPIO 引脚掩码
 * @return int8_t 成功返回 [0, 15]，失败返回 -1
 */
static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin);

/**
 * @brief 进入临界区并返回之前的中断状态
 * @return uint32_t PRIMASK 状态
 */
static uint32_t exti_enter_critical(void);

/**
 * @brief 退出临界区
 * @param primask 进入临界区前的 PRIMASK 状态
 */
static void exti_exit_critical(uint32_t primask);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 为指定 EXTI 引脚注册回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @param callback 回调函数
 * @param user 用户指针
 * @return STM32HalExtiStatus 状态码
 */
STM32HalExtiStatus exti_register_callback(uint16_t gpio_pin, STM32HalExtiCallback callback, void* user) {
    int8_t pin_index = exti_gpio_pin_to_index(gpio_pin);
    uint32_t primask;

    if(pin_index < 0 || callback == NULL) {
        return STM32_HAL_EXTI_INVALID_PARAM;
    }

    primask = exti_enter_critical();

    if(exti_callbacks[pin_index].callback != NULL
        && exti_callbacks[pin_index].callback != callback) {
        exti_exit_critical(primask);
        return STM32_HAL_EXTI_CALLBACK_OCCUPIED;
    }

    exti_callbacks[pin_index].callback = callback;
    exti_callbacks[pin_index].user = user;
    exti_exit_critical(primask);

    return STM32_HAL_EXTI_OK;
}

/**
 * @brief 注销指定 EXTI 引脚的回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @return STM32HalExtiStatus 状态码
 */
STM32HalExtiStatus exti_unregister_callback(uint16_t gpio_pin) {
    int8_t pin_index = exti_gpio_pin_to_index(gpio_pin);
    uint32_t primask;

    if(pin_index < 0) {
        return STM32_HAL_EXTI_INVALID_PARAM;
    }

    primask = exti_enter_critical();
    exti_callbacks[pin_index].callback = NULL;
    exti_callbacks[pin_index].user = NULL;
    exti_exit_critical(primask);

    return STM32_HAL_EXTI_OK;
}

/**
 * @brief 清空所有 EXTI 回调函数
 */
void exti_clear_callbacks(void) {
    uint32_t primask = exti_enter_critical();

    for(uint8_t i = 0u; i < 16u; i++) {
        exti_callbacks[i].callback = NULL;
        exti_callbacks[i].user = NULL;
    }

    exti_exit_critical(primask);
}

/**
 * @brief 将 EXTI 状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
const char* exti_error_code_to_str(STM32HalExtiStatus status) {
    switch(status) {
#define X(name, str) \
    case STM32_HAL_EXTI_##name: return str;
        STM32_HAL_EXTI_STATUS_TABLE
#undef X
        default: return "Unknown EXTI Status";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL EXTI 中断统一回调入口
 * @param GPIO_Pin 触发中断的 GPIO 引脚掩码
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int8_t pin_index = exti_gpio_pin_to_index(GPIO_Pin);
    STM32HalExtiCallback callback;
    void* user;

    if(pin_index < 0) {
        return;
    }

    callback = exti_callbacks[pin_index].callback;
    user = exti_callbacks[pin_index].user;

    if(callback != NULL) {
        callback(GPIO_Pin, user);
    }
}

/**
 * @brief 将 GPIO 引脚掩码转换为 EXTI 线索引
 * @param gpio_pin GPIO 引脚掩码
 * @return int8_t 成功返回 [0, 15]，失败返回 -1
 */
static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin) {
    if((gpio_pin == 0U) || ((gpio_pin & (gpio_pin - 1U)) != 0U)) {
        return -1;
    }

    for(int8_t pin_index = 0; pin_index < 16; pin_index++) {
        if(gpio_pin == (uint16_t)(1U << pin_index)) {
            return pin_index;
        }
    }

    return -1;
}

/**
 * @brief 进入临界区并返回之前的中断状态
 * @return uint32_t PRIMASK 状态
 */
static uint32_t exti_enter_critical(void) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区
 * @param primask 进入临界区前的 PRIMASK 状态
 */
static void exti_exit_critical(uint32_t primask) {
    if(primask == 0u) {
        __enable_irq();
    }
}
