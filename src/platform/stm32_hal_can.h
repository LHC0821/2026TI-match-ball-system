#ifndef _stm32_hal_can_h_
#define _stm32_hal_can_h_

#include "main.h" // IWYU pragma: keep

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

extern FDCAN_HandleTypeDef hfdcan1;

#define STM32_HAL_CAN_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_DLC, "Invalid DLC") \
    X(FILTER_CONFIG_FAILED, "CAN Filter Config Failed") \
    X(START_FAILED, "CAN Start Failed") \
    X(NOTIFICATION_FAILED, "CAN Notification Activation Failed") \
    X(TX_MAILBOX_TIMEOUT, "CAN TX Mailbox Timeout") \
    X(TX_FAILED, "CAN TX Failed") \
    X(RX_FAILED, "CAN RX Failed") \
    X(NO_CALLBACK_SLOT, "No CAN RX Callback Slot")

#define X(name, str) STM32_HAL_CAN_##name,
/**
 * @brief BspCanStatus 枚举类型
 */
typedef enum {
    STM32_HAL_CAN_STATUS_TABLE
} BspCanStatus;
#undef X

/**
 * @brief void 接口函数
 * @param user 用户上下文指针
 * @return 返回值
 */
typedef void (*STM32HalCanRxCallback)(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief can_filter_init 接口函数
 * @param hcan FDCAN 句柄指针
 * @return 状态码
 */
BspCanStatus can_filter_init(FDCAN_HandleTypeDef* hcan);
/**
 * @brief can_start 接口函数
 * @param hcan FDCAN 句柄指针
 * @return 状态码
 */
BspCanStatus can_start(FDCAN_HandleTypeDef* hcan);
/**
 * @brief can_send 接口函数
 * @param hcan FDCAN 句柄指针
 * @param id 设备或协议 ID
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return 状态码
 */
BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);
/**
 * @brief can_register_rx_callback 接口函数
 * @param hcan FDCAN 句柄指针
 * @param callback 回调函数指针
 * @param user 用户上下文指针
 * @return 状态码
 */
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan, STM32HalCanRxCallback callback, void* user);
/**
 * @brief can_error_code_to_str 接口函数
 * @param status 状态值
 * @return 字符串指针
 */
const char* can_error_code_to_str(BspCanStatus status);

#endif
