#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief SystemStatus 枚举类型
 */
typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;

extern volatile uint32_t tim6_500hz_pending;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief assemble_delay 接口函数
 * @return 状态码
 */
SystemStatus assemble_delay(void);
/**
 * @brief assemble_log 接口函数
 * @return 状态码
 */
SystemStatus assemble_log(void);
/**
 * @brief assemble_rgb 接口函数
 * @return 状态码
 */
SystemStatus assemble_rgb(void);
/**
 * @brief assemble_imu 接口函数
 * @return 状态码
 */
SystemStatus assemble_imu(void);
/**
 * @brief assemble_motor 接口函数
 * @return 状态码
 */
SystemStatus assemble_motor(void);
/**
 * @brief IMU 校准完成后准备摆杆电机
 * @param id 电机协议 ID
 * @return true 表示失能、清错和 MIT 模式确认全部成功
 */
bool assemble_motor_prepare(uint16_t id);
/**
 * @brief assemble_vision 接口函数
 * @return 状态码
 */
SystemStatus assemble_vision(void);
/**
 * @brief assemble_balance 接口函数
 * @return 状态码
 */
SystemStatus assemble_balance(void);
/**
 * @brief assemble_tim6_500hz 接口函数
 * @return 状态码
 */
SystemStatus assemble_tim6_500hz(void);

#endif
