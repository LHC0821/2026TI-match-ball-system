/**
 * @file assemble_motor.c
 * @brief DM-4340 摆杆电机装配
 */

#include "assemble.h"

#include "balance.h"
#include "bus_motor/bus_motor.h"
#include "bus_motor/dm_motor.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BAL_MOTOR_ID 0x01u
#define BAL_MASTER_ID 0x11u

// ! ========================= 变 量 声 明 ========================= ! //

static DmMotorConfig s_dm_config = { 0 };

static BusMotorPortOps s_motor_ops = {
    .send = 0,
    .read = 0,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
    .flush_rx = 0,
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief assemble_motor_can_send 内部辅助函数
 * @param id 设备或协议 ID
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
static bool assemble_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len);
/**
 * @brief assemble_motor_can_rx 内部辅助函数
 * @param hcan FDCAN 句柄指针
 * @param header 参数值
 * @param data 数据缓冲区指针
 * @param user 用户上下文指针
 */
static void assemble_motor_can_rx(FDCAN_HandleTypeDef* hcan,
                                  const FDCAN_RxHeaderTypeDef* header,
                                  const uint8_t data[8],
                                  void* user);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_motor(void) {
    BusMotorConfig config;

    s_motor_ops.send = assemble_motor_can_send;

    if(bus_motor_set_instance(&dm_motor_instance) != MOTOR_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    if(dm_motor_make_4340_config(&s_dm_config, BAL_MASTER_ID) != MOTOR_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    config.ops = &s_motor_ops;
    config.timeout_ms = 0u;
    config.retry_count = 0u;
    config.driver_config = &s_dm_config;
    if(bus_motor_init(&config) != MOTOR_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    if(can_register_rx_callback(&hfdcan1, assemble_motor_can_rx, 0) != STM32_HAL_CAN_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    if(can_filter_init(&hfdcan1) != STM32_HAL_CAN_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("MOTOR DM-4340 bus ready: id=0x%02X master=0x%02X",
             BAL_MOTOR_ID, BAL_MASTER_ID);
    return SYSTEM_STATUS_OK;
}

bool assemble_motor_prepare(uint16_t id) {
    BusMotorStatus status;

    if(id != BAL_MOTOR_ID) {
        log_error("MOTOR prepare rejected: id=0x%03X", id);
        return false;
    }

    status = bus_motor.disable(id);
    if(status != MOTOR_STATUS_OK) {
        log_error("MOTOR disable failed: %s", bus_motor.status_str(status));
        return false;
    }
    delay_ms(20u);

    status = dm_motor_clear_error(id);
    if(status != MOTOR_STATUS_OK) {
        log_error("MOTOR clear failed: %s", bus_motor.status_str(status));
        return false;
    }
    delay_ms(20u);

    status = bus_motor.switch_mode(id, DM_MOTOR_MODE_POS_VEL);
    if(status != MOTOR_STATUS_OK) {
        log_error("MOTOR POS_VEL switch failed: %s",
                  bus_motor.status_str(status));
        return false;
    }

    log_info("MOTOR DM-4340 ready: mode=POS_VEL");
    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool assemble_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}

static void assemble_motor_can_rx(FDCAN_HandleTypeDef* hcan,
                                  const FDCAN_RxHeaderTypeDef* header,
                                  const uint8_t data[8],
                                  void* user) {
    BusMotorFeedback feedback;

    (void)hcan;
    (void)user;

    if(header == 0 || header->Identifier != BAL_MASTER_ID) {
        return;
    }

    if(dm_motor_parse_parameter_frame(header->Identifier, data)) {
        return;
    }

    if(dm_motor_parse_feedback_frame(header->Identifier, data, &feedback) == MOTOR_STATUS_OK &&
       feedback.id == BAL_MOTOR_ID) {
        balance_on_motor_feedback(&feedback, HAL_GetTick());
    }
}
