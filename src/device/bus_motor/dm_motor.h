#ifndef _dm_motor_h_
#define _dm_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file dm_motor.h
 * @brief 达妙电机总线驱动接口
 */

/**
 * @brief 达妙电机命令帧长度，单位 byte
 */
#define DM_MOTOR_CMD_LEN 8u

/**
 * @brief 达妙电机支持的最大协议 ID
 */
#define DM_MOTOR_MAX_ID 32u

/**
 * @brief DM-4340 MIT 模式推荐位置环比例系数
 */
#define DM_MOTOR_DEFAULT_KP 4.0f

/**
 * @brief DM-4340 MIT 模式推荐位置环微分系数
 */
#define DM_MOTOR_DEFAULT_KD 0.04f

/**
 * @brief DM-4340 MIT 模式固定前馈扭矩
 */
#define DM_MOTOR_DEFAULT_TORQUE 0.0f

/**
 * @brief 达妙电机型号
 */
typedef enum {
    DM_MOTOR_MODEL_4340 = 0u, /**< DM-4340，MIT 量程为 ±12.5rad、±8rad/s、±28N*m */
} DmMotorModel;

/**
 * @brief 达妙电机驱动私有配置
 */
typedef struct {
    DmMotorModel model; /**< 电机型号 */
    uint16_t master_id; /**< 电机反馈使用的主控 CAN ID */
} DmMotorConfig;

/**
 * @brief 达妙电机控制模式
 */
typedef enum {
    DM_MOTOR_MODE_MIT = 1u,         /**< MIT 力控模式 */
    DM_MOTOR_MODE_POS_VEL = 2u,     /**< 位置-速度模式 */
    DM_MOTOR_MODE_SPEED = 3u,       /**< 速度模式 */
    DM_MOTOR_MODE_POS_VEL_TOR = 4u, /**< 位置-速度-前馈扭矩模式 */
} DmMotorMode;

/**
 * @brief 达妙电机统一接口实例
 */
extern const BusMotorInterface dm_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 生成 DM-4340 默认配置
 * @param config 输出配置
 * @param master_id 电机反馈使用的主控 CAN ID
 * @return 电机状态码
 */
BusMotorStatus dm_motor_make_4340_config(DmMotorConfig* config, uint16_t master_id);

/**
 * @brief 解析达妙电机反馈帧
 * @param frame_id CAN 反馈帧 ID
 * @param data 8 字节反馈数据
 * @param feedback 输出通用电机反馈
 * @return 电机状态码
 */
BusMotorStatus dm_motor_parse_feedback_frame(uint32_t frame_id,
                                             const uint8_t data[DM_MOTOR_CMD_LEN],
                                             BusMotorFeedback* feedback);

/**
 * @brief 清除指定达妙电机错误
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus dm_motor_clear_error(uint16_t id);

/**
 * @brief 保存指定达妙电机当前位置为零点
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus dm_motor_save_zero(uint16_t id);

#endif
