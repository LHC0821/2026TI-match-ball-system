#include "dm_motor.h"

#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define DM_MOTOR_4340_POS_LIMIT 12.5664f
#define DM_MOTOR_4340_SPD_LIMIT 10.0f
#define DM_MOTOR_4340_TOR_LIMIT 28.0f
#define DM_MOTOR_KP_LIMIT 500.0f
#define DM_MOTOR_KD_LIMIT 5.0f

#define DM_MOTOR_CTRL_FRAME_ID_POS_VEL      0x100u
#define DM_MOTOR_CTRL_FRAME_ID_SPEED        0x200u
#define DM_MOTOR_CTRL_FRAME_ID_POS_VEL_TOR  0x300u
#define DM_MOTOR_REG_WRITE_FRAME_ID         0x7FFu
#define DM_MOTOR_REG_CMD_WRITE              0x55u
#define DM_MOTOR_REG_MODE_ID                10u
#define DM_MOTOR_MODE_ACK_TIMEOUT_MS         100u

typedef struct {
    DmMotorMode mode;
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
    bool has_feedback;
    BusMotorFeedback feedback;
} DmMotorSlot;

static const BusMotorPortOps* s_ops = 0;
static bool s_is_initialized = false;
static uint16_t s_master_id = 0u;
static float s_pos_limit = DM_MOTOR_4340_POS_LIMIT;
static float s_spd_limit = DM_MOTOR_4340_SPD_LIMIT;
static float s_tor_limit = DM_MOTOR_4340_TOR_LIMIT;
static DmMotorSlot s_slots[DM_MOTOR_MAX_ID];
static volatile uint32_t s_mode_ack_sequence = 0u;
static volatile uint16_t s_mode_ack_id = 0u;
static volatile uint32_t s_mode_ack_value = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief dm_motor_init 内部辅助函数
 * @param config 配置结构体指针
 * @return 状态码
 */
static BusMotorStatus dm_motor_init(const BusMotorConfig* config);
/**
 * @brief dm_motor_status_str 内部辅助函数
 * @param status 状态值
 * @return 字符串指针
 */
static const char* dm_motor_status_str(BusMotorStatus status);
/**
 * @brief dm_motor_mode_str 内部辅助函数
 * @param mode 工作模式
 * @return 字符串指针
 */
static const char* dm_motor_mode_str(BusMotorMode mode);
/**
 * @brief dm_motor_enable 内部辅助函数
 * @param id 设备或协议 ID
 * @return 状态码
 */
static BusMotorStatus dm_motor_enable(uint16_t id);
/**
 * @brief dm_motor_disable 内部辅助函数
 * @param id 设备或协议 ID
 * @return 状态码
 */
static BusMotorStatus dm_motor_disable(uint16_t id);
/**
 * @brief dm_motor_switch_mode 内部辅助函数
 * @param id 设备或协议 ID
 * @param mode 工作模式
 * @return 状态码
 */
static BusMotorStatus dm_motor_switch_mode(uint16_t id, BusMotorMode mode);
/**
 * @brief dm_motor_set_pos 内部辅助函数
 * @param id 设备或协议 ID
 * @param position 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_set_pos(uint16_t id, float position);
/**
 * @brief dm_motor_set_spd 内部辅助函数
 * @param id 设备或协议 ID
 * @param speed 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_set_spd(uint16_t id, float speed);
/**
 * @brief dm_motor_set_pos_vel 内部辅助函数
 * @param id 设备或协议 ID
 * @param position 参数值
 * @param speed 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_set_pos_vel(uint16_t id, float position, float speed);
/**
 * @brief dm_motor_set_tor 内部辅助函数
 * @param id 设备或协议 ID
 * @param torque 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_set_tor(uint16_t id, float torque);
/**
 * @brief dm_motor_set_pd 内部辅助函数
 * @param id 设备或协议 ID
 * @param kp 参数值
 * @param kd 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_set_pd(uint16_t id, float kp, float kd);
/**
 * @brief dm_motor_update_feedback 内部辅助函数
 * @param id 设备或协议 ID
 * @param feedback 反馈数据指针
 * @return 状态码
 */
static BusMotorStatus dm_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);
/**
 * @brief dm_motor_get_pos 内部辅助函数
 * @param id 设备或协议 ID
 * @return 返回值
 */
static float dm_motor_get_pos(uint16_t id);
/**
 * @brief dm_motor_get_spd 内部辅助函数
 * @param id 设备或协议 ID
 * @return 返回值
 */
static float dm_motor_get_spd(uint16_t id);
/**
 * @brief dm_motor_get_tor 内部辅助函数
 * @param id 设备或协议 ID
 * @return 返回值
 */
static float dm_motor_get_tor(uint16_t id);
/**
 * @brief dm_motor_stop 内部辅助函数
 * @param id 设备或协议 ID
 * @return 状态码
 */
static BusMotorStatus dm_motor_stop(uint16_t id);
/**
 * @brief dm_motor_brake 内部辅助函数
 * @param id 设备或协议 ID
 * @return 状态码
 */
static BusMotorStatus dm_motor_brake(uint16_t id);

/**
 * @brief dm_motor_get_slot 内部辅助函数
 * @param id 设备或协议 ID
 * @return 结果指针
 */
static DmMotorSlot* dm_motor_get_slot(uint16_t id);
/**
 * @brief dm_motor_get_slot_const 内部辅助函数
 * @param id 设备或协议 ID
 * @return 结果指针
 */
static const DmMotorSlot* dm_motor_get_slot_const(uint16_t id);
/**
 * @brief dm_motor_reset_slot 内部辅助函数
 * @param slot 参数值
 * @param id 设备或协议 ID
 */
static void dm_motor_reset_slot(DmMotorSlot* slot, uint16_t id);
/**
 * @brief dm_motor_send 内部辅助函数
 * @param id 设备或协议 ID
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return 状态码
 */
static BusMotorStatus dm_motor_send(uint32_t id, const uint8_t* data, uint8_t len);
/**
 * @brief dm_motor_send_delayed 内部辅助函数
 * @param id 设备或协议 ID
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return 状态码
 */
static BusMotorStatus dm_motor_send_delayed(uint32_t id, const uint8_t* data, uint8_t len);
/**
 * @brief dm_motor_write_mode 内部辅助函数
 * @param id 设备或协议 ID
 * @param mode 工作模式
 * @return 状态码
 */
static BusMotorStatus dm_motor_write_mode(uint16_t id, DmMotorMode mode);
/**
 * @brief dm_motor_apply_command 内部辅助函数
 * @param id 设备或协议 ID
 * @return 状态码
 */
static BusMotorStatus dm_motor_apply_command(uint16_t id);
/**
 * @brief dm_motor_send_pos_vel 内部辅助函数
 * @param id 设备或协议 ID
 * @param slot 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_send_pos_vel(uint16_t id, const DmMotorSlot* slot);
/**
 * @brief dm_motor_send_speed 内部辅助函数
 * @param id 设备或协议 ID
 * @param slot 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_send_speed(uint16_t id, const DmMotorSlot* slot);
/**
 * @brief dm_motor_send_mit 内部辅助函数
 * @param id 设备或协议 ID
 * @param slot 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_send_mit(uint16_t id, const DmMotorSlot* slot);
/**
 * @brief dm_motor_send_pos_vel_tor 内部辅助函数
 * @param id 设备或协议 ID
 * @param slot 参数值
 * @return 状态码
 */
static BusMotorStatus dm_motor_send_pos_vel_tor(uint16_t id, const DmMotorSlot* slot);
/**
 * @brief dm_motor_f32_to_u16 内部辅助函数
 * @param val 参数值
 * @param min 参数值
 * @param max 参数值
 * @param bits 参数值
 * @return 返回值
 */
static uint16_t dm_motor_f32_to_u16(float val, float min, float max, uint8_t bits);
/**
 * @brief dm_motor_u16_to_f32 内部辅助函数
 * @param val 参数值
 * @param min 参数值
 * @param max 参数值
 * @param bits 参数值
 * @return 返回值
 */
static float dm_motor_u16_to_f32(uint16_t val, float min, float max, uint8_t bits);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BusMotorStatus dm_motor_make_4340_config(DmMotorConfig* config, uint16_t master_id) {
    if(config == 0 || master_id > 0x7FFu) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    config->model = DM_MOTOR_MODEL_4340;
    config->master_id = master_id;
    return MOTOR_STATUS_OK;
}

const BusMotorInterface dm_motor_instance = {
    .init = dm_motor_init,
    .status_str = dm_motor_status_str,
    .mode_str = dm_motor_mode_str,
    .enable = dm_motor_enable,
    .disable = dm_motor_disable,
    .switch_mode = dm_motor_switch_mode,
    .set_pos = dm_motor_set_pos,
    .set_spd = dm_motor_set_spd,
    .set_pos_vel = dm_motor_set_pos_vel,
    .set_tor = dm_motor_set_tor,
    .set_pd = dm_motor_set_pd,
    .update_feedback = dm_motor_update_feedback,
    .get_pos = dm_motor_get_pos,
    .get_spd = dm_motor_get_spd,
    .get_tor = dm_motor_get_tor,
    .stop = dm_motor_stop,
    .brake = dm_motor_brake,
};

/**
 * @brief 初始化达妙电机实例
 */
static BusMotorStatus dm_motor_init(const BusMotorConfig* config) {
    const DmMotorConfig* driver_config;
    uint16_t id;

    if(config == 0 || config->ops == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(config->ops->send == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    driver_config = (const DmMotorConfig*)config->driver_config;
    if(driver_config == 0 || driver_config->model != DM_MOTOR_MODEL_4340 || driver_config->master_id > 0x7FFu) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    s_ops = config->ops;
    s_master_id = driver_config->master_id;
    s_pos_limit = DM_MOTOR_4340_POS_LIMIT;
    s_spd_limit = DM_MOTOR_4340_SPD_LIMIT;
    s_tor_limit = DM_MOTOR_4340_TOR_LIMIT;
    s_mode_ack_sequence = 0u;
    s_mode_ack_id = 0u;
    s_mode_ack_value = 0u;
    for(id = 1u; id <= DM_MOTOR_MAX_ID; ++id) {
        dm_motor_reset_slot(&s_slots[id - 1u], id);
    }
    s_is_initialized = true;

    return MOTOR_STATUS_OK;
}

/**
 * @brief 将状态码转换为常量字符串
 */
static const char* dm_motor_status_str(BusMotorStatus status) {
    switch(status) {
#define X(name, value) case MOTOR_STATUS_##name: return #name;
        MOTOR_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

/**
 * @brief 将模式值转换为常量字符串
 */
static const char* dm_motor_mode_str(BusMotorMode mode) {
    switch((DmMotorMode)mode) {
        case DM_MOTOR_MODE_MIT: return "MIT";
        case DM_MOTOR_MODE_POS_VEL: return "POS_VEL";
        case DM_MOTOR_MODE_SPEED: return "SPEED";
        case DM_MOTOR_MODE_POS_VEL_TOR: return "POS_VEL_TOR";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 使能达妙电机
 */
static BusMotorStatus dm_motor_enable(uint16_t id) {
    static const uint8_t data[DM_MOTOR_CMD_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };

    if(dm_motor_get_slot_const(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return dm_motor_send_delayed(id, data, DM_MOTOR_CMD_LEN);
}

/**
 * @brief 失能达妙电机
 */
static BusMotorStatus dm_motor_disable(uint16_t id) {
    static const uint8_t data[DM_MOTOR_CMD_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };

    if(dm_motor_get_slot_const(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return dm_motor_send_delayed(id, data, DM_MOTOR_CMD_LEN);
}

/**
 * @brief 切换达妙电机模式
 */
static BusMotorStatus dm_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    DmMotorSlot* slot = dm_motor_get_slot(id);
    BusMotorStatus status;
    uint32_t ack_sequence;
    uint32_t elapsed_ms;

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    switch((DmMotorMode)mode) {
        case DM_MOTOR_MODE_MIT:
        case DM_MOTOR_MODE_POS_VEL:
        case DM_MOTOR_MODE_SPEED:
        case DM_MOTOR_MODE_POS_VEL_TOR: break;
        default: return MOTOR_STATUS_UNSUPPORTED;
    }

    ack_sequence = s_mode_ack_sequence;
    status = dm_motor_write_mode(id, (DmMotorMode)mode);

    if(status != MOTOR_STATUS_OK) {
        return status;
    }

    if(s_ops == 0 || s_ops->delay_ms == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    for(elapsed_ms = 0u; elapsed_ms < DM_MOTOR_MODE_ACK_TIMEOUT_MS; ++elapsed_ms) {
        if(s_mode_ack_sequence != ack_sequence &&
           s_mode_ack_id == id &&
           s_mode_ack_value == (uint32_t)mode) {
            slot->mode = (DmMotorMode)mode;
            return MOTOR_STATUS_OK;
        }
        s_ops->delay_ms(1u);
    }

    return MOTOR_STATUS_TIMEOUT;
}

/**
 * @brief 设定目标位置
 */
static BusMotorStatus dm_motor_set_pos(uint16_t id, float position) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->position = position;
    return dm_motor_apply_command(id);
}

/**
 * @brief 设定目标速度
 */
static BusMotorStatus dm_motor_set_spd(uint16_t id, float speed) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->speed = speed;
    return dm_motor_apply_command(id);
}

/**
 * @brief 设定目标位置和速度
 */
static BusMotorStatus dm_motor_set_pos_vel(uint16_t id, float position, float speed) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->position = position;
    slot->speed = speed;
    return dm_motor_apply_command(id);
}

/**
 * @brief 设定目标扭矩或前馈
 */
static BusMotorStatus dm_motor_set_tor(uint16_t id, float torque) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->torque = torque;
    return dm_motor_apply_command(id);
}

/**
 * @brief 设定位置环 PD 参数
 */
static BusMotorStatus dm_motor_set_pd(uint16_t id, float kp, float kd) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->kp = kp;
    slot->kd = kd;
    return dm_motor_apply_command(id);
}

/**
 * @brief 从反馈缓存读取最近反馈
 */
static BusMotorStatus dm_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    const DmMotorSlot* slot = dm_motor_get_slot_const(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(slot->has_feedback == false) {
        return MOTOR_STATUS_NO_FEEDBACK;
    }

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 获取最近位置反馈
 */
static float dm_motor_get_pos(uint16_t id) {
    const DmMotorSlot* slot = dm_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.position;
}

/**
 * @brief 获取最近速度反馈
 */
static float dm_motor_get_spd(uint16_t id) {
    const DmMotorSlot* slot = dm_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.speed;
}

/**
 * @brief 获取最近扭矩反馈
 */
static float dm_motor_get_tor(uint16_t id) {
    const DmMotorSlot* slot = dm_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.torque;
}

/**
 * @brief 停止电机
 */
static BusMotorStatus dm_motor_stop(uint16_t id) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->speed = 0.0f;
    slot->torque = 0.0f;
    return dm_motor_apply_command(id);
}

/**
 * @brief 制动电机
 */
static BusMotorStatus dm_motor_brake(uint16_t id) {
    DmMotorSlot* slot = dm_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if(slot->has_feedback) {
        slot->position = slot->feedback.position;
    }
    slot->speed = 0.0f;
    slot->torque = 0.0f;
    return dm_motor_apply_command(id);
}

/**
 * @brief 解析一帧达妙反馈并刷新本地缓存
 */
BusMotorStatus dm_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t data[DM_MOTOR_CMD_LEN],
    BusMotorFeedback* feedback) {
    BusMotorFeedback parsed = { 0 };
    DmMotorSlot* slot;
    uint16_t pos_bits;
    uint16_t spd_bits;
    uint16_t tor_bits;

    if(data == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(frame_id != s_master_id) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    parsed.id = (uint16_t)(data[0] & 0x0Fu);
    if(parsed.id == 0u) {
        parsed.id = (uint16_t)(frame_id & 0x0Fu);
    }
    if(parsed.id == 0u || parsed.id > DM_MOTOR_MAX_ID) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    parsed.error_code = (uint8_t)(data[0] >> 4);
    pos_bits = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    spd_bits = (uint16_t)(((uint16_t)data[3] << 4) | (((uint16_t)data[4] & 0xF0u) >> 4));
    tor_bits = (uint16_t)((((uint16_t)data[4] & 0x0Fu) << 8) | data[5]);

    parsed.position = dm_motor_u16_to_f32(pos_bits, -s_pos_limit, s_pos_limit, 16);
    parsed.speed = dm_motor_u16_to_f32(spd_bits, -s_spd_limit, s_spd_limit, 12);
    parsed.torque = dm_motor_u16_to_f32(tor_bits, -s_tor_limit, s_tor_limit, 12);

    slot = dm_motor_get_slot(parsed.id);
    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->feedback = parsed;
    slot->has_feedback = true;

    if(feedback != 0) {
        *feedback = parsed;
    }

    return MOTOR_STATUS_OK;
}

bool dm_motor_parse_parameter_frame(uint32_t frame_id,
                                    const uint8_t data[DM_MOTOR_CMD_LEN]) {
    uint16_t id;
    uint32_t value;

    if(data == 0 || frame_id != s_master_id) {
        return false;
    }
    if(data[2] != 0x33u && data[2] != DM_MOTOR_REG_CMD_WRITE) {
        return false;
    }

    id = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    if(id == 0u || id > DM_MOTOR_MAX_ID) {
        return true;
    }

    if(data[3] == DM_MOTOR_REG_MODE_ID) {
        value = (uint32_t)data[4] |
                ((uint32_t)data[5] << 8) |
                ((uint32_t)data[6] << 16) |
                ((uint32_t)data[7] << 24);
        s_mode_ack_id = id;
        s_mode_ack_value = value;
        s_mode_ack_sequence++;
    }

    return true;
}

/**
 * @brief 清除达妙电机错误
 */
BusMotorStatus dm_motor_clear_error(uint16_t id) {
    static const uint8_t data[DM_MOTOR_CMD_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB };

    if(dm_motor_get_slot_const(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return dm_motor_send_delayed(id, data, DM_MOTOR_CMD_LEN);
}

/**
 * @brief 保存当前零位
 */
BusMotorStatus dm_motor_save_zero(uint16_t id) {
    static const uint8_t data[DM_MOTOR_CMD_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };

    if(dm_motor_get_slot_const(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return dm_motor_send_delayed(id, data, DM_MOTOR_CMD_LEN);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static DmMotorSlot* dm_motor_get_slot(uint16_t id) {
    if(id == 0u || id > DM_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static const DmMotorSlot* dm_motor_get_slot_const(uint16_t id) {
    if(id == 0u || id > DM_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static void dm_motor_reset_slot(DmMotorSlot* slot, uint16_t id) {
    if(slot == 0) {
        return;
    }

    memset(slot, 0, sizeof(*slot));
    slot->mode = DM_MOTOR_MODE_POS_VEL;
    slot->kp = DM_MOTOR_DEFAULT_KP;
    slot->kd = DM_MOTOR_DEFAULT_KD;
    slot->torque = DM_MOTOR_DEFAULT_TORQUE;
    slot->feedback.id = id;
}

static BusMotorStatus dm_motor_send(uint32_t id, const uint8_t* data, uint8_t len) {
    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(s_ops == 0 || s_ops->send == 0 || data == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }
    if(s_ops->send(id, data, len) == false) {
        return MOTOR_STATUS_PORT_ERROR;
    }
    return MOTOR_STATUS_OK;
}

static BusMotorStatus dm_motor_send_delayed(uint32_t id, const uint8_t* data, uint8_t len) {
    BusMotorStatus status = dm_motor_send(id, data, len);

    if(status == MOTOR_STATUS_OK && s_ops != 0 && s_ops->delay_ms != 0) {
        s_ops->delay_ms(1u);
    }

    return status;
}

static BusMotorStatus dm_motor_write_mode(uint16_t id, DmMotorMode mode) {
    uint8_t data[DM_MOTOR_CMD_LEN];

    data[0] = (uint8_t)(id & 0xFFu);
    data[1] = (uint8_t)((id >> 8) & 0x07u);
    data[2] = DM_MOTOR_REG_CMD_WRITE;
    data[3] = DM_MOTOR_REG_MODE_ID;
    data[4] = (uint8_t)mode;
    data[5] = 0u;
    data[6] = 0u;
    data[7] = 0u;

    return dm_motor_send_delayed(DM_MOTOR_REG_WRITE_FRAME_ID, data, DM_MOTOR_CMD_LEN);
}

static BusMotorStatus dm_motor_apply_command(uint16_t id) {
    const DmMotorSlot* slot = dm_motor_get_slot_const(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    switch(slot->mode) {
        case DM_MOTOR_MODE_MIT: return dm_motor_send_mit(id, slot);
        case DM_MOTOR_MODE_POS_VEL: return dm_motor_send_pos_vel(id, slot);
        case DM_MOTOR_MODE_SPEED: return dm_motor_send_speed(id, slot);
        case DM_MOTOR_MODE_POS_VEL_TOR: return dm_motor_send_pos_vel_tor(id, slot);
        default: return MOTOR_STATUS_UNSUPPORTED;
    }
}

static BusMotorStatus dm_motor_send_pos_vel(uint16_t id, const DmMotorSlot* slot) {
    uint8_t data[DM_MOTOR_CMD_LEN];

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    memcpy(&data[0], &slot->position, sizeof(float));
    memcpy(&data[4], &slot->speed, sizeof(float));
    return dm_motor_send((uint32_t)id + DM_MOTOR_CTRL_FRAME_ID_POS_VEL, data, DM_MOTOR_CMD_LEN);
}

static BusMotorStatus dm_motor_send_speed(uint16_t id, const DmMotorSlot* slot) {
    uint8_t data[4] = { 0 };

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    memcpy(&data[0], &slot->speed, sizeof(float));
    return dm_motor_send((uint32_t)id + DM_MOTOR_CTRL_FRAME_ID_SPEED, data, 4u);
}

static BusMotorStatus dm_motor_send_mit(uint16_t id, const DmMotorSlot* slot) {
    uint16_t pos_bits;
    uint16_t spd_bits;
    uint16_t kp_bits;
    uint16_t kd_bits;
    uint16_t tor_bits;
    uint8_t data[DM_MOTOR_CMD_LEN];

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    pos_bits = dm_motor_f32_to_u16(slot->position, -s_pos_limit, s_pos_limit, 16);
    spd_bits = dm_motor_f32_to_u16(slot->speed, -s_spd_limit, s_spd_limit, 12);
    kp_bits = dm_motor_f32_to_u16(slot->kp, 0.0f, DM_MOTOR_KP_LIMIT, 12);
    kd_bits = dm_motor_f32_to_u16(slot->kd, 0.0f, DM_MOTOR_KD_LIMIT, 12);
    tor_bits = dm_motor_f32_to_u16(slot->torque, -s_tor_limit, s_tor_limit, 12);

    data[0] = (uint8_t)(pos_bits >> 8);
    data[1] = (uint8_t)(pos_bits & 0xFFu);
    data[2] = (uint8_t)(spd_bits >> 4);
    data[3] = (uint8_t)(((spd_bits & 0x0Fu) << 4) | (kp_bits >> 8));
    data[4] = (uint8_t)(kp_bits & 0xFFu);
    data[5] = (uint8_t)(kd_bits >> 4);
    data[6] = (uint8_t)(((kd_bits & 0x0Fu) << 4) | (tor_bits >> 8));
    data[7] = (uint8_t)(tor_bits & 0xFFu);

    return dm_motor_send(id, data, DM_MOTOR_CMD_LEN);
}

static BusMotorStatus dm_motor_send_pos_vel_tor(uint16_t id, const DmMotorSlot* slot) {
    int16_t spd_scaled;
    int16_t tor_scaled;
    uint8_t data[DM_MOTOR_CMD_LEN];

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    spd_scaled = (int16_t)(slot->speed * 100.0f);
    tor_scaled = (int16_t)(slot->torque * 10000.0f);

    memcpy(&data[0], &slot->position, sizeof(float));
    data[4] = (uint8_t)(spd_scaled & 0xFFu);
    data[5] = (uint8_t)(((uint16_t)spd_scaled >> 8) & 0xFFu);
    data[6] = (uint8_t)(tor_scaled & 0xFFu);
    data[7] = (uint8_t)(((uint16_t)tor_scaled >> 8) & 0xFFu);

    return dm_motor_send((uint32_t)id + DM_MOTOR_CTRL_FRAME_ID_POS_VEL_TOR, data, DM_MOTOR_CMD_LEN);
}

static uint16_t dm_motor_f32_to_u16(float val, float min, float max, uint8_t bits) {
    float span;
    float normalized;
    uint32_t max_bits_val;

    if(bits == 0u || max <= min) {
        return 0u;
    }
    if(val < min) {
        val = min;
    }
    if(val > max) {
        val = max;
    }

    span = max - min;
    normalized = (val - min) / span;
    max_bits_val = (1UL << bits) - 1UL;
    return (uint16_t)(normalized * (float)max_bits_val);
}

static float dm_motor_u16_to_f32(uint16_t val, float min, float max, uint8_t bits) {
    float span;
    uint32_t max_bits_val;

    if(bits == 0u || max <= min) {
        return 0.0f;
    }

    span = max - min;
    max_bits_val = (1UL << bits) - 1UL;
    return ((float)val) * span / (float)max_bits_val + min;
}
