#include "bus_motor.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 当前绑定的具体电机实例
 */
const BusMotorInterface* bus_motor_instance = 0;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 绑定具体电机实例
 */
BusMotorStatus bus_motor_set_instance(const BusMotorInterface* instance) {
    if(instance == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    bus_motor_instance = instance;
    return MOTOR_STATUS_OK;
}

/**
 * @brief 初始化当前绑定的电机实例
 */
BusMotorStatus bus_motor_init(const BusMotorConfig* config) {
    if(bus_motor_instance == 0 || bus_motor_instance->init == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->init(config);
}

/**
 * @brief 将状态码转换为常量字符串
 */
const char* bus_motor_status_str(BusMotorStatus status) {
    if(bus_motor_instance != 0 && bus_motor_instance->status_str != 0) {
        return bus_motor_instance->status_str(status);
    }

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
const char* bus_motor_mode_str(BusMotorMode mode) {
    if(bus_motor_instance != 0 && bus_motor_instance->mode_str != 0) {
        return bus_motor_instance->mode_str(mode);
    }

    (void)mode;
    return "UNKNOWN";
}

/**
 * @brief 使能电机
 */
BusMotorStatus bus_motor_enable(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->enable == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->enable(id);
}

/**
 * @brief 失能电机
 */
BusMotorStatus bus_motor_disable(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->disable == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->disable(id);
}

/**
 * @brief 切换电机模式
 */
BusMotorStatus bus_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    if(bus_motor_instance == 0 || bus_motor_instance->switch_mode == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->switch_mode(id, mode);
}

/**
 * @brief 设定目标位置
 */
BusMotorStatus bus_motor_set_pos(uint16_t id, float position) {
    if(bus_motor_instance == 0 || bus_motor_instance->set_pos == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->set_pos(id, position);
}

/**
 * @brief 设定目标速度
 */
BusMotorStatus bus_motor_set_spd(uint16_t id, float speed) {
    if(bus_motor_instance == 0 || bus_motor_instance->set_spd == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->set_spd(id, speed);
}

/**
 * @brief 设定目标位置和速度
 */
BusMotorStatus bus_motor_set_pos_vel(uint16_t id, float position, float speed) {
    if(bus_motor_instance == 0 || bus_motor_instance->set_pos_vel == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->set_pos_vel(id, position, speed);
}

/**
 * @brief 设定目标扭矩或等效前馈
 */
BusMotorStatus bus_motor_set_tor(uint16_t id, float torque) {
    if(bus_motor_instance == 0 || bus_motor_instance->set_tor == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->set_tor(id, torque);
}

/**
 * @brief 设定位置环 PD 参数
 */
BusMotorStatus bus_motor_set_pd(uint16_t id, float kp, float kd) {
    if(bus_motor_instance == 0 || bus_motor_instance->set_pd == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->set_pd(id, kp, kd);
}

/**
 * @brief 主动请求并更新反馈缓存
 */
BusMotorStatus bus_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    if(bus_motor_instance == 0 || bus_motor_instance->update_feedback == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->update_feedback(id, feedback);
}

/**
 * @brief 获取最近位置反馈
 */
float bus_motor_get_pos(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->get_pos == 0) {
        return 0.0f;
    }

    return bus_motor_instance->get_pos(id);
}

/**
 * @brief 获取最近速度反馈
 */
float bus_motor_get_spd(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->get_spd == 0) {
        return 0.0f;
    }

    return bus_motor_instance->get_spd(id);
}

/**
 * @brief 获取最近扭矩反馈
 */
float bus_motor_get_tor(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->get_tor == 0) {
        return 0.0f;
    }

    return bus_motor_instance->get_tor(id);
}

/**
 * @brief 停止电机
 */
BusMotorStatus bus_motor_stop(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->stop == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->stop(id);
}

/**
 * @brief 制动电机
 */
BusMotorStatus bus_motor_brake(uint16_t id) {
    if(bus_motor_instance == 0 || bus_motor_instance->brake == 0) {
        return MOTOR_STATUS_NO_INSTANCE;
    }

    return bus_motor_instance->brake(id);
}
