#ifndef _bus_motor_h_
#define _bus_motor_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file bus_motor.h
 * @brief 总线电机统一抽象接口
 *
 * 本模块只定义所有总线电机共同具备的能力，不同品牌电机的协议、
 * 模式枚举和参数含义由具体实例自行定义
 */

/**
 * @brief 当前电机实例的便捷访问宏
 */
#define bus_motor (*bus_motor_instance)

/**
 * @brief 电机通用状态码表
 */
#define MOTOR_STATUS_TABLE \
    X(OK, 0)               \
    X(ERROR, 1)            \
    X(INVALID_PARAM, 2)    \
    X(PORT_ERROR, 3)       \
    X(TIMEOUT, 4)          \
    X(ID_MISMATCH, 5)      \
    X(NO_INSTANCE, 6)      \
    X(NOT_INITIALIZE, 7)   \
    X(UNSUPPORTED, 8)      \
    X(NO_FEEDBACK, 9)

#define X(name, value) MOTOR_STATUS_##name = value,
/**
 * @brief 电机通用状态码
 */
typedef enum {
    MOTOR_STATUS_TABLE
} BusMotorStatus;
#undef X

/**
 * @brief 电机通用模式值
 *
 * 统一层只透传该值，不解释具体含义；具体模式枚举见 `DmMotorMode`、
 * `DjiMotorMode` 等驱动头文件
 */
typedef uint32_t BusMotorMode;

/**
 * @brief 最近一次解析出的通用电机反馈
 */
typedef struct {
    uint16_t id;        /**< 电机协议 ID */
    uint8_t error_code; /**< 最近一次反馈帧中的原始错误码 */
    float position;     /**< 当前位置，单位 rad */
    float speed;        /**< 当前速度，单位 rad/s */
    float torque;       /**< 当前扭矩或负载估计，单位 N*m */
} BusMotorFeedback;

/**
 * @brief 电机底层端口函数表
 */
typedef struct {
    bool (*send)(uint32_t id, const uint8_t* data, uint8_t len); /**< 发送一帧总线数据 */
    bool (*read)(uint32_t* id, uint8_t* data, uint8_t* len);     /**< 读取一帧总线数据 */
    uint32_t (*now_ms)(void);                                    /**< 获取当前单调时间，单位 ms */
    void (*delay_ms)(uint32_t ms);                               /**< 可选阻塞延时，单位 ms */
    void (*flush_rx)(void);                                      /**< 可选清空接收缓冲 */
} BusMotorPortOps;

/**
 * @brief 电机通用初始化配置
 */
typedef struct {
    const BusMotorPortOps* ops; /**< 底层端口函数表，不能为空 */
    uint32_t timeout_ms;        /**< 反馈等待超时，单位 ms，0 表示使用具体驱动默认值 */
    uint8_t retry_count;        /**< 预留重试次数，具体驱动按需使用 */

    /**
     * @brief 具体驱动的私有配置
     *
     * 通用 bus_motor 层不解释该指针，直接传递给具体驱动
     * 允许为 NULL，由具体驱动决定默认行为
     */
    const void* driver_config;
} BusMotorConfig;

/**
 * @brief 电机统一接口表
 */
typedef struct {
    BusMotorStatus (*init)(const BusMotorConfig* config);                       /**< 初始化具体电机驱动 */
    const char* (*status_str)(BusMotorStatus status);                           /**< 状态码转字符串 */
    const char* (*mode_str)(BusMotorMode mode);                                 /**< 模式值转字符串 */
    BusMotorStatus (*enable)(uint16_t id);                                      /**< 使能电机 */
    BusMotorStatus (*disable)(uint16_t id);                                     /**< 失能电机 */
    BusMotorStatus (*switch_mode)(uint16_t id, BusMotorMode mode);              /**< 切换电机模式 */
    BusMotorStatus (*set_pos)(uint16_t id, float position);                     /**< 设定目标位置，单位 rad */
    BusMotorStatus (*set_spd)(uint16_t id, float speed);                        /**< 设定目标速度，单位 rad/s */
    BusMotorStatus (*set_pos_vel)(uint16_t id, float position, float speed);    /**< 设定目标位置和速度 */
    BusMotorStatus (*set_tor)(uint16_t id, float torque);                       /**< 设定目标扭矩或前馈，单位 N*m */
    BusMotorStatus (*set_pd)(uint16_t id, float kp, float kd);                  /**< 设定位置环 PD 参数 */
    BusMotorStatus (*update_feedback)(uint16_t id, BusMotorFeedback* feedback); /**< 主动请求并更新反馈 */
    float (*get_pos)(uint16_t id);                                              /**< 从本地缓存获取最近位置，单位 rad */
    float (*get_spd)(uint16_t id);                                              /**< 从本地缓存获取最近速度，单位 rad/s */
    float (*get_tor)(uint16_t id);                                              /**< 从本地缓存获取最近扭矩，单位 N*m */
    BusMotorStatus (*stop)(uint16_t id);                                        /**< 停止电机 */
    BusMotorStatus (*brake)(uint16_t id);                                       /**< 制动电机 */
} BusMotorInterface;

/**
 * @brief 当前绑定的具体电机实例
 */
extern const BusMotorInterface* bus_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 绑定具体电机实例
 * @param instance 电机接口表
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_instance(const BusMotorInterface* instance);

/**
 * @brief 初始化当前绑定的电机实例
 * @param config 电机通用配置
 * @return 电机状态码
 */
BusMotorStatus bus_motor_init(const BusMotorConfig* config);

/**
 * @brief 状态码转字符串
 * @param status 电机状态码
 * @return 状态码名称字符串
 */
const char* bus_motor_status_str(BusMotorStatus status);

/**
 * @brief 模式值转字符串
 * @param mode 具体电机驱动定义的模式值
 * @return 模式名称字符串
 */
const char* bus_motor_mode_str(BusMotorMode mode);

/**
 * @brief 使能电机
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus bus_motor_enable(uint16_t id);

/**
 * @brief 失能电机
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus bus_motor_disable(uint16_t id);

/**
 * @brief 切换电机模式
 * @param id 电机协议 ID
 * @param mode 具体电机驱动定义的模式值
 * @return 电机状态码
 */
BusMotorStatus bus_motor_switch_mode(uint16_t id, BusMotorMode mode);

/**
 * @brief 设定目标位置
 * @param id 电机协议 ID
 * @param position 目标位置，单位 rad
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_pos(uint16_t id, float position);

/**
 * @brief 设定目标速度
 * @param id 电机协议 ID
 * @param speed 目标速度，单位 rad/s
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_spd(uint16_t id, float speed);

/**
 * @brief 设定目标位置和速度
 * @param id 电机协议 ID
 * @param position 目标位置，单位 rad
 * @param speed 目标速度，单位 rad/s
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_pos_vel(uint16_t id, float position, float speed);

/**
 * @brief 设定目标扭矩或前馈
 * @param id 电机协议 ID
 * @param torque 目标扭矩或前馈，单位 N*m
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_tor(uint16_t id, float torque);

/**
 * @brief 设定位置环 PD 参数
 * @param id 电机协议 ID
 * @param kp 比例系数
 * @param kd 微分系数
 * @return 电机状态码
 */
BusMotorStatus bus_motor_set_pd(uint16_t id, float kp, float kd);

/**
 * @brief 主动请求并更新反馈缓存
 * @param id 电机协议 ID
 * @param feedback 可选输出反馈，允许为 NULL
 * @return 电机状态码
 */
BusMotorStatus bus_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);

/**
 * @brief 获取最近位置
 * @param id 电机协议 ID
 * @return 最近位置，单位 rad
 */
float bus_motor_get_pos(uint16_t id);

/**
 * @brief 获取最近速度
 * @param id 电机协议 ID
 * @return 最近速度，单位 rad/s
 */
float bus_motor_get_spd(uint16_t id);

/**
 * @brief 获取最近扭矩或负载估计
 * @param id 电机协议 ID
 * @return 最近扭矩或负载估计，单位 N*m
 */
float bus_motor_get_tor(uint16_t id);

/**
 * @brief 停止电机
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus bus_motor_stop(uint16_t id);

/**
 * @brief 制动电机
 * @param id 电机协议 ID
 * @return 电机状态码
 */
BusMotorStatus bus_motor_brake(uint16_t id);

#endif
