#ifndef _imu_h_
#define _imu_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 当前 IMU 实例的便捷访问宏
 *
 * 上层可使用 `imu.update()`、`imu.get_acc()` 等形式访问当前绑定实例；
 * 也可以直接调用 `imu_update()`、`imu_get_acc()` 等包装函数
 */
#define imu (*imu_instance)

/**
 * @brief IMU 通用状态码表
 */
#define IMU_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(INVALID_PARAM, 2) \
    X(NO_INSTANCE, 3) \
    X(NOT_INITIALIZE, 4) \
    X(NOT_READY, 5) \
    X(UNSUPPORTED, 6)

#define X(name, value) IMU_STATUS_##name = value,
/**
 * @brief IMU 通用状态码
 */
typedef enum {
    IMU_STATUS_TABLE
} ImuStatus;
#undef X

/**
 * @brief IMU 三轴加速度
 */
typedef struct {
    float x; /**< x 轴加速度，单位 m/s^2 */
    float y; /**< y 轴加速度，单位 m/s^2 */
    float z; /**< z 轴加速度，单位 m/s^2 */
} ImuAcc;

/**
 * @brief IMU 三轴角速度
 */
typedef struct {
    float x; /**< x 轴角速度，单位 rad/s */
    float y; /**< y 轴角速度，单位 rad/s */
    float z; /**< z 轴角速度，单位 rad/s */
} ImuGyro;

/**
 * @brief IMU 姿态角
 */
typedef struct {
    float roll;  /**< 横滚角，单位 rad */
    float pitch; /**< 俯仰角，单位 rad */
    float yaw;   /**< 偏航角，单位 rad */
} ImuAngle;

/**
 * @brief IMU 采样新数据标志
 */
typedef enum {
    IMU_SAMPLE_NONE = 0,             /**< 没有新数据 */
    IMU_SAMPLE_ACC_NEW = 1 << 0,     /**< 本次采样包含新的加速度 */
    IMU_SAMPLE_GYRO_NEW = 1 << 1,    /**< 本次采样包含新的角速度 */
    IMU_SAMPLE_TEMP_NEW = 1 << 2,    /**< 本次采样包含新的温度 */
    IMU_SAMPLE_ACC_VALID = 1 << 3,   /**< 当前缓存中存在可用于融合的加速度 */
    IMU_SAMPLE_GYRO_VALID = 1 << 4,  /**< 当前缓存中存在可用于积分的角速度 */
    IMU_SAMPLE_TEMP_VALID = 1 << 5,  /**< 当前缓存中存在可读取的温度 */
} ImuSampleFlags;

/**
 * @brief IMU 一帧采样数据
 */
typedef struct {
    ImuAcc acc;                /**< 最近一次加速度数据 */
    ImuGyro gyro;              /**< 最近一次角速度数据 */
    float temperature;         /**< 最近一次温度数据，单位由具体驱动定义 */
    uint32_t acc_timestamp_us; /**< 最近一次加速度时间戳，单位 us */
    uint32_t gyro_timestamp_us;/**< 最近一次角速度时间戳，单位 us */
    uint32_t temp_timestamp_us;/**< 最近一次温度时间戳，单位 us */
    uint8_t flags;             /**< 本次采样包含的新数据标志，见 @ref ImuSampleFlags */
} ImuSample;

/**
 * @brief 姿态融合算法模式
 */
typedef enum {
    IMU_ATTITUDE_NONE = 0,          /**< 不启用姿态融合 */
    IMU_ATTITUDE_COMPLEMENTARY,     /**< 互补滤波，roll/pitch 使用加速度修正 */
    IMU_ATTITUDE_MAHONY_6AXIS,      /**< Mahony 六轴融合，不依赖磁力计 */
} ImuAttitudeMode;

/**
 * @brief 姿态融合算法配置
 *
 * 这是纯算法配置，通常由具体 IMU 配置结构持有，例如 `Bmi088Config.attitude`
 */
typedef struct {
    ImuAttitudeMode mode;       /**< 融合模式，见 @ref ImuAttitudeMode */

    uint16_t gyro_calib_samples;/**< 启动时用于估计陀螺零偏的样本数；0 表示跳过校准 */
    float acc_norm;             /**< 静止重力加速度参考值，通常为 9.80665 */
    float acc_norm_tolerance;   /**< 加速度模长可信窗口，超过该误差时不使用加速度修正 */
    uint32_t max_acc_age_us;    /**< 加速度参与融合前允许的最大陈旧时间，单位 us */

    float gyro_calib_var_threshold; /**< 启动静止校准时允许的陀螺方差阈值 */
    float complementary_tau;    /**< 互补滤波时间常数，单位 s */

    float mahony_kp;            /**< Mahony 比例增益，主要影响 roll/pitch 收敛速度 */
    float mahony_ki;            /**< Mahony x/y 轴积分增益，用于慢速零偏修正 */
    float mahony_ki_z;          /**< Mahony z 轴积分增益；六轴无绝对 yaw 参考，通常设为 0 */

    float gyro_x_temp_coeff;    /**< 角速度 x 轴温度补偿系数，单位 rad/s/℃；0 表示不启用温度补偿 */
    float gyro_y_temp_coeff;    /**< 角速度 y 轴温度补偿系数，单位 rad/s/℃；0 表示不启用温度补偿 */
    float gyro_z_temp_coeff;    /**< 角速度 z 轴温度补偿系数，单位 rad/s/℃；与当前温度直接相乘 */
    float gyro_z_bias_offset;   /**< z 轴固定 bias 截距，单位 rad/s */
    float gyro_z_bias_temp_coeff; /**< 兼容保留字段；当前实现不再使用启动 temp_ref 参与 z 轴补偿 */

    float zru_gyro_threshold;   /**< 静止 ZRU 的三轴角速度阈值，单位 rad/s；<=0 表示关闭 ZRU */
    uint32_t zru_min_static_us; /**< 触发 ZRU 前要求连续静止的最短时间，单位 us */
    float zru_bias_gain;        /**< ZRU 对 gyro_bias.z 的在线修正增益，单位 1/s；0 表示不修正 */
} ImuAttitudeConfig;

/**
 * @brief IMU 统一接口表
 *
 * 具体 IMU 驱动通过该接口表暴露通用能力平台相关回调、硬件寄存器和
 * 可选算法组合应留在具体驱动内部，不放进统一门面
 */
typedef struct {
    ImuStatus(*init)(const void* config);      /**< 初始化具体 IMU 驱动 */
    ImuStatus(*update)(void);                  /**< 刷新具体 IMU 的最新缓存 */
    ImuAcc(*get_acc)(void);                    /**< 获取最近一次缓存的加速度 */
    ImuGyro(*get_gyro)(void);                  /**< 获取最近一次缓存的原始角速度 */
    ImuGyro(*get_gyro_bias)(void);             /**< 获取姿态融合估计的陀螺零偏；可选 */
    ImuGyro(*get_gyro_corrected)(void);        /**< 获取扣零偏和温漂补偿后的角速度；可选 */
    ImuAngle(*get_angle)(void);                /**< 获取最近一次缓存的姿态角 */
    ImuStatus(*get_sample)(ImuSample* sample); /**< 获取最近一帧采样；可清除具体驱动的新数据标志 */
    const char* (*status_str)(ImuStatus status); /**< 状态码转字符串 */
} ImuInterface;

/**
 * @brief 当前绑定的具体 IMU 实例
 */
extern const ImuInterface* imu_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 绑定具体 IMU 实例
 * @param instance 具体 IMU 接口表，例如 `&bmi088_instance`
 * @return IMU 状态码
 */
ImuStatus imu_set_instance(const ImuInterface* instance);

/**
 * @brief 初始化当前绑定的 IMU 实例
 * @param config 具体 IMU 的配置结构指针
 * @return IMU 状态码
 */
ImuStatus imu_init(const void* config);

/**
 * @brief 刷新当前绑定 IMU 的数据缓存
 * @return IMU 状态码
 */
ImuStatus imu_update(void);

/**
 * @brief 获取最近一次缓存的加速度
 * @return 三轴加速度
 */
ImuAcc imu_get_acc(void);

/**
 * @brief 获取最近一次缓存的原始角速度
 * @return 三轴角速度，单位 rad/s
 */
ImuGyro imu_get_gyro(void);

/**
 * @brief 获取姿态融合估计的陀螺零偏
 * @return 三轴陀螺零偏，单位 rad/s；实例不支持时返回 0
 */
ImuGyro imu_get_gyro_bias(void);

/**
 * @brief 获取扣除零偏和温漂补偿后的角速度
 * @return 三轴角速度，单位 rad/s；实例不支持时回退为原始角速度
 */
ImuGyro imu_get_gyro_corrected(void);

/**
 * @brief 获取最近一次缓存的姿态角
 * @return 姿态角
 */
ImuAngle imu_get_angle(void);

/**
 * @brief 获取最近一帧 IMU 采样
 * @param sample 输出采样数据
 * @return IMU 状态码
 */
ImuStatus imu_get_sample(ImuSample* sample);

/**
 * @brief 将 IMU 状态码转换为字符串
 * @param status IMU 状态码
 * @return 状态码名称字符串
 */
const char* imu_status_str(ImuStatus status);

#endif
