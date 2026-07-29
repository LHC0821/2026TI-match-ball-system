#ifndef _bmi088_h_
#define _bmi088_h_

#include <stdbool.h>
#include <stdint.h>

#include "imu/imu.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief BMI088 初始化错误码
 */
typedef enum {
    BMI088_ERROR_NO_ERROR = 0x00,                 /**< 无错误 */
    BMI088_ERROR_ACC_PWR_CTRL = 0x01,             /**< 加速度计电源控制寄存器配置失败 */
    BMI088_ERROR_ACC_PWR_CONF = 0x02,             /**< 加速度计电源模式寄存器配置失败 */
    BMI088_ERROR_ACC_CONF = 0x03,                 /**< 加速度计采样配置失败 */
    BMI088_ERROR_ACC_SELF_TEST = 0x04,            /**< 加速度计自检失败 */
    BMI088_ERROR_ACC_RANGE = 0x05,                /**< 加速度计量程配置失败 */
    BMI088_ERROR_INT1_IO_CTRL = 0x06,             /**< 加速度计 INT1 IO 配置失败 */
    BMI088_ERROR_INT_MAP_DATA = 0x07,             /**< 加速度计数据就绪中断映射失败 */
    BMI088_ERROR_GYRO_RANGE = 0x08,               /**< 陀螺仪量程配置失败 */
    BMI088_ERROR_GYRO_BANDWIDTH = 0x09,           /**< 陀螺仪带宽配置失败 */
    BMI088_ERROR_GYRO_LPM1 = 0x0A,                /**< 陀螺仪电源模式配置失败 */
    BMI088_ERROR_GYRO_CTRL = 0x0B,                /**< 陀螺仪数据就绪开关配置失败 */
    BMI088_ERROR_GYRO_INT3_INT4_IO_CONF = 0x0C,   /**< 陀螺仪 INT3/INT4 IO 配置失败 */
    BMI088_ERROR_GYRO_INT3_INT4_IO_MAP = 0x0D,    /**< 陀螺仪数据就绪中断映射失败 */
    BMI088_ERROR_SELF_TEST_ACCEL = 0x80,          /**< 加速度计自检错误 */
    BMI088_ERROR_SELF_TEST_GYRO = 0x40,           /**< 陀螺仪自检错误 */
    BMI088_ERROR_NO_SENSOR = 0xFF,                /**< 未读到期望芯片 ID */
} Bmi088Error;

/**
 * @brief BMI088 平台端口函数表
 */
typedef struct {
    void (*accel_cs_low)(void);  /**< 拉低加速度计片选 */
    void (*accel_cs_high)(void); /**< 拉高加速度计片选 */
    void (*gyro_cs_low)(void);   /**< 拉低陀螺仪片选 */
    void (*gyro_cs_high)(void);  /**< 拉高陀螺仪片选 */
    uint8_t(*read_write_byte)(uint8_t tx_data); /**< SPI 单字节收发 */
    bool (*transmit_receive_dma)(uint8_t* tx_data, uint8_t* rx_data, uint16_t len); /**< 可选 SPI DMA 收发 */
    void* (*get_spi_handle)(void); /**< 可选 SPI 句柄，用于 DMA 完成回调归属判断 */
    uint32_t(*now_ms)(void);       /**< 获取当前单调时间，单位 ms */
    uint32_t(*now_us)(void);       /**< 可选高精度单调时间，单位 us；为空时回退到 `now_ms() * 1000` */
    void (*delay_ms)(uint32_t ms); /**< 可选阻塞延时，单位 ms */
    void (*delay_us)(uint16_t us); /**< 阻塞延时，单位 us */
    void (*cache_clean)(const void* addr, uint32_t len); /**< 可选 DMA 启动前 cache clean */
    void (*cache_invalidate)(const void* addr, uint32_t len); /**< 可选 DMA 结束后 cache invalidate */
} Bmi088PortOps;

/**
 * @brief BMI088 初始化配置
 */
typedef struct {
    const Bmi088PortOps* ops;     /**< 平台端口函数表，不能为空 */
    float accel_sen;              /**< 加速度原始值灵敏度系数，默认 3G 量程 */
    float gyro_sen;               /**< 陀螺仪原始值灵敏度系数，默认 2000 dps 量程 */
    uint16_t accel_int_pin;       /**< 加速度计数据就绪中断引脚；阻塞实例可填 0 */
    uint16_t gyro_int_pin;        /**< 陀螺仪数据就绪中断引脚；阻塞实例可填 0 */
    ImuAttitudeConfig attitude;   /**< 姿态融合配置；`mode` 为 `IMU_ATTITUDE_NONE` 时关闭融合 */
} Bmi088Config;

/**
 * @brief BMI088 姿态/温漂补偿调试信息
 */
typedef struct {
    float temperature;         /**< 当前缓存温度，单位 ℃ */
    float gyro_temp_ref;       /**< 启动标定时的参考温度，单位 ℃ */
    ImuGyro gyro_bias;         /**< 当前陀螺零偏估计，单位 rad/s */
    ImuGyro gyro_corrected;    /**< 当前零偏+温漂补偿后的角速度，单位 rad/s */
    ImuGyro gyro_temp_comp;    /**< 当前三轴温漂补偿量，单位 rad/s */
    float gyro_z_temp_intercept; /**< z 轴 bias 线性模型截距 */
    float gyro_z_bias_effective; /**< 当前实际用于扣除的 z 轴 bias */
    ImuGyro gyro_temp_coeff;   /**< 当前三轴温漂补偿系数，单位 rad/s/℃ */
    bool zru_enabled;          /**< true 表示当前允许执行静止 ZRU */
    bool zru_active;           /**< true 表示当前已进入静止 ZRU 修正 */
} Bmi088AttitudeDebug;

/**
 * @brief BMI088 中断 + SPI DMA 异步 IMU 实例
 */
extern const ImuInterface bmi088_instance;

/**
 * @brief BMI088 阻塞读取 IMU 实例
 */
extern const ImuInterface bmi088_blocking_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 生成 BMI088 默认配置
 * @param config 输出配置对象
 * @param ops 平台端口函数表
 * @param accel_int_pin 加速度计数据就绪中断引脚
 * @param gyro_int_pin 陀螺仪数据就绪中断引脚
 * @return IMU 状态码
 */
ImuStatus bmi088_make_config(Bmi088Config* config, const Bmi088PortOps* ops,
    const uint16_t accel_int_pin, const uint16_t gyro_int_pin);

/**
 * @brief 将 BMI088 初始化错误码转换为字符串
 * @param error BMI088 初始化错误码
 * @return 错误码名称字符串
 */
const char* bmi088_error_str(Bmi088Error error);

/**
 * @brief 获取最近一次 BMI088 初始化错误码
 * @return BMI088 初始化错误码
 */
Bmi088Error bmi088_get_init_error(void);

/**
 * @brief 获取缓存的 BMI088 温度
 * @return 温度，单位摄氏度；未初始化时返回 0
 *
 * 温度由 `update()` 内部周期性刷新，此接口不再触发阻塞读取
 */
float bmi088_get_temp(void);

/**
 * @brief 获取 BMI088 当前姿态/温漂补偿调试信息
 * @param debug 输出调试信息
 * @return IMU 状态码
 */
ImuStatus bmi088_get_attitude_debug(Bmi088AttitudeDebug* debug);

/**
 * @brief 设置 BMI088 姿态模块的静止 ZRU 开关
 * @param enabled true 允许执行静止 ZRU，false 禁止
 * @return IMU 状态码
 */
ImuStatus bmi088_set_zru_enabled(bool enabled);

/**
 * @brief 获取 BMI088 姿态模块当前是否允许执行静止 ZRU
 * @return true 允许执行，false 禁止或未启用姿态模块
 */
bool bmi088_is_zru_enabled(void);

/**
 * @brief BMI088 数据就绪 EXTI 回调转发入口
 * @param gpio_pin 触发中断的 GPIO 引脚
 */
void bmi088_exti_callback(uint16_t gpio_pin);

/**
 * @brief BMI088 SPI DMA 完成回调转发入口
 * @param spi_handle 平台 SPI 句柄
 */
void bmi088_spi_txrx_cplt_callback(void* spi_handle);

/**
 * @brief BMI088 SPI 错误回调转发入口
 * @param spi_handle 平台 SPI 句柄
 */
void bmi088_spi_error_callback(void* spi_handle);

#endif
