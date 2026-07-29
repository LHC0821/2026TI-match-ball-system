#ifndef _kalman_h_
#define _kalman_h_

#include <stdbool.h>
#include <stdint.h>

#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 通用卡尔曼滤波算法接口单例别名
 */
#define kalman kalman_interface

/**
 * @brief 通用滤波器支持的最大状态维度
 */
#define KALMAN_MAX_STATE_DIM 9u
/**
 * @brief 通用滤波器支持的最大观测维度
 */
#define KALMAN_MAX_MEAS_DIM 9u
/**
 * @brief 通用滤波器支持的最大控制输入维度
 */
#define KALMAN_MAX_CTRL_DIM 6u

/**
 * @brief 通用卡尔曼滤波状态码表
 *
 * @param OK 操作成功
 * @param INVALID_PARAM 输入指针或参数无效
 * @param INVALID_DIM 状态、观测或控制维度超出固定缓存范围
 * @param MATRIX_FAILED 底层矩阵运算失败
 * @param NOT_INITIALIZE 滤波器尚未初始化
 */
#define KALMAN_STATUS_TABLE                   \
    X(OK, "OK")                               \
    X(INVALID_PARAM, "Invalid Parameter")     \
    X(INVALID_DIM, "Invalid Dimension")       \
    X(MATRIX_FAILED, "Matrix Compute Failed") \
    X(NOT_INITIALIZE, "Not Initialize")

#define X(name, str) KALMAN_##name,
/**
 * @brief 通用卡尔曼滤波错误码
 */
typedef enum {
    KALMAN_STATUS_TABLE
} KalmanErrorCode;
#undef X

/**
 * @brief 通用线性卡尔曼滤波器实例
 *
 * 状态方程：
 * x = F * x + B * u
 * P = F * P * F^T + Q
 *
 * 观测方程：
 * z = H * x + v
 *
 * 所有矩阵均使用结构体内固定数组承载，避免在控制循环中动态分配内存
 */
typedef struct {
    uint8_t state_dim; /**< 状态向量维度 */
    uint8_t meas_dim;  /**< 观测向量维度 */
    uint8_t ctrl_dim;  /**< 控制输入维度；0 表示不使用控制输入 */

    Matrix x; /**< 状态向量 */
    Matrix P; /**< 状态协方差矩阵 */
    Matrix F; /**< 状态转移矩阵 */
    Matrix B; /**< 控制输入矩阵 */
    Matrix u; /**< 控制输入向量 */
    Matrix Q; /**< 过程噪声协方差矩阵 */
    Matrix z; /**< 观测向量 */
    Matrix H; /**< 观测矩阵 */
    Matrix R; /**< 观测噪声协方差矩阵 */

    float x_data[KALMAN_MAX_STATE_DIM];                        /**< x 的固定存储 */
    float P_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM]; /**< P 的固定存储 */
    float F_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM]; /**< F 的固定存储 */
    float B_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_CTRL_DIM];  /**< B 的固定存储 */
    float u_data[KALMAN_MAX_CTRL_DIM];                         /**< u 的固定存储 */
    float Q_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM]; /**< Q 的固定存储 */
    float z_data[KALMAN_MAX_MEAS_DIM];                         /**< z 的固定存储 */
    float H_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_STATE_DIM];  /**< H 的固定存储 */
    float R_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_MEAS_DIM];   /**< R 的固定存储 */

    bool initialized; /**< true 表示滤波器已初始化 */
} KalmanFilter;

/**
 * @brief 通用卡尔曼滤波接口表
 */
#define X(name, str) KalmanErrorCode name;
extern const struct KalmanInterface {
    struct {
        KALMAN_STATUS_TABLE
    };
    /**
     * @brief 初始化通用线性卡尔曼滤波器
     * @param filter 滤波器实例
     * @param state_dim 状态维度
     * @param meas_dim 观测维度
     * @param ctrl_dim 控制输入维度；0 表示不使用控制输入
     * @return KalmanErrorCode 状态码
     */
    KalmanErrorCode (*filter_init)(KalmanFilter* filter, uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim);
    /**
     * @brief 执行一次预测步骤
     * @param filter 滤波器实例
     * @return KalmanErrorCode 状态码
     */
    KalmanErrorCode (*filter_predict)(KalmanFilter* filter);
    /**
     * @brief 执行一次观测更新步骤
     * @param filter 滤波器实例；调用前需由上层填好 z、H、R
     * @return KalmanErrorCode 状态码
     */
    KalmanErrorCode (*filter_update)(KalmanFilter* filter);
    /**
     * @brief 将卡尔曼状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 状态码说明
     */
    const char* (*error_code_to_str)(KalmanErrorCode status);
} kalman_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化通用线性卡尔曼滤波器
 * @param filter 滤波器实例
 * @param state_dim 状态维度
 * @param meas_dim 观测维度
 * @param ctrl_dim 控制输入维度；0 表示不使用控制输入
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_init(KalmanFilter* filter, uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim);
/**
 * @brief 执行一次预测步骤
 * @param filter 滤波器实例
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_predict(KalmanFilter* filter);
/**
 * @brief 执行一次观测更新步骤
 * @param filter 滤波器实例；调用前需由上层填好 z、H、R
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_update(KalmanFilter* filter);
/**
 * @brief 将卡尔曼状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
const char* kalman_error_code_to_str(KalmanErrorCode status);

#endif
