#include "kalman.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 通用卡尔曼接口单例的文件内短别名
 */
#define kf kalman_interface

/**
 * @brief 通用卡尔曼滤波接口单例
 */
#define X(name, str) .name = KALMAN_##name,
const struct KalmanInterface kalman_interface = {
    { KALMAN_STATUS_TABLE },
    .filter_init = kalman_filter_init,
    .filter_predict = kalman_filter_predict,
    .filter_update = kalman_filter_update,
    .error_code_to_str = kalman_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 检查滤波维度是否在固定缓存范围内
 */
static bool kalman_dim_valid(uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim);
/**
 * @brief 将矩阵清零
 */
static void kalman_matrix_zero(Matrix* matrix);
/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void kalman_matrix_identity(Matrix* matrix);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化通用线性卡尔曼滤波器
 * @param filter 滤波器实例
 * @param state_dim 状态维度
 * @param meas_dim 观测维度
 * @param ctrl_dim 控制输入维度；0 表示不使用控制输入
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_init(KalmanFilter* filter, uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim) {
    if(filter == NULL) {
        return kf.INVALID_PARAM;
    }
    if(!kalman_dim_valid(state_dim, meas_dim, ctrl_dim)) {
        return kf.INVALID_DIM;
    }

    memset(filter, 0, sizeof(KalmanFilter));
    filter->state_dim = state_dim;
    filter->meas_dim = meas_dim;
    filter->ctrl_dim = ctrl_dim;

    if(matrix(&filter->x, state_dim, 1u, filter->x_data) != MATRIX_SUCCESS || matrix(&filter->P, state_dim, state_dim, filter->P_data) != MATRIX_SUCCESS || matrix(&filter->F, state_dim, state_dim, filter->F_data) != MATRIX_SUCCESS || matrix(&filter->B, state_dim, ctrl_dim == 0u ? 1u : ctrl_dim, filter->B_data) != MATRIX_SUCCESS || matrix(&filter->u, ctrl_dim == 0u ? 1u : ctrl_dim, 1u, filter->u_data) != MATRIX_SUCCESS || matrix(&filter->Q, state_dim, state_dim, filter->Q_data) != MATRIX_SUCCESS || matrix(&filter->z, meas_dim, 1u, filter->z_data) != MATRIX_SUCCESS || matrix(&filter->H, meas_dim, state_dim, filter->H_data) != MATRIX_SUCCESS || matrix(&filter->R, meas_dim, meas_dim, filter->R_data) != MATRIX_SUCCESS) {
        return kf.MATRIX_FAILED;
    }

    kalman_matrix_identity(&filter->P);
    kalman_matrix_identity(&filter->F);
    kalman_matrix_zero(&filter->B);
    kalman_matrix_zero(&filter->Q);
    kalman_matrix_zero(&filter->H);
    kalman_matrix_identity(&filter->R);

    filter->initialized = true;
    return kf.OK;
}

/**
 * @brief 执行一次预测步骤
 * @param filter 滤波器实例；调用前需由上层填好 F、Q，以及可选的 B、u
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_predict(KalmanFilter* filter) {
    float x_pred_data[KALMAN_MAX_STATE_DIM];
    float Bu_data[KALMAN_MAX_STATE_DIM];
    float FP_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float Ft_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float P_pred_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    Matrix x_pred;
    Matrix Bu;
    Matrix FP;
    Matrix Ft;
    Matrix P_pred;

    if(filter == NULL) {
        return kf.INVALID_PARAM;
    }
    if(!filter->initialized) {
        return kf.NOT_INITIALIZE;
    }

    (void)matrix(&x_pred, filter->state_dim, 1u, x_pred_data);
    (void)matrix(&Bu, filter->state_dim, 1u, Bu_data);
    (void)matrix(&FP, filter->state_dim, filter->state_dim, FP_data);
    (void)matrix(&Ft, filter->state_dim, filter->state_dim, Ft_data);
    (void)matrix(&P_pred, filter->state_dim, filter->state_dim, P_pred_data);

    if(matrix_mul(&filter->F, &filter->x, &x_pred) != MATRIX_SUCCESS) {
        return kf.MATRIX_FAILED;
    }

    if(filter->ctrl_dim > 0u) {
        if(matrix_mul(&filter->B, &filter->u, &Bu) != MATRIX_SUCCESS || matrix_add(&x_pred, &Bu, &filter->x) != MATRIX_SUCCESS) {
            return kf.MATRIX_FAILED;
        }
    }
    else if(matrix_copy(&x_pred, &filter->x) != MATRIX_SUCCESS) {
        return kf.MATRIX_FAILED;
    }

    if(matrix_mul(&filter->F, &filter->P, &FP) != MATRIX_SUCCESS || matrix_transpose(&filter->F, &Ft) != MATRIX_SUCCESS || matrix_mul(&FP, &Ft, &P_pred) != MATRIX_SUCCESS || matrix_add(&P_pred, &filter->Q, &filter->P) != MATRIX_SUCCESS) {
        return kf.MATRIX_FAILED;
    }

    return kf.OK;
}

/**
 * @brief 执行一次观测更新步骤
 * @param filter 滤波器实例；调用前需由上层填好 z、H、R
 * @return KalmanErrorCode 状态码
 */
KalmanErrorCode kalman_filter_update(KalmanFilter* filter) {
    float Hx_data[KALMAN_MAX_MEAS_DIM];
    float y_data[KALMAN_MAX_MEAS_DIM];
    float Ht_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_MEAS_DIM];
    float PHt_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_MEAS_DIM];
    float HP_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_STATE_DIM];
    float S_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_MEAS_DIM];
    float S_inv_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_MEAS_DIM];
    float K_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_MEAS_DIM];
    float Ky_data[KALMAN_MAX_STATE_DIM];
    float KH_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float I_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float I_KH_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float P_new_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    Matrix Hx;
    Matrix y;
    Matrix Ht;
    Matrix PHt;
    Matrix HP;
    Matrix S;
    Matrix S_inv;
    Matrix K;
    Matrix Ky;
    Matrix KH;
    Matrix I;
    Matrix I_KH;
    Matrix P_new;

    if(filter == NULL) {
        return kf.INVALID_PARAM;
    }
    if(!filter->initialized) {
        return kf.NOT_INITIALIZE;
    }

    (void)matrix(&Hx, filter->meas_dim, 1u, Hx_data);
    (void)matrix(&y, filter->meas_dim, 1u, y_data);
    (void)matrix(&Ht, filter->state_dim, filter->meas_dim, Ht_data);
    (void)matrix(&PHt, filter->state_dim, filter->meas_dim, PHt_data);
    (void)matrix(&HP, filter->meas_dim, filter->state_dim, HP_data);
    (void)matrix(&S, filter->meas_dim, filter->meas_dim, S_data);
    (void)matrix(&S_inv, filter->meas_dim, filter->meas_dim, S_inv_data);
    (void)matrix(&K, filter->state_dim, filter->meas_dim, K_data);
    (void)matrix(&Ky, filter->state_dim, 1u, Ky_data);
    (void)matrix(&KH, filter->state_dim, filter->state_dim, KH_data);
    (void)matrix(&I, filter->state_dim, filter->state_dim, I_data);
    (void)matrix(&I_KH, filter->state_dim, filter->state_dim, I_KH_data);
    (void)matrix(&P_new, filter->state_dim, filter->state_dim, P_new_data);

    kalman_matrix_identity(&I);

    if(matrix_mul(&filter->H, &filter->x, &Hx) != MATRIX_SUCCESS || matrix_sub(&filter->z, &Hx, &y) != MATRIX_SUCCESS || matrix_transpose(&filter->H, &Ht) != MATRIX_SUCCESS || matrix_mul(&filter->P, &Ht, &PHt) != MATRIX_SUCCESS || matrix_mul(&filter->H, &filter->P, &HP) != MATRIX_SUCCESS || matrix_mul(&HP, &Ht, &S) != MATRIX_SUCCESS || matrix_add(&S, &filter->R, &S) != MATRIX_SUCCESS || matrix_inverse(&S, &S_inv) != MATRIX_SUCCESS || matrix_mul(&PHt, &S_inv, &K) != MATRIX_SUCCESS || matrix_mul(&K, &y, &Ky) != MATRIX_SUCCESS || matrix_add(&filter->x, &Ky, &filter->x) != MATRIX_SUCCESS || matrix_mul(&K, &filter->H, &KH) != MATRIX_SUCCESS || matrix_sub(&I, &KH, &I_KH) != MATRIX_SUCCESS || matrix_mul(&I_KH, &filter->P, &P_new) != MATRIX_SUCCESS || matrix_copy(&P_new, &filter->P) != MATRIX_SUCCESS) {
        return kf.MATRIX_FAILED;
    }

    return kf.OK;
}

/**
 * @brief 将卡尔曼状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
#define X(name, str)    \
    case KALMAN_##name: \
        return str;
const char* kalman_error_code_to_str(KalmanErrorCode status) {
    switch(status) {
        KALMAN_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 检查滤波维度是否在固定缓存范围内
 */
static bool kalman_dim_valid(uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim) {
    return state_dim > 0u && state_dim <= KALMAN_MAX_STATE_DIM && meas_dim > 0u && meas_dim <= KALMAN_MAX_MEAS_DIM && ctrl_dim <= KALMAN_MAX_CTRL_DIM;
}

/**
 * @brief 将矩阵清零
 */
static void kalman_matrix_zero(Matrix* matrix) {
    unsigned int i;

    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row * matrix->col; ++i) {
        matrix->pdata[i] = 0.0f;
    }
}

/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void kalman_matrix_identity(Matrix* matrix) {
    unsigned int i;
    unsigned int j;

    kalman_matrix_zero(matrix);
    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row; ++i) {
        for(j = 0u; j < matrix->col; ++j) {
            matrix->pdata[i * matrix->col + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}
