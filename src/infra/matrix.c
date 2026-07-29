#include "matrix.h"

#include <stddef.h>
#include <math.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 创建一个矩阵结构体
 * @param m 输出的矩阵结构体
 * @param row 矩阵的行数
 * @param col 矩阵的列数
 * @param data 矩阵数据的指针，必须至少包含 row * col 个 float 元素
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix(Matrix* const m, unsigned int row, unsigned int col, float* data) {
    if(m == NULL || data == NULL || row == 0 || col == 0) return MATRIX_CREATE_FAILED;

    m->pdata = data;
    m->row = row;
    m->col = col;

    return MATRIX_SUCCESS;
}

/**
 * @brief 创建一个单位矩阵
 * @param m 输出的矩阵结构体
 * @param size 矩阵的行列数，必须为正整数
 * @param data 矩阵数据的指针，必须至少包含 size * size 个 float 元素
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_identity(Matrix* const m, unsigned int size, float* data) {
    if(m == NULL || data == NULL || size == 0) return MATRIX_CREATE_FAILED;

    m->pdata = data;
    m->row = size;
    m->col = size;

    for(unsigned int i = 0; i < size; ++i) {
        for(unsigned int j = 0; j < size; ++j) {
            m->pdata[i * size + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 获取矩阵中指定位置的元素值
 * @param m 输入的矩阵结构体
 * @param r 行索引，范围 0 到 m->row - 1
 * @param c 列索引，范围 0 到 m->col - 1
 * @param value 输出的元素值指针
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_get(const Matrix* const m, unsigned int r, unsigned int c, float* value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;
    if(value == NULL) return MATRIX_ERROR;

    *value = m->pdata[r * m->col + c];

    return MATRIX_SUCCESS;
}

/**
 * @brief 设置矩阵中指定位置的元素值
 * @param m 输入的矩阵结构体
 * @param r 行索引，范围 0 到 m->row - 1
 * @param c 列索引，范围 0 到 m->col - 1
 * @param value 要设置的元素值
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_set(Matrix* const m, unsigned int r, unsigned int c, float value) {
    if(m == NULL || m->pdata == NULL || r >= m->row || c >= m->col) return MATRIX_INVALID;

    m->pdata[r * m->col + c] = value;

    return MATRIX_SUCCESS;
}

/**
 * @brief 复制一个矩阵
 * @param m 输入的矩阵结构体
 * @param out 输出的矩阵结构体，必须与输入矩阵具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_copy(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 矩阵加法
 * @param A 输入矩阵 A
 * @param B 输入矩阵 B，必须与 A 具有相同的行列数
 * @param out 输出矩阵，必须与 A 和 B 具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_add(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] + B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 矩阵减法
 * @param A 输入矩阵 A
 * @param B 输入矩阵 B，必须与 A 具有相同的行列数
 * @param out 输出矩阵，必须与 A 和 B 具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_sub(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->row != B->row || A->col != B->col || A->row != out->row || A->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < A->col; ++j) {
            out->pdata[i * A->col + j] = A->pdata[i * A->col + j] - B->pdata[i * B->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 矩阵数乘
 * @param m 输入矩阵
 * @param scalar 乘数
 * @param out 输出矩阵，必须与输入矩阵具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_scalar_mul(const Matrix* const m, float scalar, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[i * m->col + j] = m->pdata[i * m->col + j] * scalar;
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 矩阵乘法
 * @param A 输入矩阵 A，尺寸为 m x n
 * @param B 输入矩阵 B，尺寸为 n x p
 * @param out 输出矩阵，尺寸必须为 m x p
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_mul(const Matrix* const A, const Matrix* const B, Matrix* const out) {
    if(A == NULL || A->pdata == NULL || B == NULL || B->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(A->col != B->row || A->row != out->row || B->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(A == out || B == out) return MATRIX_ERROR;

    for(unsigned int i = 0; i < A->row; ++i) {
        for(unsigned int j = 0; j < B->col; ++j) {
            float sum = 0.0;
            for(unsigned int k = 0; k < A->col; ++k) {
                sum += A->pdata[i * A->col + k] * B->pdata[k * B->col + j];
            }
            out->pdata[i * out->col + j] = sum;
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 矩阵转置
 * @param m 输入矩阵
 * @param out 输出矩阵，行列数与输入矩阵相反
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_transpose(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != out->col || m->col != out->row) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    for(unsigned int i = 0; i < m->row; ++i) {
        for(unsigned int j = 0; j < m->col; ++j) {
            out->pdata[j * out->col + i] = m->pdata[i * m->col + j];
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 将矩阵转换为上三角矩阵
 * @param m 输入矩阵，必须为方阵
 * @param out 输出矩阵，必须与输入矩阵具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_to_upper_triangular(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    matrix_copy(m, out);

    for(unsigned int i = 0; i < n; ++i) {
        unsigned int max_row = i;
        float max_val = fabsf(out->pdata[i * n + i]);

        for(unsigned int k = i + 1; k < n; ++k) {
            float val = fabsf(out->pdata[k * n + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f) return MATRIX_PIVOT_IS_ZERO;
        if(max_row != i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = out->pdata[i * n + j];
                out->pdata[i * n + j] = out->pdata[max_row * n + j];
                out->pdata[max_row * n + j] = tmp;
            }
        }

        float pivot = out->pdata[i * n + i];
        for(unsigned int j = i + 1; j < n; ++j) {
            float factor = out->pdata[j * n + i] / pivot;

            for(unsigned int k = i; k < n; ++k) {
                out->pdata[j * n + k] -= factor * out->pdata[i * n + k];
            }

            out->pdata[j * n + i] = 0.0f;
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 将矩阵转换为下三角矩阵
 * @param m 输入矩阵，必须为方阵
 * @param out 输出矩阵，必须与输入矩阵具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_to_lower_triangular(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    matrix_copy(m, out);

    for(int i = (int)n - 1; i >= 0; --i) {
        unsigned int max_row = (unsigned int)i;
        float max_val = fabsf(out->pdata[(unsigned int)i * n + (unsigned int)i]);

        for(int k = i - 1; k >= 0; --k) {
            float val = fabsf(out->pdata[(unsigned int)k * n + (unsigned int)i]);
            if(val > max_val) {
                max_val = val;
                max_row = (unsigned int)k;
            }
        }

        if(max_val < 1e-8f) return MATRIX_PIVOT_IS_ZERO;
        if(max_row != (unsigned int)i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = out->pdata[(unsigned int)i * n + j];
                out->pdata[(unsigned int)i * n + j] = out->pdata[max_row * n + j];
                out->pdata[max_row * n + j] = tmp;
            }
        }

        float pivot = out->pdata[(unsigned int)i * n + (unsigned int)i];
        for(int j = i - 1; j >= 0; --j) {
            float factor = out->pdata[(unsigned int)j * n + (unsigned int)i] / pivot;

            for(int k = i; k >= 0; --k) {
                out->pdata[(unsigned int)j * n + (unsigned int)k] -= factor * out->pdata[(unsigned int)i * n + (unsigned int)k];
            }

            out->pdata[(unsigned int)j * n + (unsigned int)i] = 0.0f;
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 计算矩阵的行列式
 * @param m 输入矩阵，必须为方阵
 * @param out 输出的行列式值指针
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_determinant(const Matrix* const m, float* out) {
    if(m == NULL || m->pdata == NULL || out == NULL)
        return MATRIX_INVALID;

    if(m->row != m->col)
        return MATRIX_CANNOT_COMPUTE;

    unsigned int n = m->row;

    float temp_data[n * n];
    Matrix temp;
    matrix(&temp, n, n, temp_data);
    matrix_copy(m, &temp);
    int swap_count = 0;

    for(unsigned int i = 0; i < n; ++i) {
        unsigned int max_row = i;
        float max_val = fabsf(temp.pdata[i * n + i]);

        for(unsigned int k = i + 1; k < n; ++k) {
            float val = fabsf(temp.pdata[k * n + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f) {
            *out = 0.0f;
            return MATRIX_SUCCESS;
        }

        if(max_row != i) {
            for(unsigned int j = 0; j < n; ++j) {
                float tmp = temp.pdata[i * n + j];
                temp.pdata[i * n + j] = temp.pdata[max_row * n + j];
                temp.pdata[max_row * n + j] = tmp;
            }
            swap_count++;
        }

        float pivot = temp.pdata[i * n + i];
        for(unsigned int j = i + 1; j < n; ++j) {
            float factor = temp.pdata[j * n + i] / pivot;

            for(unsigned int k = i; k < n; ++k) {
                temp.pdata[j * n + k] -= factor * temp.pdata[i * n + k];
            }
        }
    }

    float det = 1.0f;
    for(unsigned int i = 0; i < n; ++i) {
        det *= temp.pdata[i * n + i];
    }

    if(swap_count % 2 != 0) {
        det = -det;
    }
    *out = det;

    return MATRIX_SUCCESS;
}

/**
 * @brief 计算矩阵的逆
 * @param m 输入矩阵，必须为方阵
 * @param out 输出矩阵，必须与输入矩阵具有相同的行列数
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_inverse(const Matrix* const m, Matrix* const out) {
    if(m == NULL || m->pdata == NULL || out == NULL || out->pdata == NULL) return MATRIX_INVALID;
    if(m->row != m->col || m->row != out->row || m->col != out->col) return MATRIX_CANNOT_COMPUTE;
    if(m == out) return MATRIX_INPLACE;

    unsigned int n = m->row;
    float aug_data[n * 2 * n]; Matrix aug = { 0 }; matrix(&aug, n, 2 * n, aug_data);

    for(unsigned int i = 0; i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            aug.pdata[i * aug.col + j] = m->pdata[i * m->col + j];
            aug.pdata[i * aug.col + (j + n)] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for(unsigned int i = 0; i < n; i++) {

        unsigned int max_row = i;
        float max_val = fabsf(aug.pdata[i * aug.col + i]);

        for(unsigned int k = i + 1; k < n; k++) {
            float val = fabsf(aug.pdata[k * aug.col + i]);
            if(val > max_val) {
                max_val = val;
                max_row = k;
            }
        }

        if(max_val < 1e-8f)
            return MATRIX_PIVOT_IS_ZERO;

        if(max_row != i) {
            for(unsigned int j = 0; j < 2 * n; j++) {
                float tmp = aug.pdata[i * aug.col + j];
                aug.pdata[i * aug.col + j] = aug.pdata[max_row * aug.col + j];
                aug.pdata[max_row * aug.col + j] = tmp;
            }
        }

        float pivot = aug.pdata[i * aug.col + i];
        for(unsigned int j = 0; j < 2 * n; j++) {
            aug.pdata[i * aug.col + j] /= pivot;
        }

        for(unsigned int k = 0; k < n; k++) {
            if(k == i) continue;

            float factor = aug.pdata[k * aug.col + i];

            for(unsigned int j = 0; j < 2 * n; j++) {
                aug.pdata[k * aug.col + j] -= factor * aug.pdata[i * aug.col + j];
            }
        }
    }

    for(unsigned int i = 0; i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            out->pdata[i * out->col + j] = aug.pdata[i * aug.col + (j + n)];
        }
    }

    return MATRIX_SUCCESS;
}

/**
 * @brief 归一化四元数
 * @param quat 输入的四元数，长度必须为 4
 * @param out 输出的归一化四元数，长度必须为 4
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode quat_normalize(const float quat[4], float out[4]) {
    if(quat == NULL || out == NULL) return MATRIX_INVALID;

    float norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if(norm < 1e-8f) return MATRIX_ERROR;

    out[0] = quat[0] / norm;
    out[1] = quat[1] / norm;
    out[2] = quat[2] / norm;
    out[3] = quat[3] / norm;

    return MATRIX_SUCCESS;
}

/**
 * @brief 将旋转矩阵转换为四元数
 * @param R 输入的旋转矩阵，必须为 3x3 矩阵
 * @param quat 输出的四元数，长度必须为 4，格式为 [w, x, y, z]
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode matrix_to_quat(const Matrix* const R, float quat[4]) {
    if(R == NULL || R->pdata == NULL || quat == NULL) return MATRIX_INVALID;
    if(R->row != 3 || R->col != 3) return MATRIX_CANNOT_COMPUTE;

    float trace = R->pdata[0 * 3 + 0] + R->pdata[1 * 3 + 1] + R->pdata[2 * 3 + 2];
    float S, qw, qx, qy, qz;

    if(trace > 0) {
        S = 2 * sqrtf(trace + 1.0f);
        qw = 0.25f * S;
        qx = (R->pdata[2 * 3 + 1] - R->pdata[1 * 3 + 2]) / S;
        qy = (R->pdata[0 * 3 + 2] - R->pdata[2 * 3 + 0]) / S;
        qz = (R->pdata[1 * 3 + 0] - R->pdata[0 * 3 + 1]) / S;
    }
    else {
        if(R->pdata[0 * 3 + 0] > R->pdata[1 * 3 + 1] && R->pdata[0 * 3 + 0] > R->pdata[2 * 3 + 2]) {
            S = 2 * sqrtf(1.0f + R->pdata[0 * 3 + 0] - R->pdata[1 * 3 + 1] - R->pdata[2 * 3 + 2]);
            qw = (R->pdata[2 * 3 + 1] - R->pdata[1 * 3 + 2]) / S;
            qx = 0.25f * S;
            qy = (R->pdata[0 * 3 + 1] + R->pdata[1 * 3 + 0]) / S;
            qz = (R->pdata[0 * 3 + 2] + R->pdata[2 * 3 + 0]) / S;
        }
        else if(R->pdata[1 * 3 + 1] > R->pdata[2 * 3 + 2]) {
            S = 2 * sqrtf(1.0f + R->pdata[1 * 3 + 1] - R->pdata[0 * 3 + 0] - R->pdata[2 * 3 + 2]);
            qw = (R->pdata[0 * 3 + 2] - R->pdata[2 * 3 + 0]) / S;
            qx = (R->pdata[0 * 3 + 1] + R->pdata[1 * 3 + 0]) / S;
            qy = 0.25f * S;
            qz = (R->pdata[1 * 3 + 2] + R->pdata[2 * 3 + 1]) / S;
        }
        else {
            S = 2 * sqrtf(1.0f + R->pdata[2 * 3 + 2] - R->pdata[0 * 3 + 0] - R->pdata[1 * 3 + 1]);
            qw = (R->pdata[1 * 3 + 0] - R->pdata[0 * 3 + 1]) / S;
            qx = (R->pdata[0 * 3 + 2] + R->pdata[2 * 3 + 0]) / S;
            qy = (R->pdata[1 * 3 + 2] + R->pdata[2 * 3 + 1]) / S;
            qz = 0.25f * S;
        }
    }

    quat[0] = qw;
    quat[1] = qx;
    quat[2] = qy;
    quat[3] = qz;

    return MATRIX_SUCCESS;
}

/**
 * @brief 将四元数转换为旋转矩阵
 * @param quat 输入的四元数，长度必须为 4，格式为 [w, x, y, z]
 * @param R 输出的旋转矩阵，必须为 3x3 矩阵
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode quat_to_matrix(const float quat[4], Matrix* const R) {
    if(quat == NULL || R == NULL || R->pdata == NULL) return MATRIX_INVALID;
    if(R->row != 3 || R->col != 3) return MATRIX_CANNOT_COMPUTE;
    quat_normalize(quat, (float*)quat);

    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    float* r = R->pdata;
    r[0 * 3 + 0] = 1.0f - 2.0f * (yy + zz);
    r[0 * 3 + 1] = 2.0f * (xy - wz);
    r[0 * 3 + 2] = 2.0f * (xz + wy);
    r[1 * 3 + 0] = 2.0f * (xy + wz);
    r[1 * 3 + 1] = 1.0f - 2.0f * (xx + zz);
    r[1 * 3 + 2] = 2.0f * (yz - wx);
    r[2 * 3 + 0] = 2.0f * (xz - wy);
    r[2 * 3 + 1] = 2.0f * (yz + wx);
    r[2 * 3 + 2] = 1.0f - 2.0f * (xx + yy);

    return MATRIX_SUCCESS;
}

/**
 * @brief 3D 向量加法
 * @param a 输入向量 a
 * @param b 输入向量 b
 * @param out 输出向量，必须与输入向量具有相同的维度
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_add(const Vector3* const a, const Vector3* const b, Vector3* out) {
    if(a == NULL || b == NULL || out == NULL) return MATRIX_INVALID;

    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;

    return MATRIX_SUCCESS;
}

/**
 * @brief 3D 向量减法
 * @param a 输入向量 a
 * @param b 输入向量 b
 * @param out 输出向量，必须与输入向量具有相同的维度
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_sub(const Vector3* const a, const Vector3* const b, Vector3* out) {
    if(a == NULL || b == NULL || out == NULL) return MATRIX_INVALID;

    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;

    return MATRIX_SUCCESS;
}

/**
 * @brief 3D 向量数乘
 * @param v 输入向量
 * @param scalar 乘数
 * @param out 输出向量，必须与输入向量具有相同的维度
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_scalar_mul(const Vector3* const v, float scalar, Vector3* out) {
    if(v == NULL || out == NULL) return MATRIX_INVALID;

    out->x = v->x * scalar;
    out->y = v->y * scalar;
    out->z = v->z * scalar;

    return MATRIX_SUCCESS;
}

/**
 * @brief 计算 3D 向量的范数
 * @param v 输入向量
 * @param out 输出的范数值指针
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_norm(const Vector3* const v, float* out) {
    if(v == NULL || out == NULL) return MATRIX_INVALID;

    *out = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);

    return MATRIX_SUCCESS;
}

/**
 * @brief 归一化 3D 向量
 * @param v 输入向量
 * @param out 输出的归一化向量，必须与输入向量具有相同的维度
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_normalize(const Vector3* const v, Vector3* out) {
    if(v == NULL || out == NULL) return MATRIX_INVALID;

    float norm;
    if(vec3_norm(v, &norm) != MATRIX_SUCCESS) return MATRIX_ERROR;

    if(norm < 1e-8f) return MATRIX_ERROR;

    out->x = v->x / norm;
    out->y = v->y / norm;
    out->z = v->z / norm;

    return MATRIX_SUCCESS;
}

/**
 * @brief 计算 3D 向量的点积
 * @param a 输入向量 a
 * @param b 输入向量 b
 * @param out 输出的点积值指针
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_dot(const Vector3* const a, const Vector3* const b, float* out) {
    if(a == NULL || b == NULL || out == NULL) return MATRIX_INVALID;

    *out = a->x * b->x + a->y * b->y + a->z * b->z;

    return MATRIX_SUCCESS;
}

/**
 * @brief 计算 3D 向量的叉积
 * @param a 输入向量 a
 * @param b 输入向量 b
 * @param out 输出的叉积向量，必须与输入向量具有相同的维度
 * @return MatrixErrorCode 错误码
 */
MatrixErrorCode vec3_cross(const Vector3* const a, const Vector3* const b, Vector3* out) {
    if(a == NULL || b == NULL || out == NULL) return MATRIX_INVALID;

    out->x = a->y * b->z - b->y * a->z;
    out->y = a->z * b->x - b->z * a->x;
    out->z = a->x * b->y - b->x * a->y;

    return MATRIX_SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


