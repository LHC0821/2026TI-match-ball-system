#ifndef _matrix_h_
#define _matrix_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 矩阵错误码
 * @param MATRIX_SUCCESS 成功
 * @param MATRIX_ERROR 一般错误
 * @param MATRIX_CREATE_FAILED 矩阵创建失败，可能是内存不足
 * @param MATRIX_INVALID 无效的矩阵输入，例如空指针或维度不匹配
 * @param MATRIX_CANNOT_COMPUTE 无法计算，例如矩阵乘法维度
 * @param MATRIX_PIVOT_IS_ZERO 在求逆过程中遇到零主元，矩阵可能是奇异的
 * @param MATRIX_INPLACE 输入输出矩阵指向同一内存地址，可能导致未定义行为
 */
typedef enum {
    MATRIX_SUCCESS = 0,
    MATRIX_ERROR,
    MATRIX_CREATE_FAILED,
    MATRIX_INVALID,
    MATRIX_CANNOT_COMPUTE,
    MATRIX_PIVOT_IS_ZERO,
    MATRIX_INPLACE,
} MatrixErrorCode;

/**
 * @brief 矩阵结构体，包含数据指针和维度信息
 * @param pdata 指向矩阵数据的指针，按行优先存储
 * @param row 矩阵的行数
 * @param col 矩阵的列数
 */
typedef struct {
    float* pdata;
    unsigned int row;
    unsigned int col;
} Matrix;

/**
 * @brief 3D 向量结构体，包含 x、y、z 三个分量
 * @param x 向量的 x 分量
 * @param y 向量的 y 分量
 * @param z 向量的 z 分量
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 创建一个矩阵，原地创建一个静态 float 数组作为数据存储，name 可以直接使用
 * @param name 矩阵名称
 * @param row 行数
 * @param col 列数
 * @note 该宏会定义一个名为 name##_data 的 float 数组作为矩阵数据存储，并创建一个 Matrix 结构体指向该数据
 * @note 创建后 name 就是可用的矩阵变量
 */
#define matrix_create(name, row, col) \
    static float name##_data[row * col]; \
    Matrix name; \
    matrix(&name, row, col, name##_data)

/**
 * @brief 创建一个单位矩阵，原地创建一个静态 float 数组作为数据存储，name 可以直接使用
 * @param name 矩阵名称
 * @param size 矩阵的行列数，单位矩阵必须是方阵
 * @note 该宏会定义一个名为 name##_data 的 float 数组作为矩阵数据存储，并创建一个 Matrix 结构体指向该数据，然后将其初始化为单位矩阵
 * @note 创建后 name 就是可用的单位矩阵变量
 */
#define matrix_identity_create(name, size) \
    static float name##_data[size * size]; \
    Matrix name; \
    matrix_identity(&name, size, name##_data)

MatrixErrorCode matrix(Matrix* const m, unsigned int row, unsigned int col, float* data);
MatrixErrorCode matrix_identity(Matrix* const m, unsigned int size, float* data);
MatrixErrorCode matrix_get(const Matrix* const m, unsigned int r, unsigned int c, float* value);
MatrixErrorCode matrix_set(Matrix* const m, unsigned int r, unsigned int c, float value);
MatrixErrorCode matrix_copy(const Matrix* const m, Matrix* const out);

MatrixErrorCode matrix_add(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_sub(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_scalar_mul(const Matrix* const m, float scalar, Matrix* const out);
MatrixErrorCode matrix_mul(const Matrix* const A, const Matrix* const B, Matrix* const out);
MatrixErrorCode matrix_transpose(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_to_upper_triangular(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_to_lower_triangular(const Matrix* const m, Matrix* const out);
MatrixErrorCode matrix_determinant(const Matrix* const m, float* out);
MatrixErrorCode matrix_inverse(const Matrix* const m, Matrix* const out);

MatrixErrorCode quat_normalize(const float quat[4], float out[4]);
MatrixErrorCode matrix_to_quat(const Matrix* const R, float quat[4]);
MatrixErrorCode quat_to_matrix(const float quat[4], Matrix* const R);

MatrixErrorCode vec3_add(const Vector3* const a, const Vector3* const b, Vector3* out);
MatrixErrorCode vec3_sub(const Vector3* const a, const Vector3* const b, Vector3* out);
MatrixErrorCode vec3_scalar_mul(const Vector3* const v, float scalar, Vector3* out);
MatrixErrorCode vec3_norm(const Vector3* const v, float* out);
MatrixErrorCode vec3_normalize(const Vector3* const v, Vector3* out);
MatrixErrorCode vec3_dot(const Vector3* const a, const Vector3* const b, float* out);
MatrixErrorCode vec3_cross(const Vector3* const a, const Vector3* const b, Vector3* out);

#endif
