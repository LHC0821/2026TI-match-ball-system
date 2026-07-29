#ifndef _app_config_h_
/**
 * @brief app_config.h 头文件包含保护宏
 */
#define _app_config_h_

/**
 * @file app_config.h
 * @brief 滚球平衡应用参数默认值与无线调参边界
 *
 * 固定通信参数仍由各驱动装配层管理，本文件只集中维护应用控制参数
 * 无线调参修改 RAM 中的运行参数，重新上电后恢复为本文件中的默认值
 */

// ! ========================= 默 认 参 数 ========================= ! //

/**
 * @brief 默认 DM-4340 电机协议 ID
 */
#define APP_CFG_MOTOR_ID 0x01u

/**
 * @brief 摆杆电机安装零位，单位 rad
 */
#define APP_CFG_MOTOR_ZERO_RAD 0.0f

/**
 * @brief 摆杆电机应用软下限，单位 rad
 */
#define APP_CFG_MOTOR_MIN_RAD -0.35f

/**
 * @brief 摆杆电机应用软上限，单位 rad
 */
#define APP_CFG_MOTOR_MAX_RAD 0.35f

/**
 * @brief POS_VEL 模式下发送给电机的位置命令速度参数，单位 rad/s
 */
#define APP_CFG_MOTOR_SPEED_RAD_S 1.0f

/**
 * @brief MCU 内部目标角变化率限制，单位 rad/s
 */
#define APP_CFG_MOTOR_RATE_RAD_S 1.5f

/**
 * @brief 控制角到电机方向的符号，取值为 +1 或 -1
 */
#define APP_CFG_MOTOR_SIGN 1.0f

/**
 * @brief 默认用于纵向加速度估计的 IMU 加速度轴
 */
#define APP_CFG_ACC_AXIS BALANCE_AXIS_X

/**
 * @brief 纵向加速度正方向符号，取值为 +1 或 -1
 */
#define APP_CFG_ACC_SIGN 1.0f

/**
 * @brief 俯仰角正方向符号，取值为 +1 或 -1
 */
#define APP_CFG_PITCH_SIGN 1.0f

/**
 * @brief 视觉球位正方向符号，取值为 +1 或 -1
 */
#define APP_CFG_BALL_SIGN 1.0f

/**
 * @brief 车体俯仰水平补偿增益
 */
#define APP_CFG_LEVEL_GAIN -1.0f

/**
 * @brief 纵向加速度前馈补偿增益
 */
#define APP_CFG_ACC_GAIN -1.0f

/**
 * @brief 纵向加速度一阶低通时间常数，单位 s
 */
#define APP_CFG_ACC_TAU_S 0.08f

/**
 * @brief 视觉球位一阶低通时间常数，单位 s
 */
#define APP_CFG_POS_TAU_S 0.03f

/**
 * @brief 视觉球速一阶低通时间常数，单位 s
 */
#define APP_CFG_VEL_TAU_S 0.08f

/**
 * @brief 球位闭环 PID 模式
 */
#define APP_CFG_POS_MODE PID_MODE_PID

/**
 * @brief 球位闭环 PID 启用的功能特性位
 */
#define APP_CFG_POS_FEATURES ((PidFeature)(PID_FEAT_OUTPUT_LIMIT | PID_FEAT_INTEGRAL_SEP | \
                                           PID_FEAT_DEADBAND | PID_FEAT_DIFF_FILTER |      \
                                           PID_FEAT_DIFF_ON_MEAS | PID_FEAT_ANTI_WINDUP |  \
                                           PID_FEAT_OUTPUT_RATE_LIMIT))

/**
 * @brief 球位 PID 比例系数
 */
#define APP_CFG_POS_KP 1.8f

/**
 * @brief 球位 PID 积分系数
 */
#define APP_CFG_POS_KI 0.05f

/**
 * @brief 球位 PID 微分系数
 */
#define APP_CFG_POS_KD 0.35f

/**
 * @brief 球位 PID 输出角限幅，单位 rad
 */
#define APP_CFG_POS_MAX_OUT_RAD 0.12f

/**
 * @brief 球位 PID 积分分离阈值，单位 m
 */
#define APP_CFG_POS_INT_SEP_M 0.02f

/**
 * @brief 球位 PID 死区阈值，单位 m
 */
#define APP_CFG_POS_DEAD_BAND_M 0.002f

/**
 * @brief 球位 PID 微分滤波系数，取值范围 0~1
 */
#define APP_CFG_POS_DIFF_ALPHA 0.20f

/**
 * @brief 球位 PID 输出变化率限制，单位 rad/s
 */
#define APP_CFG_POS_OUT_RATE_RAD_S 1.0f

/**
 * @brief 视觉修正角最大幅值，单位 rad
 */
#define APP_CFG_VISION_ANGLE_MAX_RAD 0.12f

/**
 * @brief 视觉失效后视觉修正角衰减时间常数，单位 s
 */
#define APP_CFG_VISION_DECAY_TAU_S 0.20f

/**
 * @brief 视觉命令允许设置的目标位置最大绝对值，单位 m
 */
#define APP_CFG_TARGET_LIMIT_M 0.10f

/**
 * @brief 视觉球位有效所需的最低质量分数
 */
#define APP_CFG_VISION_QUALITY_MIN 40u

/**
 * @brief 有效视觉球位超时时间，单位 ms
 */
#define APP_CFG_VISION_TIMEOUT_MS 200u

/**
 * @brief IMU 在线超时时间，单位 ms
 */
#define APP_CFG_IMU_TIMEOUT_MS 50u

/**
 * @brief 电机反馈在线超时时间，单位 ms
 */
#define APP_CFG_MOTOR_TIMEOUT_MS 300u

/**
 * @brief 往返测试目标位置幅值，单位 m
 */
#define APP_CFG_SWEEP_POS_M 0.05f

/**
 * @brief 往返测试判定到位的位置误差阈值，单位 m
 */
#define APP_CFG_SWEEP_ERR_M 0.01f

/**
 * @brief 往返测试判定稳定的球速阈值，单位 m/s
 */
#define APP_CFG_SWEEP_VEL_M_S 0.03f

/**
 * @brief 往返测试到位后要求保持稳定的时间，单位 ms
 */
#define APP_CFG_SWEEP_STABLE_MS 300u

/**
 * @brief 上电校准前静置等待时间，单位 ms
 */
#define APP_CFG_CALIB_SETTLE_MS 4000u

/**
 * @brief 上电校准累计的 IMU 有效样本数
 */
#define APP_CFG_CALIB_SAMPLES 250u

/**
 * @brief 校准完成后是否自动请求进入 ACTIVE
 */
#define APP_CFG_AUTO_ENABLE true

/**
 * @brief 默认启用的平衡控制通道位掩码
 */
#define APP_CFG_CTRL_MASK ((uint8_t)(BALANCE_CTRL_LEVEL | BALANCE_CTRL_ACC | \
                                     BALANCE_CTRL_VISION | BALANCE_CTRL_VISION_I))

// ! ========================= 无 线 调 参 边 界 ========================= ! //

/**
 * @brief 无线调参单行命令缓冲区长度，单位 byte
 */
#define APP_TUNE_LINE_SIZE 128u

/**
 * @brief 无线调参串口接收环形缓冲区长度，单位 byte
 */
#define APP_TUNE_RX_SIZE 256u

/**
 * @brief 无线调参回复缓冲区长度，单位 byte
 */
#define APP_TUNE_REPLY_SIZE 256u

/**
 * @brief 无线调参通用补偿增益最小值
 */
#define APP_TUNE_GAIN_MIN -5.0f

/**
 * @brief 无线调参通用补偿增益最大值
 */
#define APP_TUNE_GAIN_MAX 5.0f

/**
 * @brief 无线调参 PID 比例系数最小值
 */
#define APP_TUNE_KP_MIN 0.0f

/**
 * @brief 无线调参 PID 比例系数最大值
 */
#define APP_TUNE_KP_MAX 50.0f

/**
 * @brief 无线调参 PID 积分系数最小值
 */
#define APP_TUNE_KI_MIN 0.0f

/**
 * @brief 无线调参 PID 积分系数最大值
 */
#define APP_TUNE_KI_MAX 20.0f

/**
 * @brief 无线调参 PID 微分系数最小值
 */
#define APP_TUNE_KD_MIN 0.0f

/**
 * @brief 无线调参 PID 微分系数最大值
 */
#define APP_TUNE_KD_MAX 10.0f

/**
 * @brief 无线调参一阶低通时间常数最小值，单位 s
 */
#define APP_TUNE_TAU_MIN_S 0.0f

/**
 * @brief 无线调参一阶低通时间常数最大值，单位 s
 */
#define APP_TUNE_TAU_MAX_S 2.0f

/**
 * @brief 无线调参角度限幅最小值，单位 rad
 */
#define APP_TUNE_ANGLE_MIN_RAD 0.001f

/**
 * @brief 无线调参角度限幅最大值，单位 rad
 */
#define APP_TUNE_ANGLE_MAX_RAD 0.70f

/**
 * @brief 无线调参角速度或变化率最小值，单位 rad/s
 */
#define APP_TUNE_RATE_MIN_RAD_S 0.01f

/**
 * @brief 无线调参角速度或变化率最大值，单位 rad/s
 */
#define APP_TUNE_RATE_MAX_RAD_S 20.0f

/**
 * @brief 无线调参电机位置类参数最小值，单位 rad
 */
#define APP_TUNE_MOTOR_POS_MIN_RAD -1.57f

/**
 * @brief 无线调参电机位置类参数最大值，单位 rad
 */
#define APP_TUNE_MOTOR_POS_MAX_RAD 1.57f

/**
 * @brief 无线调参目标位置最大绝对值，单位 m
 */
#define APP_TUNE_TARGET_MAX_M 0.12f

/**
 * @brief 无线调参位置类限幅最大值，单位 m
 */
#define APP_TUNE_POS_LIMIT_MAX_M 0.15f

/**
 * @brief 无线调参视觉质量分数最大值
 */
#define APP_TUNE_QUALITY_MAX 100u

/**
 * @brief 无线调参超时时间最小值，单位 ms
 */
#define APP_TUNE_TIMEOUT_MIN_MS 20u

/**
 * @brief 无线调参超时时间最大值，单位 ms
 */
#define APP_TUNE_TIMEOUT_MAX_MS 5000u

/**
 * @brief 无线调参往返测试稳定保持时间最大值，单位 ms
 */
#define APP_TUNE_SWEEP_STABLE_MAX_MS 5000u

#endif
