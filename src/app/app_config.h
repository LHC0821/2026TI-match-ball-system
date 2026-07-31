#ifndef _app_config_h_
#define _app_config_h_

/**
 * @file app_config.h
 * @brief 滚球平衡应用默认参数
 *
 * @details
 * 本文件只保存应用真正需要的默认参数；IMU 轴与方向、视觉偏移方向和
 * 电机方向属于编译期配置；DM-4340 固定使用 MIT 模式
 * 无线调参只修改运行参数，不负责修改传感器安装方向
 *
 * @par 目录
 * - 1. 电机参数
 *   - 1.1 电机协议 ID
 *   - 1.2 电机应用零位和位置范围
 *   - 1.3 MIT 模式参数
 *   - 1.4 电机方向
 * - 2. 传感器方向参数
 *   - 2.1 加速度轴
 *   - 2.2 加速度、俯仰角和视觉球位方向
 * - 3. 平衡补偿参数
 *   - 3.1 平衡控制通道开关
 *   - 3.2 控制公式
 *   - 3.3 水平补偿和加速度前馈
 *   - 3.4 加速度、视觉滤波及视觉增益
 * - 4. 启动校准参数
 *   - 4.1 校准静置时间
 *   - 4.2 校准样本数量
 *   - 4.3 校准完成后的自动使能
 * - 5. 无线调参缓冲区
 *   - 5.1 单行命令缓冲区
 *   - 5.2 串口接收环形缓冲区
 *   - 5.3 调参回复缓冲区
 */

// ! ========================= 1. 电 机 参 数 ========================= ! //

/**
 * @brief DM-4340 电机协议 ID
 */
#define APP_CFG_MOTOR_ID 0x01u

/**
 * @brief 摆杆电机应用零位，单位 rad
 */
#define APP_CFG_MOTOR_ZERO_RAD 0.0f

/**
 * @brief 摆杆电机应用位置下限，单位 rad
 */
#define APP_CFG_MOTOR_MIN_RAD -0.345f

/**
 * @brief 摆杆电机应用位置上限，单位 rad
 */
#define APP_CFG_MOTOR_MAX_RAD 1.096f

/**
 * @brief MIT 模式目标速度，单位 rad/s
 */
#define APP_CFG_MOTOR_VEL_RAD_S 0.0f

/**
 * @brief MIT 模式位置比例系数
 */
#define APP_CFG_MOTOR_KP 10.0f

/**
 * @brief MIT 模式位置微分系数
 */
#define APP_CFG_MOTOR_KD 0.1f

/**
 * @brief MIT 模式前馈扭矩，单位 N*m
 */
#define APP_CFG_MOTOR_TOR_NM 0.0f

/**
 * @brief MIT 姿态扭矩补偿增益，单位 N*m/rad
 */
#define APP_CFG_TOR_PITCH_GAIN 0.0f

/**
 * @brief 控制角到电机方向的符号
 */
#define APP_CFG_MOTOR_SIGN 1.0f

// ! ========================= 2. 传 感 器 方 向 参 数 ========================= ! //

/**
 * @brief 纵向加速度使用的 IMU 轴
 */
#define APP_CFG_ACC_AXIS BALANCE_AXIS_X

/**
 * @brief 纵向加速度方向符号
 */
#define APP_CFG_ACC_SIGN 1.0f

/**
 * @brief 俯仰角方向符号
 */
#define APP_CFG_PITCH_SIGN 1.0f

/**
 * @brief 视觉球位偏移方向符号
 */
#define APP_CFG_BALL_SIGN 1.0f

// ! ========================= 3. 平 衡 补 偿 参 数 ========================= ! //

/**
 * @brief 平衡控制通道开关
 *
 * @details
 * 通过位掩码分别控制姿态水平补偿、纵向加速度前馈和视觉位置修正;
 * 关闭某个通道后，该通道对应的角度输出固定为 0，但传感器状态仍会继续更新;
 *
 * @par 可选项
 * 可按位或组合 `BALANCE_CTRL_LEVEL`、`BALANCE_CTRL_ACC` 和
 * `BALANCE_CTRL_VISION`；设置为 `BALANCE_CTRL_NONE` 时关闭全部补偿通道
 */
#define APP_CFG_CTRL_MASK ((uint8_t)(BALANCE_CTRL_LEVEL | BALANCE_CTRL_ACC | \
                                     BALANCE_CTRL_VISION))

/**
 * @details
 * MCU 端控制输出公式为：
 *
 * pitch_rel = pitch - pitch_zero
 *
 * acc_input = acc_axis - acc_zero
 *
 * acc_error = acc_input - acc_filt
 *
 * acc_tau = fast_mode ? acc_fast_tau_s : acc_tau_s
 *
 * acc_filt = LPF(acc_input, acc_tau)
 *
 * level_angle = level_gain * pitch_rel
 *
 * acc_angle = acc_gain * atan2(acc_filt, gravity)
 *
 * offset_filt = LPF(ball_sign * offset, vision_tau_s)
 *
 * offset_vel = ball_sign * vision_velocity
 *
 * vision_error = 0 - offset_filt
 *
 * vision_raw = vision_kp * vision_error
 *            + vision_ki * vision_integral
 *            - vision_kd * offset_vel
 *
 * vision_angle = clamp(vision_raw, -vision_max_angle, vision_max_angle)
 *
 * motor_cmd_raw = motor_zero
 *               + motor_sign * (level_angle + acc_angle + vision_angle)
 *
 * motor_cmd = clamp(motor_cmd_raw, motor_min, motor_max)
 *
 * tor_ff = clamp(motor_tor + tor_pitch_gain * pitch_rel, -28, 28)
 *
 * 其中 offset 是视觉端相对期望位置的偏移，MCU 不保存绝对保持位置
 */

/**
 * @brief 车体俯仰水平补偿增益
 */
#define APP_CFG_LEVEL_GAIN 11.5f

/**
 * @brief 纵向加速度前馈补偿增益
 */
#define APP_CFG_ACC_GAIN -15.0f

/**
 * @brief 加速度一阶低通时间常数，单位 s
 */
#define APP_CFG_ACC_TAU_S 0.1f

/**
 * @brief 加速度瞬态快速滤波时间常数，单位 s
 */
#define APP_CFG_ACC_FAST_TAU_S 0.015f

/**
 * @brief 加速度瞬态快速滤波进入误差阈值，单位 m/s2
 */
#define APP_CFG_ACC_FAST_ENTER_M_S2 0.20f

/**
 * @brief 加速度瞬态快速滤波退出误差阈值，单位 m/s2
 */
#define APP_CFG_ACC_FAST_EXIT_M_S2 0.10f

/**
 * @brief 视觉偏移一阶低通时间常数，单位 s
 *
 * @details
 * 0 表示直接使用视觉端最新有效偏移，避免视觉外环增加额外滞后
 */
#define APP_CFG_VISION_TAU_S 0.0f

/**
 * @brief 视觉偏移比例增益，单位 rad/px
 */
#define APP_CFG_VISION_KP 0.0018f

/**
 * @brief 视觉偏移积分增益，单位 rad/(px*s)
 */
#define APP_CFG_VISION_KI 0.0f

/**
 * @brief 视觉偏移微分增益，单位 rad*s/px
 */
#define APP_CFG_VISION_KD 0.00035f

/**
 * @brief 视觉控制角度限幅，单位 rad
 */
#define APP_CFG_VISION_MAX_ANGLE_RAD 0.05236f

// ! ========================= 4. 启 动 校 准 参 数 ========================= ! //

/**
 * @brief 上电校准前静置时间，单位 ms
 */
#define APP_CFG_CALIB_SETTLE_MS 4000u

/**
 * @brief 上电校准有效样本数量
 */
#define APP_CFG_CALIB_SAMPLES 250u

/**
 * @brief 校准完成后是否自动进入 ACTIVE
 */
#define APP_CFG_AUTO_ENABLE true

// ! ========================= 5. 无 线 调 参 缓 冲 区 ========================= ! //

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

#endif
