#ifndef _app_config_h_
#define _app_config_h_

/**
 * @file app_config.h
 * @brief 滚球平衡应用默认参数
 *
 * @details
 * 本文件只保存应用真正需要的默认参数；IMU 轴与方向、视觉偏移方向和
 * 电机方向属于编译期配置；DM-4340 MIT 的 kp、kd 和 tor 固定在驱动层
 * 无线调参只修改运行参数，不负责修改传感器安装方向
 *
 * @par 目录
 * - 1. 电机参数
 *   - 1.1 电机协议 ID
 *   - 1.2 电机应用零位和位置范围
 *   - 1.3 MIT 目标速度
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
#define APP_CFG_MOTOR_MIN_RAD -1.5f

/**
 * @brief 摆杆电机应用位置上限，单位 rad
 */
#define APP_CFG_MOTOR_MAX_RAD 1.5f

/**
 * @brief MIT 目标速度，单位 rad/s
 *
 * @details
 * MIT 控制帧中的速度字段只由该参数决定；DM-4340 的 kp、kd 和 tor
 * 使用驱动层固定的推荐值
 */
#define APP_CFG_MOTOR_SPEED_RAD_S 8.0f

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
 * motor_cmd = clamp(motor_zero + motor_sign * (level_angle + acc_angle + vision_angle),
 *                   motor_min, motor_max)
 *
 * level_angle = level_gain * pitch_rel
 *
 * pitch_rel = pitch - pitch_zero
 *
 * acc_filt = LPF(acc_axis - acc_zero + gravity * sin(pitch_rel), acc_tau_s)
 *
 * acc_angle = acc_gain * atan2(acc_filt, gravity)
 *
 * offset_filt = LPF(ball_sign * offset, vision_tau_s)
 *
 * offset_vel = d(offset_filt) / dt
 *
 * vision_angle = vision_kp * (0 - offset_filt) + vision_kd * (0 - offset_vel)
 *
 * 其中 offset 是视觉端相对期望位置的偏移，MCU 不保存绝对保持位置
 */

/**
 * @brief 车体俯仰水平补偿增益
 */
#define APP_CFG_LEVEL_GAIN -1.0f

/**
 * @brief 纵向加速度前馈补偿增益
 */
#define APP_CFG_ACC_GAIN -10.0f

/**
 * @brief 加速度一阶低通时间常数，单位 s
 */
#define APP_CFG_ACC_TAU_S 0.08f

/**
 * @brief 视觉偏移一阶低通时间常数，单位 s
 *
 * @details
 * 0 表示直接使用视觉端最新有效偏移，避免视觉外环增加额外滞后
 */
#define APP_CFG_VISION_TAU_S 0.0f

/**
 * @brief 视觉偏移比例增益
 */
#define APP_CFG_VISION_KP 1.8f

/**
 * @brief 视觉偏移微分增益
 */
#define APP_CFG_VISION_KD 0.35f

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
