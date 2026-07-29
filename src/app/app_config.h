#ifndef _app_config_h_
/**
 * @brief app_config.h 头文件包含保护宏
 *
 * @par 可选项
 * 说明: include guard only; no user configuration.
 */
#define _app_config_h_

/**
 * @file app_config.h
 * @brief 滚球平衡应用默认参数与无线调参边界
 *
 * @details
 * 本文件集中维护“应用层可调参数”的默认值，以及无线调参命令允许修改
 * 参数时使用的安全边界；固定外设通信参数仍由各驱动装配层管理，例如
 * FDCAN、SPI、UART 和 DMA 配置不放在这里
 *
 * 无线调参只修改 RAM 中的运行参数；重新上电后，系统会重新使用本文件
 * 中的默认值；因此这里的每一项都应能作为“出厂起调值”和“复位回退值”
 *
 * 目录：
 * - 1. 默认控制参数
 *   - 1.1 电机基础参数
 *   - 1.2 方向与坐标参数
 *   - 1.3 滤波参数
 *   - 1.4 球位 PID 参数
 *   - 1.5 视觉闭环与在线判断参数
 *   - 1.6 往返测试参数
 *   - 1.7 上电校准与默认控制通道
 * - 2. 无线调参边界
 *   - 2.1 无线调参通信缓冲区
 *   - 2.2 增益与 PID 边界
 *   - 2.3 滤波、角度、速度和位置边界
 *   - 2.4 视觉质量、超时和往返测试边界
 */

// ! ========================= 1. 默 认 控 制 参 数 ========================= ! //

// ! ========================= 1.1 电 机 基 础 参 数 ========================= ! //

/**
 * @brief 1.1.1 默认 DM-4340 电机协议 ID
 *
 * @details
 * 该 ID 用于 `assemble_balance.c` 填充 `BalanceConfig.motor_id`，平衡控制
 * 服务后续所有 `bus_motor.*` 命令都会发送到这个电机 ID
 *
 * @note 必须与达妙调试助手中写入的电机 CAN ID 一致若 ID 不一致，
 * 电机会收不到控制命令，反馈也会被 `assemble_motor_can_rx()` 过滤掉
 *
 * @par 可选项
 * 可选范围: 1..DM_MOTOR_MAX_ID; 0 is invalid; must match DM-4340 CAN ID.
 */
#define APP_CFG_MOTOR_ID 0x01u

/**
 * @brief 1.1.2 摆杆电机安装零位，单位 rad
 *
 * @details
 * 控制公式中的最终电机目标角会以该值为中心叠加水平补偿、加速度前馈
 * 和视觉 PID 修正；该值应对应摆杆机械中位或调试确认后的平衡零点
 *
 * @note 首次实机调试时应先确认机械不会撞限位，再微调该值
 *
 * @par 可选项
 * 可选范围: APP_CFG_MOTOR_MIN_RAD <= value <= APP_CFG_MOTOR_MAX_RAD; tune range
 * APP_TUNE_MOTOR_POS_MIN_RAD..APP_TUNE_MOTOR_POS_MAX_RAD.
 */
#define APP_CFG_MOTOR_ZERO_RAD 0.0f

/**
 * @brief 1.1.3 摆杆电机应用软下限，单位 rad
 *
 * @details
 * 平衡控制服务会在 `balance_update_control()` 中把最终电机目标角限制在
 * `[APP_CFG_MOTOR_MIN_RAD, APP_CFG_MOTOR_MAX_RAD]` 内
 *
 * @note 这是应用层软限位，不等同于 DM-4340 驱动器内部限位；调大前必须
 * 先确认连杆、摆杆和球管不会发生机械干涉
 *
 * @par 可选项
 * 可选范围: APP_TUNE_MOTOR_POS_MIN_RAD..APP_TUNE_MOTOR_POS_MAX_RAD; must be less
 * than APP_CFG_MOTOR_ZERO_RAD.
 */
#define APP_CFG_MOTOR_MIN_RAD -0.5f

/**
 * @brief 1.1.4 摆杆电机应用软上限，单位 rad
 *
 * @details
 * 与 `APP_CFG_MOTOR_MIN_RAD` 共同构成摆杆目标角的应用层安全窗口
 * 所有补偿项叠加后都会被限制在这个范围内
 *
 * @note 若发现控制输出长期顶在上限，应优先检查方向、零位和 PID 参数，
 * 不要直接扩大软限位
 *
 * @par 可选项
 * 可选范围: APP_TUNE_MOTOR_POS_MIN_RAD..APP_TUNE_MOTOR_POS_MAX_RAD; must be greater
 * than APP_CFG_MOTOR_ZERO_RAD.
 */
#define APP_CFG_MOTOR_MAX_RAD 0.5f

/**
 * @brief 1.1.5 MIT 模式目标速度参数，单位 rad/s
 *
 * @details
 * 该值作为 `bus_motor.set_pos_vel()` 的速度参数写入 MIT 控制帧的
 * 目标速度字段；位置参数仍由同一次调用写入 MIT 控制帧的位置字段
 *
 * @note 该值越大，电机响应越快，但机构冲击和球位过冲风险也会增大
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_RATE_MIN_RAD_S..APP_TUNE_RATE_MAX_RAD_S.
 */
#define APP_CFG_MOTOR_SPEED_RAD_S 1.0f

/**
 * @brief 1.1.6 MCU 内部目标角变化率限制，单位 rad/s
 *
 * @details
 * `balance_update_control()` 会按控制周期 `dt_s` 限制相邻两次电机目标角
 * 的变化量，避免视觉误检或状态切换造成摆杆目标突变
 *
 * @note 该限制发生在发送给电机前，是 MCU 应用层的斜率限制
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_RATE_MIN_RAD_S..APP_TUNE_RATE_MAX_RAD_S.
 */
#define APP_CFG_MOTOR_RATE_RAD_S 1.5f

/**
 * @brief 1.1.7 控制角到电机方向的符号
 *
 * @details
 * 取值应为 `+1.0f` 或 `-1.0f`；最终电机命令为：
 * `motor_zero + motor_sign * (level + acc + vision)`
 *
 * @note 如果控制方向整体相反，优先修改本项，而不是在控制公式中散落负号
 *
 * @par 可选项
 * 可选值: +1.0f or -1.0f.
 */
#define APP_CFG_MOTOR_SIGN 1.0f

// ! ========================= 1.2 方 向 与 坐 标 参 数 ========================= ! //

/**
 * @brief 1.2.1 用于纵向加速度估计的 IMU 加速度轴
 *
 * @details
 * 平衡服务会从 `ImuAcc` 中选取该轴作为车辆纵向加速度原始量，再乘以
 * `APP_CFG_ACC_SIGN`；默认假设 BMI088 的 X 轴为车辆纵向轴
 *
 * @note 实机安装方向改变时，只改该配置，不应改 `balance_get_acc_axis()`
 *
 * @par 可选项
 * 可选值: BALANCE_AXIS_X, BALANCE_AXIS_Y, BALANCE_AXIS_Z.
 */
#define APP_CFG_ACC_AXIS BALANCE_AXIS_X

/**
 * @brief 1.2.2 纵向加速度正方向符号
 *
 * @details
 * 取值应为 `+1.0f` 或 `-1.0f`，用于把 IMU 安装坐标转换为控制坐标
 * 小车朝定义的正方向加速时，滤波后的 `acc_m_s2` 应呈现期望正号
 *
 * @par 可选项
 * 可选值: +1.0f or -1.0f.
 */
#define APP_CFG_ACC_SIGN 1.0f

/**
 * @brief 1.2.3 俯仰角正方向符号
 *
 * @details
 * 取值应为 `+1.0f` 或 `-1.0f`，用于修正 BMI088 姿态解算输出到车体
 * 俯仰控制坐标的方向
 *
 * @note 手动抬高车头或车尾时，应通过日志确认 `pitch_rad` 的符号是否符合预期
 *
 * @par 可选项
 * 可选值: +1.0f or -1.0f.
 */
#define APP_CFG_PITCH_SIGN 1.0f

/**
 * @brief 1.2.4 视觉球位正方向符号
 *
 * @details
 * 取值应为 `+1.0f` 或 `-1.0f`，用于把视觉端上报的 `pos_mm` 转换为
 * 平衡控制内部的球位坐标
 *
 * @note 如果球在画面/机构中向正方向移动，但 MCU 日志中的 `ball` 方向相反，
 * 应修改本项
 *
 * @par 可选项
 * 可选值: +1.0f or -1.0f.
 */
#define APP_CFG_BALL_SIGN 1.0f

/**
 * @brief 1.2.5 车体俯仰水平补偿增益
 *
 * @details
 * 该增益作用于上电校准后的相对俯仰角 `pitch_rel`，生成
 * `level_angle_rad`；用于在车体前后倾斜时让摆杆保持相对水平
 *
 * @note 方向错误时通常先改正负号；幅值不合适时再调大小
 *
 * @par 可选项
 * 可选范围: finite float; tune range APP_TUNE_GAIN_MIN..APP_TUNE_GAIN_MAX.
 */
#define APP_CFG_LEVEL_GAIN -1.0f

/**
 * @brief 1.2.6 纵向加速度前馈补偿增益
 *
 * @details
 * 该增益作用于纵向线加速度估计，生成 `acc_angle_rad`用于车辆加速
 * 或制动时提前改变摆杆角度，减小钢球因惯性产生的位移
 *
 * @note 前馈方向必须在无球或低风险状态下先验证，再进入视觉闭环调试
 *
 * @par 可选项
 * 可选范围: finite float; tune range APP_TUNE_GAIN_MIN..APP_TUNE_GAIN_MAX.
 */
#define APP_CFG_ACC_GAIN -1.0f

// ! ========================= 1.3 滤 波 参 数 ========================= ! //

/**
 * @brief 1.3.1 纵向加速度一阶低通时间常数，单位 s
 *
 * @details
 * 用于滤波 `acc_linear`，得到参与前馈的 `acc_m_s2`；值越大，前馈越平滑，
 * 但响应越慢；值越小，响应越快，但更容易把振动和噪声引入摆杆
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_TAU_MIN_S..APP_TUNE_TAU_MAX_S; 0 disables
 * filtering.
 */
#define APP_CFG_ACC_TAU_S 0.08f

/**
 * @brief 1.3.2 视觉球位一阶低通时间常数，单位 s
 *
 * @details
 * 用于滤波视觉端上报的小球位置；视觉帧率约 30 Hz 时，默认值用于在
 * 噪声抑制和位置响应之间折中
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_TAU_MIN_S..APP_TUNE_TAU_MAX_S; 0 disables
 * filtering.
 */
#define APP_CFG_POS_TAU_S 0.03f

/**
 * @brief 1.3.3 视觉球速一阶低通时间常数，单位 s
 *
 * @details
 * 球速由滤波后球位差分得到，再经过本时间常数低通；该速度用于
 * `SWEEP_TEST` 稳定判定和状态观测
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_TAU_MIN_S..APP_TUNE_TAU_MAX_S; 0 disables
 * filtering.
 */
#define APP_CFG_VEL_TAU_S 0.08f

// ! ========================= 1.4 球 位 PID 参 数 ========================= ! //

/**
 * @brief 1.4.1 球位闭环 PID 模式
 *
 * @details
 * 默认启用完整 PID 三项；该宏直接写入 `PidConfig.mode`，由 `pid_calculate()`
 * 决定比例、积分、微分项是否参与输出
 *
 * @par 可选项
 * 可选值: PID_MODE_P, PID_MODE_I, PID_MODE_D, PID_MODE_PI, PID_MODE_PD,
 * PID_MODE_PID.
 */
#define APP_CFG_POS_MODE PID_MODE_PID

/**
 * @brief 1.4.2 球位闭环 PID 功能特性位
 *
 * @details
 * 默认启用输出限幅、积分分离、死区、微分滤波、对测量值微分、抗积分饱和
 * 和输出变化率限制；该组合是为了降低视觉噪声和突发误差对摆杆的冲击
 *
 * @par 可选项
 * 可选值: bitwise OR of PID_FEAT_*; may also use PID_FEAT_NONE or PID_FEAT_ALL.
 */
#define APP_CFG_POS_FEATURES ((PidFeature)(PID_FEAT_OUTPUT_LIMIT | PID_FEAT_INTEGRAL_SEP | \
                                           PID_FEAT_DEADBAND | PID_FEAT_DIFF_FILTER |      \
                                           PID_FEAT_DIFF_ON_MEAS | PID_FEAT_ANTI_WINDUP |  \
                                           PID_FEAT_OUTPUT_RATE_LIMIT))

/**
 * @brief 1.4.3 球位 PID 比例系数
 *
 * @details
 * 比例项根据 `target_m - ball_pos_m` 直接产生视觉修正角；增大该值会提高
 * 回球力度，但过大容易造成振荡和过冲
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_KP_MIN..APP_TUNE_KP_MAX.
 */
#define APP_CFG_POS_KP 1.8f

/**
 * @brief 1.4.4 球位 PID 积分系数
 *
 * @details
 * 积分项用于消除机械零位、摩擦或安装偏差带来的静差；默认值较小，
 * 并配合积分分离和抗饱和使用
 *
 * @note 调试初期可临时设为 0，先完成方向、比例和微分调试
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_KI_MIN..APP_TUNE_KI_MAX.
 */
#define APP_CFG_POS_KI 0.05f

/**
 * @brief 1.4.5 球位 PID 微分系数
 *
 * @details
 * 微分项用于抑制球位变化速度带来的过冲；当前 PID 配置使用
 * `PID_FEAT_DIFF_ON_MEAS`，因此微分主要作用在测量变化上
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_KD_MIN..APP_TUNE_KD_MAX.
 */
#define APP_CFG_POS_KD 0.35f

/**
 * @brief 1.4.6 球位 PID 输出角限幅，单位 rad
 *
 * @details
 * 限制视觉闭环最多能贡献多少摆杆角度；最终电机目标还会再次经过
 * `APP_CFG_MOTOR_MIN_RAD` 和 `APP_CFG_MOTOR_MAX_RAD` 的总软限位
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_ANGLE_MIN_RAD..APP_TUNE_ANGLE_MAX_RAD.
 */
#define APP_CFG_POS_MAX_OUT_RAD 0.12f

/**
 * @brief 1.4.7 球位 PID 积分分离阈值，单位 m
 *
 * @details
 * 当球位误差超过该阈值时，PID 不累计积分，避免大偏差阶段积分堆积
 * 当球接近目标后，积分项才用于修正静差
 *
 * @par 可选项
 * 可选范围: >= 0; tune range 0..APP_TUNE_POS_LIMIT_MAX_M.
 */
#define APP_CFG_POS_INT_SEP_M 0.02f

/**
 * @brief 1.4.8 球位 PID 死区阈值，单位 m
 *
 * @details
 * 当球位误差绝对值小于该阈值时，PID 认为误差为 0；用于避免视觉小噪声
 * 让摆杆持续微动
 *
 * @par 可选项
 * 可选范围: >= 0; tune range 0..APP_TUNE_POS_LIMIT_MAX_M.
 */
#define APP_CFG_POS_DEAD_BAND_M 0.002f

/**
 * @brief 1.4.9 球位 PID 微分滤波系数
 *
 * @details
 * 取值范围为 0~1值越大，新微分量占比越高，响应更快但更敏感；
 * 值越小，微分更平滑但滞后更明显
 *
 * @par 可选项
 * 可选范围: 0..1; 0 is smoothest, 1 uses newest diff directly.
 */
#define APP_CFG_POS_DIFF_ALPHA 0.20f

/**
 * @brief 1.4.10 球位 PID 输出变化率限制，单位 rad/s
 *
 * @details
 * 限制视觉 PID 输出角的变化速度，避免单帧视觉跳变直接转化为摆杆突变
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_RATE_MIN_RAD_S..APP_TUNE_RATE_MAX_RAD_S.
 */
#define APP_CFG_POS_OUT_RATE_RAD_S 1.0f

// ! ========================= 1.5 视 觉 闭 环 与 在 线 判 断 参 数 ========================= ! //

/**
 * @brief 1.5.1 视觉修正角最大幅值，单位 rad
 *
 * @details
 * 平衡服务会再次将视觉 PID 输出限制到该幅值内；通常应与
 * `APP_CFG_POS_MAX_OUT_RAD` 保持一致或更保守
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_ANGLE_MIN_RAD..APP_TUNE_ANGLE_MAX_RAD.
 */
#define APP_CFG_VISION_ANGLE_MAX_RAD 0.12f

/**
 * @brief 1.5.2 视觉失效后的修正角衰减时间常数，单位 s
 *
 * @details
 * 当视觉球位无效或超时时，视觉修正角不会立即清零，而是按该时间常数
 * 衰减到 0，以避免摆杆目标发生阶跃
 *
 * @par 可选项
 * 可选范围: >= 0; tune range APP_TUNE_TAU_MIN_S..APP_TUNE_TAU_MAX_S; 0 means
 * immediate decay.
 */
#define APP_CFG_VISION_DECAY_TAU_S 0.20f

/**
 * @brief 1.5.3 视觉命令目标位置最大绝对值，单位 m
 *
 * @details
 * `HOLD_POS` 模式下，视觉端下发的目标位置会被限制在
 * `[-APP_CFG_TARGET_LIMIT_M, +APP_CFG_TARGET_LIMIT_M]`
 *
 * @par 可选项
 * 可选范围: > 0; tune range 0.001f..APP_TUNE_TARGET_MAX_M.
 */
#define APP_CFG_TARGET_LIMIT_M 0.10f

/**
 * @brief 1.5.4 视觉球位有效所需的最低质量分数
 *
 * @details
 * 视觉端 `VIS_BALL` 帧中的 `quality` 必须大于或等于该值，并且 `VALID`
 * 标志有效，MCU 才会把本帧球位送入位置滤波和 PID
 *
 * @par 可选项
 * 可选范围: 0..APP_TUNE_QUALITY_MAX.
 */
#define APP_CFG_VISION_QUALITY_MIN 40u

/**
 * @brief 1.5.5 有效视觉球位超时时间，单位 ms
 *
 * @details
 * 超过该时间没有收到有效球位时，`vision_valid` 会变为 false，视觉修正角
 * 进入衰减；该情况不会直接触发 `FAULT`
 *
 * @par 可选项
 * 可选范围: > 0; tune range APP_TUNE_TIMEOUT_MIN_MS..APP_TUNE_TIMEOUT_MAX_MS.
 */
#define APP_CFG_VISION_TIMEOUT_MS 200u

/**
 * @brief 1.5.6 IMU 在线超时时间，单位 ms
 *
 * @details
 * 控制周期中若超过该时间没有有效 IMU 更新，会触发 `BALANCE_FAULT_IMU_TIMEOUT`
 *
 * @par 可选项
 * 可选范围: > 0; not exposed by wireless tuning.
 */
#define APP_CFG_IMU_TIMEOUT_MS 50u

/**
 * @brief 1.5.7 电机反馈在线超时时间，单位 ms
 *
 * @details
 * 已开始发送电机命令后，若超过该时间没有收到匹配电机反馈，会触发
 * `BALANCE_FAULT_MOTOR_TIMEOUT`
 *
 * @par 可选项
 * 可选范围: > 0; not exposed by wireless tuning.
 */
#define APP_CFG_MOTOR_TIMEOUT_MS 300u

// ! ========================= 1.6 往 返 测 试 参 数 ========================= ! //

/**
 * @brief 1.6.1 往返测试目标位置幅值，单位 m
 *
 * @details
 * `SWEEP_TEST` 模式先将目标设为 `+APP_CFG_SWEEP_POS_M`，稳定后切换到
 * `-APP_CFG_SWEEP_POS_M`
 *
 * @par 可选项
 * 可选范围: > 0 and <= target_limit_m; tune range 0.001f..APP_TUNE_TARGET_MAX_M.
 */
#define APP_CFG_SWEEP_POS_M 0.05f

/**
 * @brief 1.6.2 往返测试到位位置误差阈值，单位 m
 *
 * @details
 * 当 `|target_m - ball_pos_m|` 小于或等于该值时，位置条件被认为满足
 *
 * @par 可选项
 * 可选范围: > 0 and < sweep_pos_m; tune range 0.0005f..APP_TUNE_TARGET_MAX_M.
 */
#define APP_CFG_SWEEP_ERR_M 0.01f

/**
 * @brief 1.6.3 往返测试稳定球速阈值，单位 m/s
 *
 * @details
 * 当 `|ball_vel_m_s|` 小于或等于该值时，速度条件被认为满足
 *
 * @par 可选项
 * 可选范围: > 0; tune range 0.001f..1.0f.
 */
#define APP_CFG_SWEEP_VEL_M_S 0.03f

/**
 * @brief 1.6.4 往返测试稳定保持时间，单位 ms
 *
 * @details
 * 位置条件、速度条件和视觉有效条件同时满足并持续该时间后，测试目标
 * 从正向位置切换到负向位置
 *
 * @par 可选项
 * 可选范围: > 0; tune range 1..APP_TUNE_SWEEP_STABLE_MAX_MS.
 */
#define APP_CFG_SWEEP_STABLE_MS 300u

// ! ========================= 1.7 上 电 校 准 与 默 认 控 制 通 道 ========================= ! //

/**
 * @brief 1.7.1 上电校准前静置等待时间，单位 ms
 *
 * @details
 * 进入 `CALIB` 后先等待该时间，再开始累计 IMU 样本；用于避开上电、
 * 外设初始化和人工松手带来的瞬态扰动
 *
 * @par 可选项
 * 可选范围: >= 0; not exposed by wireless tuning.
 */
#define APP_CFG_CALIB_SETTLE_MS 4000u

/**
 * @brief 1.7.2 上电校准累计的 IMU 有效样本数
 *
 * @details
 * 平衡服务会累计该数量的有效 IMU 样本，用于计算启动时俯仰零偏和
 * 纵向加速度零偏
 *
 * @par 可选项
 * 可选范围: > 0; not exposed by wireless tuning.
 */
#define APP_CFG_CALIB_SAMPLES 250u

/**
 * @brief 1.7.3 校准完成后是否自动请求进入 ACTIVE
 *
 * @details
 * 为 true 时，校准完成且依赖在线后会自动请求进入 `ACTIVE`；为 false 时，
 * 系统停留在 `READY`，等待视觉端或无线调参命令使能
 *
 * @par 可选项
 * 可选值: true or false.
 */
#define APP_CFG_AUTO_ENABLE true

/**
 * @brief 1.7.4 默认启用的平衡控制通道位掩码
 *
 * @details
 * 默认同时启用水平补偿、加速度前馈、视觉修正和视觉积分通道
 * 无线调参可通过 `CTRL LEVEL|ACC|VISION|VIS_I 0|1` 临时开关这些通道
 *
 * @par 可选项
 * 可选值: bitwise OR of BALANCE_CTRL_LEVEL, BALANCE_CTRL_ACC, BALANCE_CTRL_VISION,
 * BALANCE_CTRL_VISION_I; 0 disables all channels for debug only.
 */
#define APP_CFG_CTRL_MASK ((uint8_t)(BALANCE_CTRL_LEVEL | BALANCE_CTRL_ACC | \
                                     BALANCE_CTRL_VISION | BALANCE_CTRL_VISION_I))

// ! ========================= 2. 无 线 调 参 边 界 ========================= ! //

// ! ========================= 2.1 无 线 调 参 通 信 缓 冲 区 ========================= ! //

/**
 * @brief 2.1.1 无线调参单行命令缓冲区长度，单位 byte
 *
 * @details
 * `app_tune_process()` 会把 USART1 收到的可打印字符累积成一行命令
 * 单行超过该长度时，本行会被丢弃并返回 reject
 *
 * @par 可选项
 * 可选范围: > 1; should fit the longest command plus NUL.
 */
#define APP_TUNE_LINE_SIZE 128u

/**
 * @brief 2.1.2 无线调参串口接收环形缓冲区长度，单位 byte
 *
 * @details
 * USART1 接收完成回调只把字节写入该环形缓冲，命令解析在主循环中完成
 * 缓冲区过小会增加高频输入时的溢出风险
 *
 * @par 可选项
 * 可选范围: > 1; should cover bytes received during worst-case main-loop latency.
 */
#define APP_TUNE_RX_SIZE 256u

/**
 * @brief 2.1.3 无线调参回复缓冲区长度，单位 byte
 *
 * @details
 * 每条 `[TUNE]` 回复会先格式化到该缓冲区，再通过 USART1 发送
 * 若回复内容过长会被截断到缓冲区容量内
 *
 * @par 可选项
 * 可选范围: > longest reply; too small truncates [TUNE] responses.
 */
#define APP_TUNE_REPLY_SIZE 256u

// ! ========================= 2.2 增 益 与 PID 边 界 ========================= ! //

/**
 * @brief 2.2.1 无线调参通用补偿增益最小值
 *
 * @details
 * 限制 `LEVEL_GAIN` 和 `ACC_GAIN` 的最小值，避免无线调参写入过大的负增益
 *
 * @par 可选项
 * 可选范围: finite float; must be less than APP_TUNE_GAIN_MAX.
 */
#define APP_TUNE_GAIN_MIN -5.0f

/**
 * @brief 2.2.2 无线调参通用补偿增益最大值
 *
 * @details
 * 限制 `LEVEL_GAIN` 和 `ACC_GAIN` 的最大值，避免补偿项过大导致摆杆打满
 *
 * @par 可选项
 * 可选范围: finite float; must be greater than APP_TUNE_GAIN_MIN.
 */
#define APP_TUNE_GAIN_MAX 5.0f

/**
 * @brief 2.2.3 无线调参 PID 比例系数最小值
 *
 * @details
 * 限制 `KP` 的最小值；比例系数不允许为负，方向应通过 `BALL_SIGN`
 * 或电机方向参数调整
 *
 * @par 可选项
 * 可选范围: >= 0 and < APP_TUNE_KP_MAX.
 */
#define APP_TUNE_KP_MIN 0.0f

/**
 * @brief 2.2.4 无线调参 PID 比例系数最大值
 *
 * @details
 * 限制 `KP` 的最大值，避免单次无线命令写入过强的回球力度
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_KP_MIN.
 */
#define APP_TUNE_KP_MAX 50.0f

/**
 * @brief 2.2.5 无线调参 PID 积分系数最小值
 *
 * @details
 * 限制 `KI` 的最小值；积分系数不允许为负，避免静差修正方向异常
 *
 * @par 可选项
 * 可选范围: >= 0 and < APP_TUNE_KI_MAX.
 */
#define APP_TUNE_KI_MIN 0.0f

/**
 * @brief 2.2.6 无线调参 PID 积分系数最大值
 *
 * @details
 * 限制 `KI` 的最大值，降低积分累积过快造成慢性过冲的风险
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_KI_MIN.
 */
#define APP_TUNE_KI_MAX 20.0f

/**
 * @brief 2.2.7 无线调参 PID 微分系数最小值
 *
 * @details
 * 限制 `KD` 的最小值；微分系数不允许为负，避免速度抑制变成正反馈
 *
 * @par 可选项
 * 可选范围: >= 0 and < APP_TUNE_KD_MAX.
 */
#define APP_TUNE_KD_MIN 0.0f

/**
 * @brief 2.2.8 无线调参 PID 微分系数最大值
 *
 * @details
 * 限制 `KD` 的最大值，避免视觉噪声经微分放大后驱动摆杆抖动
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_KD_MIN.
 */
#define APP_TUNE_KD_MAX 10.0f

// ! ========================= 2.3 滤 波、角 度、速 度 和 位 置 边 界 ========================= ! //

/**
 * @brief 2.3.1 无线调参一阶低通时间常数最小值，单位 s
 *
 * @details
 * 限制 `ACC_TAU`、`POS_TAU`、`VEL_TAU` 和 `VIS_FADE` 的最小值
 * 允许为 0，表示对应滤波退化为直接使用输入值
 *
 * @par 可选项
 * 可选范围: >= 0 and < APP_TUNE_TAU_MAX_S.
 */
#define APP_TUNE_TAU_MIN_S 0.0f

/**
 * @brief 2.3.2 无线调参一阶低通时间常数最大值，单位 s
 *
 * @details
 * 限制滤波时间常数最大值，避免滤波过慢导致控制响应明显滞后
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_TAU_MIN_S.
 */
#define APP_TUNE_TAU_MAX_S 2.0f

/**
 * @brief 2.3.3 无线调参角度限幅最小值，单位 rad
 *
 * @details
 * 限制 `POS_MAX` 和 `VIS_MAX` 的最小值该值大于 0，避免视觉闭环输出
 * 被设置成完全不可动作
 *
 * @par 可选项
 * 可选范围: > 0 and < APP_TUNE_ANGLE_MAX_RAD.
 */
#define APP_TUNE_ANGLE_MIN_RAD 0.001f

/**
 * @brief 2.3.4 无线调参角度限幅最大值，单位 rad
 *
 * @details
 * 限制 `POS_MAX` 和 `VIS_MAX` 的最大值，避免视觉闭环贡献过大的摆杆角度
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_ANGLE_MIN_RAD.
 */
#define APP_TUNE_ANGLE_MAX_RAD 0.70f

/**
 * @brief 2.3.5 无线调参角速度或变化率最小值，单位 rad/s
 *
 * @details
 * 限制 `MOTOR_SPEED`、`MOTOR_RATE` 和 `POS_RATE` 的最小值，避免调到过小
 * 后电机目标几乎不动
 *
 * @par 可选项
 * 可选范围: > 0 and < APP_TUNE_RATE_MAX_RAD_S.
 */
#define APP_TUNE_RATE_MIN_RAD_S 0.01f

/**
 * @brief 2.3.6 无线调参角速度或变化率最大值，单位 rad/s
 *
 * @details
 * 限制 `MOTOR_SPEED`、`MOTOR_RATE` 和 `POS_RATE` 的最大值，避免摆杆速度
 * 或目标变化率过高
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_RATE_MIN_RAD_S.
 */
#define APP_TUNE_RATE_MAX_RAD_S 20.0f

/**
 * @brief 2.3.7 无线调参电机位置类参数最小值，单位 rad
 *
 * @details
 * 限制 `MOTOR_ZERO`、`MOTOR_MIN` 和 `MOTOR_MAX` 的最小值这是无线调参
 * 层额外保护范围，实际运行还会使用应用软限位
 *
 * @par 可选项
 * 可选范围: finite float; must be less than APP_TUNE_MOTOR_POS_MAX_RAD.
 */
#define APP_TUNE_MOTOR_POS_MIN_RAD -1.57f

/**
 * @brief 2.3.8 无线调参电机位置类参数最大值，单位 rad
 *
 * @details
 * 限制 `MOTOR_ZERO`、`MOTOR_MIN` 和 `MOTOR_MAX` 的最大值，防止无线命令
 * 写入明显超出机构可接受范围的位置
 *
 * @par 可选项
 * 可选范围: finite float; must be greater than APP_TUNE_MOTOR_POS_MIN_RAD.
 */
#define APP_TUNE_MOTOR_POS_MAX_RAD 1.57f

/**
 * @brief 2.3.9 无线调参目标位置最大绝对值，单位 m
 *
 * @details
 * 限制 `TARGET_LIMIT` 和 `SWEEP_POS` 的调参范围；该边界应不超过视觉标定
 * 和机械球管实际可控范围
 *
 * @par 可选项
 * 可选范围: > 0; should not exceed visual calibration and mechanical control range.
 */
#define APP_TUNE_TARGET_MAX_M 0.12f

/**
 * @brief 2.3.10 无线调参位置类限幅最大值，单位 m
 *
 * @details
 * 限制 `INT_SEP` 和 `DEAD_BAND` 等位置误差阈值类参数的最大值
 *
 * @par 可选项
 * 可选范围: > 0; caps position-threshold parameters such as INT_SEP and DEAD_BAND.
 */
#define APP_TUNE_POS_LIMIT_MAX_M 0.15f

// ! ========================= 2.4 视 觉 质 量、超 时 和 往 返 测 试 边 界 ========================= ! //

/**
 * @brief 2.4.1 无线调参视觉质量分数最大值
 *
 * @details
 * 限制 `QUALITY_MIN` 的最大值；视觉质量分数按 0~100 约定解释
 *
 * @par 可选项
 * 可选范围: protocol quality score maximum; normally 100u.
 */
#define APP_TUNE_QUALITY_MAX 100u

/**
 * @brief 2.4.2 无线调参超时时间最小值，单位 ms
 *
 * @details
 * 限制 `VIS_TIMEOUT` 的最小值，避免设置得过短导致正常视觉帧间隔也被误判失联
 *
 * @par 可选项
 * 可选范围: > 0 and < APP_TUNE_TIMEOUT_MAX_MS.
 */
#define APP_TUNE_TIMEOUT_MIN_MS 20u

/**
 * @brief 2.4.3 无线调参超时时间最大值，单位 ms
 *
 * @details
 * 限制 `VIS_TIMEOUT` 的最大值，避免视觉长时间失效后系统仍认为球位有效
 *
 * @par 可选项
 * 可选范围: > APP_TUNE_TIMEOUT_MIN_MS.
 */
#define APP_TUNE_TIMEOUT_MAX_MS 5000u

/**
 * @brief 2.4.4 无线调参往返测试稳定保持时间最大值，单位 ms
 *
 * @details
 * 限制 `SWEEP_STABLE` 的最大值，避免测试模式因为保持时间过长而迟迟不切换目标
 *
 * @par 可选项
 * 可选范围: > 0.
 */
#define APP_TUNE_SWEEP_STABLE_MAX_MS 5000u

#endif
