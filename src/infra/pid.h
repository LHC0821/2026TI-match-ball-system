#ifndef _pid_h_
#define _pid_h_

#include <stddef.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 *  PID 模式 (按位组合)
 */
typedef enum {
    PID_MODE_P = 0x04u,     // 0b100
    PID_MODE_I = 0x02u,     // 0b010
    PID_MODE_D = 0x01u,     // 0b001
    PID_MODE_PI = 0x06u,    // 0b110
    PID_MODE_PD = 0x05u,    // 0b101
    PID_MODE_PID = 0x07u    // 0b111
} PidMode;

/**
 * @brief PID 功能特性 (按位组合)
 */
typedef enum {
    PID_FEAT_NONE = 0x00u,
    PID_FEAT_OUTPUT_LIMIT = (1u << 0),      // 输出限幅
    PID_FEAT_INTEGRAL_SEP = (1u << 1),      // 积分分离
    PID_FEAT_DEADBAND = (1u << 2),          // 死区    
    PID_FEAT_DIFF_FILTER = (1u << 3),       // 微分滤波
    PID_FEAT_DIFF_ON_MEAS = (1u << 4),      // 微分先行
    PID_FEAT_ANTI_WINDUP = (1u << 5),       // 积分抗饱和
    PID_FEAT_OUTPUT_RATE_LIMIT = (1u << 6), // 输出变化率限制
    PID_FEAT_FEEDFORWARD = (1u << 7),       // 前馈控制
    PID_FEAT_ALL = 0xFFu
} PidFeature;

/**
 * @brief PID 配置结构体 (用于初始化)
 */
typedef struct {
    PidMode mode;                   // PID 模式, PID_MODE_xxx
    PidFeature features;            // 功能特性, PID_FEAT_xxx 按位或
    float kp;                       // 比例系数
    float ki;                       // 积分系数
    float kd;                       // 微分系数
    float max_out;                  // 最大输出值
    float integral_separation;      // 积分分离阈值
    float dead_band;                // 死区阈值
    float diff_filter_alpha;        // 微分滤波系数 (0~1)
    float output_max_rate;          // 输出最大变化率         
} PidConfig;

/**
 * @brief PID 控制器类
 */

typedef struct {
// public:
    PidConfig cfg;

    float ff_value;                 // 前馈值
    float output;                   // 当前输出
    float integral;                 // 积分累积值
    float prev_err;                 // 上一次误差

// private:
    float filtered_diff_;
    float prev_output_;
    float prev_measurement_;
} Pid;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void pid_init(Pid* self, PidMode mode, PidFeature features);
void pid_init_cfg(Pid* self, const PidConfig* cfg);
void pid_set_gains(Pid* self, float kp, float ki, float kd);
void pid_set_params(Pid* self, float max_out, float integral_separation, float dead_band, float diff_filter_alpha, float output_max_rate);
void pid_set_feedforward(Pid* self, float ff_value);
float pid_calculate(Pid* self, float setpoint, float measurement, float dt_s);
void pid_reset(Pid* self);

#define pid_create(name) static Pid name = { 0 }

#endif
