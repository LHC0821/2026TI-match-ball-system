#include "pid.h"

#include <stdbool.h>
#include <assert.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

#define pid_abs(x)              ((x) >= 0.0f ? (x) : -(x))
#define pid_clamp(v, lo, hi)    ((v) > (hi) ? (hi) : ((v) < (lo) ? (lo) : (v)))

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief   初始化 Pid 控制器
 * @param   self      Pid 实例指针
 * @param   mode     Pid 模式 (PidMODExxx)
 * @param   features 功能特性 (PidFEATxxx 按位或)
 */
void pid_init(Pid* self, PidMode mode, PidFeature features) {
    assert(self != NULL);

    self->cfg.mode = mode;
    self->cfg.features = features;
    self->cfg.kp = 0.0f;  self->cfg.ki = 0.0f;  self->cfg.kd = 0.0f;

    self->cfg.max_out = 0.0f;
    self->cfg.integral_separation = 0.0f;
    self->cfg.dead_band = 0.0f;
    self->cfg.diff_filter_alpha = 0.0f;
    self->cfg.output_max_rate = 0.0f;
    self->ff_value = 0.0f;

    self->output = 0.0f;
    self->integral = 0.0f;
    self->prev_err = 0.0f;

    self->filtered_diff_ = 0.0f;
    self->prev_output_ = 0.0f;
    self->prev_measurement_ = 0.0f;
}

/**
 * @brief   通过配置表初始化 Pid 控制器
 * @param   self Pid 实例指针
 * @param   cfg 配置结构体指针
 */
void pid_init_cfg(Pid* self, const PidConfig* cfg) {
    assert(self != NULL);

    if(cfg == NULL) return;

    pid_init(self, cfg->mode, cfg->features);
    self->cfg.kp = cfg->kp;
    self->cfg.ki = cfg->ki;
    self->cfg.kd = cfg->kd;
    self->cfg.max_out = cfg->max_out;
    self->cfg.integral_separation = cfg->integral_separation;
    self->cfg.dead_band = cfg->dead_band;
    self->cfg.diff_filter_alpha = cfg->diff_filter_alpha;
    self->cfg.output_max_rate = cfg->output_max_rate;
}

/**
 * @brief   设置 Pid 增益
 */
void pid_set_gains(Pid* self, float kp, float ki, float kd) {
    assert(self != NULL);

    self->cfg.kp = kp;
    self->cfg.ki = ki;
    self->cfg.kd = kd;
}

/**
 * @brief   设置高级参数
 */
void pid_set_params(Pid* self, float max_out, float integral_separation,
    float dead_band, float diff_filter_alpha, float output_max_rate) {

    assert(self != NULL);

    self->cfg.max_out = max_out;
    self->cfg.integral_separation = integral_separation;
    self->cfg.dead_band = dead_band;
    self->cfg.diff_filter_alpha = diff_filter_alpha;
    self->cfg.output_max_rate = output_max_rate;
}

/**
 * @brief   设置前馈值
 */
void pid_set_feedforward(Pid* self, float ff_value) {
    assert(self != NULL);

    self->ff_value = ff_value;
}

/**
 * @brief   计算 Pid 输出
 * @param   self    Pid 实例指针
 * @param   target 目标值
 * @param   actual 实际值
 * @param   dts   时间间隔 (秒); 0 时积分离散累加, 微分项不计算
 * @return  Pid 输出值
 */
float pid_calculate(Pid* self, float setpoint, float measurement, float dt_s) {
    assert(self != NULL);

    float err = setpoint - measurement;
    PidFeature feat = self->cfg.features;
    PidMode mode = self->cfg.mode;

    /* 死区 */
    if((feat & PID_FEAT_DEADBAND) && pid_abs(err) < self->cfg.dead_band) {
        err = 0.0f;
    }

    float out = 0.0f;

    /* 比例项 */
    if(mode & PID_MODE_P) {
        out += self->cfg.kp * err;
    }

    /* 积分项 */
    if(mode & PID_MODE_I) {
        bool allow_integral = true;
        bool allow_separation = false;

        /* 积分抗饱和 : 条件积分法 (输出饱和且误差同向时禁止积分) */
        if(feat & PID_FEAT_ANTI_WINDUP) {
            if(self->prev_output_ >= self->cfg.max_out && err > 0.0f) {
                allow_integral = false;
            }
            if(self->prev_output_ <= -self->cfg.max_out && err < 0.0f) {
                allow_integral = false;
            }
        }

        if(allow_integral) {
            self->integral += (dt_s > 0.0f) ? (err * dt_s) : err;
        }

        /* 积分分离 (误差过大时不叠加积分输出) */
        if(feat & PID_FEAT_INTEGRAL_SEP) {
            if(pid_abs(err) > self->cfg.integral_separation) {
                allow_separation = true;
            }
        }

        if(!allow_separation) {
            out += self->cfg.ki * self->integral;
        }
    }

    /* 微分项 */
    if(mode & PID_MODE_D) {
        float diff;

        /* 微分先行: 基于测量值变化率, 避免目标突变时 D 项跳变 */
        if(feat & PID_FEAT_DIFF_ON_MEAS) {
            diff = (dt_s > 0.0f) ? (-(measurement - self->prev_measurement_) / dt_s) : 0.0f;
            self->prev_measurement_ = measurement;
        }
        else {
            diff = (dt_s > 0.0f) ? ((err - self->prev_err) / dt_s) : 0.0f;
        }

        /* 微分滤波: 一阶低通 */
        if(feat & PID_FEAT_DIFF_FILTER) {
            diff = self->cfg.diff_filter_alpha * diff
                + (1.0f - self->cfg.diff_filter_alpha) * self->filtered_diff_;
            self->filtered_diff_ = diff;
        }

        out += self->cfg.kd * diff;
    }

    /* 前馈 */
    if(feat & PID_FEAT_FEEDFORWARD) {
        out += self->ff_value;
    }

    /* 保存未限幅输出, 用于反计算法抗饱和 */
    float total_output = out;

    /* 输出限幅 */
    if(feat & PID_FEAT_OUTPUT_LIMIT) {
        out = pid_clamp(out, -self->cfg.max_out, self->cfg.max_out);
    }

    /* 输出变化率限制 */
    if(feat & PID_FEAT_OUTPUT_RATE_LIMIT) {
        if(dt_s > 0.0f) {
            float max_change = self->cfg.output_max_rate * dt_s;
            float delta = out - self->prev_output_;
            if(pid_abs(delta) > max_change) {
                out = self->prev_output_ + (delta > 0.0f ? max_change : -max_change);
            }
        }
    }

    /* 积分抗饱和 : 反计算法 (back-calculation) */
    if((mode & PID_MODE_I) && (feat & PID_FEAT_ANTI_WINDUP)
        && (feat & PID_FEAT_OUTPUT_LIMIT) && (dt_s > 0.0f)) {
        float output_diff = total_output - out;
        if(pid_abs(self->cfg.kp) > 1e-6f && pid_abs(self->cfg.ki) > 1e-6f) {
            float kb = self->cfg.ki / self->cfg.kp;  /* Kb = 1/Tt = Ki/Kp */
            self->integral -= output_diff * kb * dt_s;
        }
    }

    self->output = out;
    self->prev_output_ = out;
    self->prev_err = err;

    return out;
}

/**
 * @brief   重置 Pid 控制器状态 (不改变参数)
 */
void pid_reset(Pid* self) {
    assert(self != NULL);

    self->output = 0.0f;
    self->integral = 0.0f;
    self->prev_err = 0.0f;
    self->filtered_diff_ = 0.0f;
    self->prev_output_ = 0.0f;
    self->prev_measurement_ = 0.0f;
}
