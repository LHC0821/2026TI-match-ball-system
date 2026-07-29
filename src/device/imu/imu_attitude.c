#include "imu_attitude.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define IMU_ATTITUDE_PI                       3.14159265358979323846f
#define IMU_ATTITUDE_2PI                      (2.0f * IMU_ATTITUDE_PI)

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float imu_attitude_wrap_pi(float angle);
static float imu_attitude_acc_norm(const ImuAcc* acc);
static bool imu_attitude_acc_is_trusted(const ImuAttitude* attitude, const ImuAcc* acc);
static bool imu_attitude_acc_can_fuse(const ImuAttitude* attitude, const ImuSample* sample, float* acc_norm, uint32_t* acc_age_us);
static void imu_attitude_quat_normalize(ImuQuat* quat);
static void imu_attitude_quat_from_angle(ImuQuat* quat, const ImuAngle* angle);
static void imu_attitude_angle_from_quat(ImuAngle* angle, const ImuQuat* quat);
static void imu_attitude_update_angle_by_acc(ImuAttitude* attitude, const ImuAcc* acc, bool reset_yaw);
static void imu_attitude_integrate_calibrating_yaw(ImuAttitude* attitude, const ImuSample* sample);
static void imu_attitude_reset_calibration(ImuAttitude* attitude);
static ImuAttitudeStatus imu_attitude_calibrate_gyro(ImuAttitude* attitude, const ImuSample* sample);
static ImuGyro imu_attitude_get_temp_comp(const ImuAttitude* attitude, const ImuSample* sample);
static float imu_attitude_get_z_bias_offset(const ImuAttitude* attitude);
static bool imu_attitude_z_bias_model_enabled(const ImuAttitude* attitude);
static ImuGyro imu_attitude_get_corrected_gyro(ImuAttitude* attitude, const ImuSample* sample);
static void imu_attitude_update_complementary(ImuAttitude* attitude, const ImuSample* sample, float dt);
static void imu_attitude_update_mahony(ImuAttitude* attitude, const ImuSample* sample, float dt);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ImuAttitudeStatus imu_attitude_init(ImuAttitude* attitude, const ImuAttitudeConfig* config) {
    if(attitude == 0 || config == 0 || config->mode == IMU_ATTITUDE_NONE) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    memset(attitude, 0, sizeof(ImuAttitude));
    attitude->config = *config;
    attitude->quat.w = 1.0f;
    attitude->zru_enabled = false;

    if(config->gyro_calib_samples == 0U) {
        attitude->calibrated = true;
    }

    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_update(ImuAttitude* attitude, const ImuSample* sample) {
    float dt = 0.0f;

    if(attitude == 0 || sample == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if((sample->flags & IMU_SAMPLE_GYRO_NEW) == 0U ||
        (sample->flags & IMU_SAMPLE_GYRO_VALID) == 0U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    if(!attitude->calibrated) {
        return imu_attitude_calibrate_gyro(attitude, sample);
    }

    if(attitude->last_update_us == 0U) {
        attitude->last_update_us = sample->gyro_timestamp_us;

        if(!attitude->has_angle && (sample->flags & IMU_SAMPLE_ACC_VALID) != 0U) {
            imu_attitude_update_angle_by_acc(attitude, &sample->acc, true);
        }

        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    dt = (float)(sample->gyro_timestamp_us - attitude->last_update_us) * 1.0e-6f;
    attitude->last_update_us = sample->gyro_timestamp_us;

    if(dt <= 0.0f || dt > 0.1f) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    attitude->zru_active = false;

    if(attitude->config.mode == IMU_ATTITUDE_COMPLEMENTARY) {
        imu_attitude_update_complementary(attitude, sample, dt);
    }
    else if(attitude->config.mode == IMU_ATTITUDE_MAHONY_6AXIS) {
        imu_attitude_update_mahony(attitude, sample, dt);
    }
    else {
        return IMU_ATTITUDE_STATUS_ERROR;
    }

    attitude->has_angle = true;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_angle(const ImuAttitude* attitude, ImuAngle* angle) {
    if(attitude == 0 || angle == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(!attitude->has_angle) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    *angle = attitude->angle;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_quat(const ImuAttitude* attitude, ImuQuat* quat) {
    if(attitude == 0 || quat == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(!attitude->has_angle) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    *quat = attitude->quat;
    return IMU_ATTITUDE_STATUS_OK;
}


ImuAttitudeStatus imu_attitude_get_gyro_bias(const ImuAttitude* attitude, ImuGyro* gyro_bias) {
    if(attitude == 0 || gyro_bias == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(!attitude->calibrated) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    *gyro_bias = attitude->gyro_bias;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_gyro_corrected(const ImuAttitude* attitude, ImuGyro* gyro_corrected) {
    if(attitude == 0 || gyro_corrected == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    *gyro_corrected = attitude->gyro_filtered;
    if(!attitude->calibrated) {
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }
    if(!attitude->has_angle) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_reset_yaw(ImuAttitude* attitude, float yaw) {
    if(attitude == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(!attitude->has_angle) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    attitude->angle.yaw = imu_attitude_wrap_pi(yaw);
    imu_attitude_quat_from_angle(&attitude->quat, &attitude->angle);
    return IMU_ATTITUDE_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static float imu_attitude_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, IMU_ATTITUDE_2PI);
    if(angle >= IMU_ATTITUDE_PI) {
        angle -= IMU_ATTITUDE_2PI;
    }
    else if(angle < -IMU_ATTITUDE_PI) {
        angle += IMU_ATTITUDE_2PI;
    }

    return angle;
}

static float imu_attitude_acc_norm(const ImuAcc* acc) {
    if(acc == 0) {
        return 0.0f;
    }

    return sqrtf(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
}

static bool imu_attitude_acc_is_trusted(const ImuAttitude* attitude, const ImuAcc* acc) {
    float acc_norm = 0.0f;

    if(attitude == 0 || acc == 0) {
        return false;
    }

    if(attitude->config.acc_norm <= 0.0f || attitude->config.acc_norm_tolerance < 0.0f) {
        return true;
    }

    acc_norm = imu_attitude_acc_norm(acc);
    return fabsf(acc_norm - attitude->config.acc_norm) <= attitude->config.acc_norm_tolerance;
}

static bool imu_attitude_acc_can_fuse(
    const ImuAttitude* attitude, const ImuSample* sample, float* acc_norm, uint32_t* acc_age_us) {
    uint32_t age_us = 0U;

    if(acc_norm != 0) {
        *acc_norm = 0.0f;
    }

    if(acc_age_us != 0) {
        *acc_age_us = 0U;
    }

    if(attitude == 0 || sample == 0) {
        return false;
    }

    if((sample->flags & IMU_SAMPLE_ACC_VALID) == 0U ||
        (sample->flags & IMU_SAMPLE_GYRO_VALID) == 0U) {
        return false;
    }

    age_us = sample->gyro_timestamp_us - sample->acc_timestamp_us;
    if(acc_age_us != 0) {
        *acc_age_us = age_us;
    }

    if(attitude->config.max_acc_age_us != 0U && age_us > attitude->config.max_acc_age_us) {
        return false;
    }

    if(acc_norm != 0) {
        *acc_norm = imu_attitude_acc_norm(&sample->acc);
    }

    return imu_attitude_acc_is_trusted(attitude, &sample->acc);
}

static void imu_attitude_quat_normalize(ImuQuat* quat) {
    float norm = 0.0f;

    if(quat == 0) {
        return;
    }

    norm = sqrtf(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    if(norm <= 0.0f) {
        quat->w = 1.0f;
        quat->x = 0.0f;
        quat->y = 0.0f;
        quat->z = 0.0f;
        return;
    }

    quat->w /= norm;
    quat->x /= norm;
    quat->y /= norm;
    quat->z /= norm;
}

static void imu_attitude_quat_from_angle(ImuQuat* quat, const ImuAngle* angle) {
    float cr = 0.0f;
    float sr = 0.0f;
    float cp = 0.0f;
    float sp = 0.0f;
    float cy = 0.0f;
    float sy = 0.0f;

    if(quat == 0 || angle == 0) {
        return;
    }

    cr = cosf(angle->roll * 0.5f);
    sr = sinf(angle->roll * 0.5f);
    cp = cosf(angle->pitch * 0.5f);
    sp = sinf(angle->pitch * 0.5f);
    cy = cosf(angle->yaw * 0.5f);
    sy = sinf(angle->yaw * 0.5f);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;
    imu_attitude_quat_normalize(quat);
}

static void imu_attitude_angle_from_quat(ImuAngle* angle, const ImuQuat* quat) {
    float sinr_cosp = 0.0f;
    float cosr_cosp = 0.0f;
    float sinp = 0.0f;
    float siny_cosp = 0.0f;
    float cosy_cosp = 0.0f;

    if(angle == 0 || quat == 0) {
        return;
    }

    sinr_cosp = 2.0f * (quat->w * quat->x + quat->y * quat->z);
    cosr_cosp = 1.0f - 2.0f * (quat->x * quat->x + quat->y * quat->y);
    angle->roll = atan2f(sinr_cosp, cosr_cosp);

    sinp = 2.0f * (quat->w * quat->y - quat->z * quat->x);
    if(sinp >= 1.0f) {
        angle->pitch = IMU_ATTITUDE_PI * 0.5f;
    }
    else if(sinp <= -1.0f) {
        angle->pitch = -IMU_ATTITUDE_PI * 0.5f;
    }
    else {
        angle->pitch = asinf(sinp);
    }

    siny_cosp = 2.0f * (quat->w * quat->z + quat->x * quat->y);
    cosy_cosp = 1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z);
    angle->yaw = atan2f(siny_cosp, cosy_cosp);

    angle->roll = imu_attitude_wrap_pi(angle->roll);
    angle->pitch = imu_attitude_wrap_pi(angle->pitch);
    angle->yaw = imu_attitude_wrap_pi(angle->yaw);
}

static void imu_attitude_update_angle_by_acc(ImuAttitude* attitude, const ImuAcc* acc, bool reset_yaw) {
    if(attitude == 0 || acc == 0) {
        return;
    }

    attitude->angle.roll = atan2f(acc->y, acc->z);
    attitude->angle.pitch = atan2f(-acc->x, sqrtf(acc->y * acc->y + acc->z * acc->z));
    if(reset_yaw) {
        attitude->angle.yaw = 0.0f;
    }
    attitude->acc_filtered = *acc;
    attitude->last_acc_norm = imu_attitude_acc_norm(acc);
    attitude->last_acc_age_us = 0U;
    attitude->acc_trusted = true;
    imu_attitude_quat_from_angle(&attitude->quat, &attitude->angle);
    attitude->has_angle = true;
}

static void imu_attitude_integrate_calibrating_yaw(ImuAttitude* attitude, const ImuSample* sample) {
    float dt = 0.0f;
    float bias_x = 0.0f;
    float bias_y = 0.0f;
    float bias_z = 0.0f;

    if(attitude == 0 || sample == 0 || !attitude->has_angle) {
        return;
    }

    if(attitude->last_update_us == 0U) {
        attitude->last_update_us = sample->gyro_timestamp_us;
        return;
    }

    dt = (float)(sample->gyro_timestamp_us - attitude->last_update_us) * 1.0e-6f;
    attitude->last_update_us = sample->gyro_timestamp_us;
    if(dt <= 0.0f || dt > 0.1f) {
        return;
    }

    if(attitude->calib_count > 0U) {
        bias_x = attitude->gyro_bias_sum.x / (float)attitude->calib_count;
        bias_y = attitude->gyro_bias_sum.y / (float)attitude->calib_count;
        bias_z = attitude->gyro_bias_sum.z / (float)attitude->calib_count;
    }

    attitude->gyro_filtered.x = sample->gyro.x - bias_x;
    attitude->gyro_filtered.y = sample->gyro.y - bias_y;
    attitude->gyro_filtered.z = sample->gyro.z - bias_z;
    attitude->angle.yaw = imu_attitude_wrap_pi(attitude->angle.yaw + attitude->gyro_filtered.z * dt);
    imu_attitude_quat_from_angle(&attitude->quat, &attitude->angle);
}

static void imu_attitude_reset_calibration(ImuAttitude* attitude) {
    if(attitude == 0) {
        return;
    }

    attitude->gyro_bias_sum = (ImuGyro){ 0.0f, 0.0f, 0.0f };
    attitude->gyro_sq_sum = (ImuGyro){ 0.0f, 0.0f, 0.0f };
    attitude->gyro_temp_ref = 0.0f;
    attitude->gyro_temp_sum = 0.0f;
    attitude->gyro_temp_count = 0U;
    attitude->gyro_temp_valid = false;
    attitude->gyro_z_temp_intercept = 0.0f;
    attitude->gyro_z_bias_effective = 0.0f;
    attitude->zru_static_time_us = 0U;
    attitude->zru_active = false;
    attitude->calib_count = 0U;
    attitude->calibrated = false;
}

static ImuAttitudeStatus imu_attitude_calibrate_gyro(ImuAttitude* attitude, const ImuSample* sample) {
    uint16_t calib_target = 0U;
    float acc_norm = 0.0f;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_z = 0.0f;
    float z_bias_offset = 0.0f;
    float var_x = 0.0f;
    float var_y = 0.0f;
    float var_z = 0.0f;

    if(attitude == 0 || sample == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(!attitude->has_angle && (sample->flags & IMU_SAMPLE_ACC_VALID) != 0U) {
        imu_attitude_update_angle_by_acc(attitude, &sample->acc, true);
    }
    imu_attitude_integrate_calibrating_yaw(attitude, sample);

    calib_target = attitude->config.gyro_calib_samples;
    if(calib_target == 0U) {
        attitude->calibrated = true;

        if((sample->flags & IMU_SAMPLE_ACC_VALID) != 0U) {
            imu_attitude_update_angle_by_acc(attitude, &sample->acc, !attitude->has_angle);
        }

        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    if((sample->flags & IMU_SAMPLE_ACC_VALID) != 0U &&
        attitude->config.acc_norm > 0.0f &&
        attitude->config.acc_norm_tolerance >= 0.0f) {
        acc_norm = imu_attitude_acc_norm(&sample->acc);
        if(fabsf(acc_norm - attitude->config.acc_norm) > attitude->config.acc_norm_tolerance) {
            imu_attitude_reset_calibration(attitude);
            return IMU_ATTITUDE_STATUS_CALIBRATING;
        }

        imu_attitude_update_angle_by_acc(attitude, &sample->acc, false);
    }

    if((sample->flags & IMU_SAMPLE_ACC_VALID) != 0U &&
        attitude->config.max_acc_age_us != 0U &&
        (sample->gyro_timestamp_us - sample->acc_timestamp_us) > attitude->config.max_acc_age_us) {
        imu_attitude_reset_calibration(attitude);
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    attitude->gyro_bias_sum.x += sample->gyro.x;
    attitude->gyro_bias_sum.y += sample->gyro.y;
    attitude->gyro_bias_sum.z += sample->gyro.z;
    if((sample->flags & IMU_SAMPLE_TEMP_VALID) != 0U) {
        attitude->gyro_temp_sum += sample->temperature;
        attitude->gyro_temp_count++;
    }
    attitude->gyro_sq_sum.x += sample->gyro.x * sample->gyro.x;
    attitude->gyro_sq_sum.y += sample->gyro.y * sample->gyro.y;
    attitude->gyro_sq_sum.z += sample->gyro.z * sample->gyro.z;
    attitude->calib_count++;

    mean_x = attitude->gyro_bias_sum.x / (float)attitude->calib_count;
    mean_y = attitude->gyro_bias_sum.y / (float)attitude->calib_count;
    mean_z = attitude->gyro_bias_sum.z / (float)attitude->calib_count;
    var_x = attitude->gyro_sq_sum.x / (float)attitude->calib_count - mean_x * mean_x;
    var_y = attitude->gyro_sq_sum.y / (float)attitude->calib_count - mean_y * mean_y;
    var_z = attitude->gyro_sq_sum.z / (float)attitude->calib_count - mean_z * mean_z;

    if(attitude->config.gyro_calib_var_threshold > 0.0f &&
        (var_x > attitude->config.gyro_calib_var_threshold ||
            var_y > attitude->config.gyro_calib_var_threshold ||
            var_z > attitude->config.gyro_calib_var_threshold)) {
        imu_attitude_reset_calibration(attitude);
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    if(attitude->calib_count < calib_target) {
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    attitude->gyro_bias.x = mean_x;
    attitude->gyro_bias.y = mean_y;
    attitude->gyro_bias.z = mean_z;
    attitude->gyro_z_temp_intercept = 0.0f;
    attitude->gyro_z_bias_effective = 0.0f;
    if(attitude->gyro_temp_count > 0U) {
        attitude->gyro_temp_ref = attitude->gyro_temp_sum / (float)attitude->gyro_temp_count;
        attitude->gyro_temp_valid = true;
    }

    z_bias_offset = imu_attitude_get_z_bias_offset(attitude);
    attitude->gyro_z_temp_intercept = z_bias_offset;
    attitude->gyro_z_bias_effective = z_bias_offset;

    attitude->calibrated = true;
    attitude->last_update_us = sample->gyro_timestamp_us;

    if((sample->flags & IMU_SAMPLE_ACC_VALID) != 0U) {
        imu_attitude_update_angle_by_acc(attitude, &sample->acc, !attitude->has_angle);
    }
    else {
        imu_attitude_quat_from_angle(&attitude->quat, &attitude->angle);
        attitude->has_angle = true;
    }

    return IMU_ATTITUDE_STATUS_CALIBRATING;
}

static ImuGyro imu_attitude_get_temp_comp(const ImuAttitude* attitude, const ImuSample* sample) {
    ImuGyro temp_comp = { 0.0f, 0.0f, 0.0f };

    if(attitude == 0 || sample == 0 || (sample->flags & IMU_SAMPLE_TEMP_VALID) == 0U) {
        return temp_comp;
    }

    temp_comp.x = attitude->config.gyro_x_temp_coeff * sample->temperature;
    temp_comp.y = attitude->config.gyro_y_temp_coeff * sample->temperature;
    temp_comp.z = attitude->config.gyro_z_temp_coeff * sample->temperature;
    return temp_comp;
}

static float imu_attitude_get_z_bias_offset(const ImuAttitude* attitude) {
    if(attitude == 0) {
        return 0.0f;
    }

    return attitude->config.gyro_z_bias_offset;
}

static bool imu_attitude_z_bias_model_enabled(const ImuAttitude* attitude) {
    if(attitude == 0) {
        return false;
    }

    return attitude->config.gyro_z_temp_coeff != 0.0f ||
        attitude->config.gyro_z_bias_offset != 0.0f;
}

static ImuGyro imu_attitude_get_corrected_gyro(ImuAttitude* attitude, const ImuSample* sample) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
    ImuGyro temp_comp = { 0.0f, 0.0f, 0.0f };
    float bias_z = 0.0f;

    if(attitude == 0 || sample == 0) {
        return gyro;
    }

    temp_comp = imu_attitude_get_temp_comp(attitude, sample);
    bias_z = attitude->gyro_bias.z;
    if(imu_attitude_z_bias_model_enabled(attitude)) {
        bias_z = attitude->gyro_z_temp_intercept + temp_comp.z;
    }

    gyro.x = sample->gyro.x - attitude->gyro_bias.x - temp_comp.x;
    gyro.y = sample->gyro.y - attitude->gyro_bias.y - temp_comp.y;
    gyro.z = sample->gyro.z - bias_z;
    attitude->gyro_z_bias_effective = bias_z;
    return gyro;
}

static void imu_attitude_update_complementary(ImuAttitude* attitude, const ImuSample* sample, float dt) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
    float roll_gyro = 0.0f;
    float pitch_gyro = 0.0f;
    float yaw_gyro = 0.0f;
    float acc_norm = 0.0f;
    uint32_t acc_age_us = 0U;

    if(attitude == 0 || sample == 0) {
        return;
    }

    gyro = imu_attitude_get_corrected_gyro(attitude, sample);
    attitude->acc_trusted = imu_attitude_acc_can_fuse(attitude, sample, &acc_norm, &acc_age_us);
    attitude->last_acc_norm = acc_norm;
    attitude->last_acc_age_us = acc_age_us;
    attitude->gyro_filtered = gyro;

    roll_gyro = attitude->angle.roll + gyro.x * dt;
    pitch_gyro = attitude->angle.pitch + gyro.y * dt;
    yaw_gyro = attitude->angle.yaw + gyro.z * dt;

    attitude->angle.roll = roll_gyro;
    attitude->angle.pitch = pitch_gyro;
    attitude->angle.yaw = imu_attitude_wrap_pi(yaw_gyro);

    if(attitude->acc_trusted) {
        float roll_acc = atan2f(sample->acc.y, sample->acc.z);
        float pitch_acc = atan2f(-sample->acc.x, sqrtf(sample->acc.y * sample->acc.y + sample->acc.z * sample->acc.z));
        float alpha = 0.0f;

        attitude->acc_filtered = sample->acc;
        if(attitude->config.complementary_tau > 0.0f) {
            alpha = attitude->config.complementary_tau / (attitude->config.complementary_tau + dt);
        }

        attitude->angle.roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
        attitude->angle.pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    }

    attitude->angle.roll = imu_attitude_wrap_pi(attitude->angle.roll);
    attitude->angle.pitch = imu_attitude_wrap_pi(attitude->angle.pitch);
    imu_attitude_quat_from_angle(&attitude->quat, &attitude->angle);
}

static void imu_attitude_update_mahony(ImuAttitude* attitude, const ImuSample* sample, float dt) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float acc_norm = 0.0f;
    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float qw = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    uint32_t acc_age_us = 0U;

    if(attitude == 0 || sample == 0) {
        return;
    }

    gyro = imu_attitude_get_corrected_gyro(attitude, sample);
    attitude->acc_trusted = imu_attitude_acc_can_fuse(attitude, sample, &acc_norm, &acc_age_us);
    attitude->last_acc_norm = acc_norm;
    attitude->last_acc_age_us = acc_age_us;
    gx = gyro.x;
    gy = gyro.y;
    gz = gyro.z;

    if(attitude->acc_trusted && acc_norm > 0.0f) {
        ax = sample->acc.x / acc_norm;
        ay = sample->acc.y / acc_norm;
        az = sample->acc.z / acc_norm;
        attitude->acc_filtered = sample->acc;

        vx = 2.0f * (attitude->quat.x * attitude->quat.z - attitude->quat.w * attitude->quat.y);
        vy = 2.0f * (attitude->quat.w * attitude->quat.x + attitude->quat.y * attitude->quat.z);
        vz = attitude->quat.w * attitude->quat.w - attitude->quat.x * attitude->quat.x -
            attitude->quat.y * attitude->quat.y + attitude->quat.z * attitude->quat.z;

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        attitude->gyro_bias.x += attitude->config.mahony_ki * ex * dt;
        attitude->gyro_bias.y += attitude->config.mahony_ki * ey * dt;
        if(imu_attitude_z_bias_model_enabled(attitude)) {
            attitude->gyro_z_temp_intercept += attitude->config.mahony_ki_z * ez * dt;
            attitude->gyro_z_bias_effective = attitude->gyro_z_temp_intercept;
        }
        else {
            attitude->gyro_bias.z += attitude->config.mahony_ki_z * ez * dt;
            attitude->gyro_z_bias_effective = attitude->gyro_bias.z;
        }

        gx += attitude->config.mahony_kp * ex;
        gy += attitude->config.mahony_kp * ey;
    }

    attitude->gyro_filtered.x = gx;
    attitude->gyro_filtered.y = gy;
    attitude->gyro_filtered.z = gz;

    qw = attitude->quat.w;
    qx = attitude->quat.x;
    qy = attitude->quat.y;
    qz = attitude->quat.z;

    attitude->quat.w += (-qx * gx - qy * gy - qz * gz) * 0.5f * dt;
    attitude->quat.x += (qw * gx + qy * gz - qz * gy) * 0.5f * dt;
    attitude->quat.y += (qw * gy - qx * gz + qz * gx) * 0.5f * dt;
    attitude->quat.z += (qw * gz + qx * gy - qy * gx) * 0.5f * dt;

    imu_attitude_quat_normalize(&attitude->quat);
    imu_attitude_angle_from_quat(&attitude->angle, &attitude->quat);
}
