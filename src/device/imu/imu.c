#include "imu.h"

// ! ========================= 变 量 声 明 ========================= ! //

const ImuInterface* imu_instance = 0;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ImuStatus imu_set_instance(const ImuInterface* instance) {
    if(instance == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    imu_instance = instance;
    return IMU_STATUS_OK;
}

ImuStatus imu_init(const void* config) {
    if(imu_instance == 0 || imu_instance->init == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    return imu_instance->init(config);
}

ImuStatus imu_update(void) {
    if(imu_instance == 0 || imu_instance->update == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    return imu_instance->update();
}

ImuAcc imu_get_acc(void) {
    ImuAcc acc = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_acc == 0) {
        return acc;
    }

    return imu_instance->get_acc();
}

ImuGyro imu_get_gyro(void) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_gyro == 0) {
        return gyro;
    }

    return imu_instance->get_gyro();
}


ImuGyro imu_get_gyro_bias(void) {
    ImuGyro gyro_bias = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_gyro_bias == 0) {
        return gyro_bias;
    }

    return imu_instance->get_gyro_bias();
}

ImuGyro imu_get_gyro_corrected(void) {
    if(imu_instance == 0) {
        ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
        return gyro;
    }

    if(imu_instance->get_gyro_corrected != 0) {
        return imu_instance->get_gyro_corrected();
    }

    if(imu_instance->get_gyro != 0) {
        return imu_instance->get_gyro();
    }

    {
        ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
        return gyro;
    }
}

ImuAngle imu_get_angle(void) {
    ImuAngle angle = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_angle == 0) {
        return angle;
    }

    return imu_instance->get_angle();
}

ImuStatus imu_get_sample(ImuSample* sample) {
    if(sample == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    if(imu_instance == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    if(imu_instance->get_sample != 0) {
        return imu_instance->get_sample(sample);
    }

    if(imu_instance->get_acc == 0 || imu_instance->get_gyro == 0) {
        return IMU_STATUS_UNSUPPORTED;
    }

    sample->acc = imu_instance->get_acc();
    sample->gyro = imu_instance->get_gyro();
    sample->temperature = 0.0f;

    sample->acc_timestamp_us = 0U;
    sample->gyro_timestamp_us = 0U;
    sample->temp_timestamp_us = 0U;
    sample->flags = IMU_SAMPLE_ACC_NEW | IMU_SAMPLE_GYRO_NEW |
        IMU_SAMPLE_ACC_VALID | IMU_SAMPLE_GYRO_VALID;
    return IMU_STATUS_OK;
}

const char* imu_status_str(ImuStatus status) {
    if(imu_instance != 0 && imu_instance->status_str != 0) {
        return imu_instance->status_str(status);
    }

    switch(status) {
#define X(name, value) case IMU_STATUS_##name: return #name;
        IMU_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}
