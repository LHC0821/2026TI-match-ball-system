#include "assemble.h"

#include "imu/bmi088.h"
#include "imu/imu.h"
#include "log.h"
#include "stm32_hal_bmi088.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_imu(void) {
    Bmi088Config bmi088_config;
    ImuStatus status = IMU_STATUS_OK;
    const uint16_t accel_int_pin = stm32_bmi088_get_accel_int_pin();
    const uint16_t gyro_int_pin = stm32_bmi088_get_gyro_int_pin();

    if(imu_set_instance(&bmi088_blocking_instance) != IMU_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    if(bmi088_make_config(&bmi088_config, stm32_bmi088_get_ops(), accel_int_pin, gyro_int_pin) != IMU_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    stm32_bmi088_register_callbacks();

    status = imu.init(&bmi088_config);
    if(status != IMU_STATUS_OK) {
        log_error(
            "BMI088 initialization failed: %s (%s)",
            imu.status_str(status),
            bmi088_error_str(bmi088_get_init_error()));
        return SYSTEM_STATUS_ERROR;
    }
    return SYSTEM_STATUS_OK;
}
