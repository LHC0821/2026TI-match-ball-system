#include "assemble.h"

#include "log.h"
#include "stm32_hal_tim.h"

// ! ========================= 变 量 声 明 ========================= ! //

volatile bool tim6_500hz_flag = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void tim6_callback(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_tim6_500hz(void) {
    tim_register_callback(&htim6, tim6_callback);
    tim_start();
    log_info("Tim6 started");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void tim6_callback(void) {
    tim6_500hz_flag = true;
}
