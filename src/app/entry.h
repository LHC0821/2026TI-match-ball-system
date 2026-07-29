#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX main.c 的最小应用入口
 */

#include "app_runtime.h"
#include "assemble/assemble.h"
#include "delay.h"
#include "log.h"

#include <stdbool.h>

// ! ========================= 变 量 声 明 ========================= ! //

static bool init_ok = false;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief entry_init 内部辅助函数
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;
    delay_ms(100);

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    delay_ms(100);

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb ready");
    delay_ms(100);

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu ready");
    delay_ms(100);

    if(assemble_vision() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT vision ready");
    delay_ms(100);

    if(assemble_motor() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT motor ready");
    delay_ms(100);

    if(assemble_balance() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT balance ready");
    delay_ms(100);

    app_runtime_init();
    log_info("BOOT runtime ready");
    delay_ms(100);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6_500hz ready");
    delay_ms(100);

    init_ok = true;
    log_info("System initialized successfully");
    delay_ms(100);
}

/**
 * @brief entry_loop 内部辅助函数
 */
static inline void entry_loop(void) {
    if(!init_ok) {
        return;
    }

    app_runtime_background();

    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;
        app_runtime_process();
    }
}

#endif
