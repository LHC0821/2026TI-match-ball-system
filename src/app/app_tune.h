#ifndef _app_tune_h_
#define _app_tune_h_

/**
 * @file app_tune.h
 * @brief USART1 无线调参应用接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief AppTuneState 枚举类型
 */
typedef enum {
    APP_TUNE_IDLE = 0,
    APP_TUNE_EDIT,
    APP_TUNE_APPLY_PENDING,
    APP_TUNE_APPLIED,
    APP_TUNE_REJECTED,
} AppTuneState;

/**
 * @brief AppTuneStatus 结构体类型
 */
typedef struct {
    AppTuneState state;
    uint32_t rx_line_count;
    uint32_t apply_count;
    uint32_t reject_count;
    uint32_t overflow_count;
    uint32_t last_rx_ms;
    uint16_t session_seq;
    bool dirty;
    bool apply_pending;
} AppTuneStatus;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief app_tune_init 接口函数
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool app_tune_init(void);
/**
 * @brief app_tune_process 接口函数
 */
void app_tune_process(void);
/**
 * @brief app_tune_apply_pending 接口函数
 */
void app_tune_apply_pending(void);
/**
 * @brief app_tune_get_status 接口函数
 * @return 结果指针
 */
const AppTuneStatus* app_tune_get_status(void);
/**
 * @brief app_tune_state_str 接口函数
 * @param state 状态枚举值
 * @return 字符串指针
 */
const char* app_tune_state_str(AppTuneState state);

#endif
