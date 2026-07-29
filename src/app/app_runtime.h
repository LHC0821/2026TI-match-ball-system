#ifndef _app_runtime_h_
#define _app_runtime_h_

/**
 * @file app_runtime.h
 * @brief 滚球平衡应用超级循环接口
 */

#include "balance.h"

#include <stdbool.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief app_runtime_init 接口函数
 */
bool app_runtime_init(void);
/**
 * @brief app_runtime_process 接口函数
 */
void app_runtime_process(void);
/**
 * @brief app_runtime_background 接口函数
 */
void app_runtime_background(void);
/**
 * @brief app_runtime_get_status 接口函数
 * @return 结果指针
 */
const BalanceStatus* app_runtime_get_status(void);

#endif
