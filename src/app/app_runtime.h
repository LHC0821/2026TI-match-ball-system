#ifndef _app_runtime_h_
#define _app_runtime_h_

/**
 * @file app_runtime.h
 * @brief 滚球平衡应用超级循环接口
 */

#include "balance.h"

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void app_runtime_init(void);
void app_runtime_process(void);
void app_runtime_background(void);
const BalanceStatus* app_runtime_get_status(void);

#endif
