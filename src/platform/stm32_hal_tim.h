#ifndef _stm32_hal_tim_h_
#define _stm32_hal_tim_h_

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 滚球平衡调度所用 TIM6 句柄
 */
extern TIM_HandleTypeDef htim6;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 启动 TIM6 基本定时器中断
 */
void tim_start(void);

/**
 * @brief 停止 TIM6 基本定时器中断
 */
void tim_stop(void);

/**
 * @brief 注册 TIM 周期到期回调
 * @param htim 定时器句柄
 * @param callback 回调函数
 */
void tim_register_callback(TIM_HandleTypeDef* htim, void (*callback)(void));

#endif
