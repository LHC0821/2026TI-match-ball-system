#include "stm32_hal_tim.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief TIM6 周期到期回调函数
 */
static void (*tim_callback)(void) = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 启动 TIM6 基本定时器中断
 */
void tim_start(void) {
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 停止 TIM6 基本定时器中断
 */
void tim_stop(void) {
    HAL_TIM_Base_Stop_IT(&htim6);
}

/**
 * @brief 注册 TIM 周期到期回调
 * @param htim 定时器句柄
 * @param callback 回调函数
 */
void tim_register_callback(TIM_HandleTypeDef* htim, void (*callback)(void)) {
    (void)htim;
    tim_callback = callback;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL 定时器周期到期回调入口
 * @param htim 触发回调的定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if(htim->Instance == htim6.Instance) {
        if(tim_callback != NULL) {
            tim_callback();
        }
    }
}
