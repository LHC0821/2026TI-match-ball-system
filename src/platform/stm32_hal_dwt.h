#ifndef _stm32_hal_dwt_h_
#define _stm32_hal_dwt_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //



// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief dwt_init 接口函数
 */
void dwt_init(void);
/**
 * @brief dwt_get_us 接口函数
 * @return 返回值
 */
uint32_t dwt_get_us(void);

#endif
