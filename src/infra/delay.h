#ifndef _delay_h_
#define _delay_h_

/**
 * @file delay.h
 * @brief 延时服务
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 毫秒和微秒时间类型定义
 */
typedef uint32_t ms_t;
typedef uint32_t us_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void delay_ms_init(ms_t(*get_ms)(void));
ms_t delay_now_ms(void);
void delay_ms(ms_t ms);
void delay_s(ms_t s);
bool delay_nb_ms(ms_t* start, ms_t interval_ms);

void delay_us_init(us_t(*get_us)(void));
us_t delay_now_us(void);
void delay_us(us_t us);
bool delay_nb_us(us_t* start, us_t interval_us);

#endif
