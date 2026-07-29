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
/**
 * @brief us_t 类型定义
 */
typedef uint32_t us_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief delay_ms_init 接口函数
 * @param get_ms 参数值
 */
void delay_ms_init(ms_t(*get_ms)(void));
/**
 * @brief delay_now_ms 接口函数
 * @return 返回值
 */
ms_t delay_now_ms(void);
/**
 * @brief delay_ms 接口函数
 * @param ms 时间，单位 ms
 */
void delay_ms(ms_t ms);
/**
 * @brief delay_s 接口函数
 * @param s 时间，单位 s
 */
void delay_s(ms_t s);
/**
 * @brief delay_nb_ms 接口函数
 * @param start 参数值
 * @param interval_ms 间隔时间，单位 ms
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool delay_nb_ms(ms_t* start, ms_t interval_ms);

/**
 * @brief delay_us_init 接口函数
 * @param get_us 参数值
 */
void delay_us_init(us_t(*get_us)(void));
/**
 * @brief delay_now_us 接口函数
 * @return 返回值
 */
us_t delay_now_us(void);
/**
 * @brief delay_us 接口函数
 * @param us 时间，单位 us
 */
void delay_us(us_t us);
/**
 * @brief delay_nb_us 接口函数
 * @param start 参数值
 * @param interval_us 间隔时间，单位 us
 * @return true 表示成功或条件成立，false 表示失败或条件不成立
 */
bool delay_nb_us(us_t* start, us_t interval_us);

#endif
