/**
 * @file    delay.c
 * @brief   延时服务实现
 */
#include "delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 延时服务操作函数指针结构体
 * @param get_ms 获取当前毫秒数的函数指针
 * @param get_us 获取当前微秒数的函数指针
 */
struct {
    ms_t(*get_ms)(void);
    us_t(*get_us)(void);
} delay_ops = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool ms_timeout(ms_t start, ms_t timeout_ms);
static bool us_timeout(us_t start, us_t timeout_us);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 毫秒延时服务初始化
 * @param get_ms 获取当前毫秒数的函数指针
 * @param ms_timeout 判断毫秒级是否超时的函数指针, 超时返回 true, 未超时返回 false
 * @note 用法: 在系统初始化时调用, 传入对应的函数指针, 即可使用延时服务. 例如:
 * @note delay_ms_init(systick_get_ms, systick_is_timeout);
 * @note 其中: ms_t = us_t = uint32_t
 */
void delay_ms_init(ms_t(*get_ms)(void)) {
    delay_ops.get_ms = get_ms;
}

/**
 * @brief 获取当前毫秒数
 * @return 当前毫秒数, 如果未初始化则返回 0
 */
ms_t delay_now_ms(void) {
    if(delay_ops.get_ms == 0) return 0;
    return delay_ops.get_ms();
}

/**
 * @brief 毫秒级阻塞延时
 * @param ms 延时毫秒数
 * @return None
 */
void delay_ms(ms_t ms) {
    if(delay_ops.get_ms == 0) return;
    ms_t start = delay_ops.get_ms();
    while(!ms_timeout(start, ms));
}

/**
 * @brief 秒级阻塞延时
 * @param s 延时秒数
 * @return None
 */
void delay_s(ms_t s) {
    if(delay_ops.get_ms == 0) return;
    ms_t start = delay_ops.get_ms();
    while(!ms_timeout(start, s * 1000));
}

/**
 * @brief 非阻塞毫秒延时
 * @param start 指向起始时间的指针
 * @param interval_ms 延时间隔(ms)
 * @return bool -  true:时间到, false:未到
 */
bool delay_nb_ms(ms_t* start, ms_t interval_ms) {
    if(start == 0 || delay_ops.get_ms == 0) return false;

    ms_t now = delay_ops.get_ms();
    if(*start == 0) {
        *start = (now == 0) ? 1 : now;
        return false;
    }

    if(ms_timeout(*start, interval_ms)) {
        *start = 0;
        return true;
    }
    return false;
}

/**
 * @brief 微秒延时服务初始化
 * @param get_us 获取当前微秒数的函数指针
 */
void delay_us_init(us_t(*get_us)(void)) {
    delay_ops.get_us = get_us;
}

/**
 * @brief 获取当前微秒数
 * @return 当前微秒数, 如果未初始化则返回 0
 */
us_t delay_now_us(void) {
    if(delay_ops.get_us == 0) return 0;
    return delay_ops.get_us();
}

/**
 * @brief 微秒级阻塞延时
 * @param us 延时微秒数
 * @return None
 */
void delay_us(us_t us) {
    if(delay_ops.get_us == 0) return;
    us_t start = delay_ops.get_us();
    while(!us_timeout(start, us));
}

/**
 * @brief 非阻塞微秒延时
 * @param start 指向起始时间的指针
 * @param interval_us 延时间隔(us)
 * @return bool -  true:时间到, false:未到
 */
bool delay_nb_us(us_t* start, us_t interval_us) {
    if(start == 0 || delay_ops.get_us == 0) return false;

    us_t now = delay_ops.get_us();
    if(*start == 0) {
        *start = (now == 0) ? 1 : now;
        return false;
    }

    if(us_timeout(*start, interval_us)) {
        *start = 0;
        return true;
    }
    return false;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 判断是否达到指定的毫秒级超时时间
 * @param start 起始时间（毫秒）
 * @param timeout_ms 超时时间（毫秒）
 * @return true 表示已超时，false 表示未超时
 */
static bool ms_timeout(ms_t start, ms_t timeout_ms) {
    return (ms_t)(delay_ops.get_ms() - start) >= timeout_ms;
}

/**
 * @brief 判断是否达到指定的微秒级超时时间
 * @param start 起始时间（微秒）
 * @param timeout_us 超时时间（微秒）
 * @return true 表示已超时，false 表示未超时
 */
static bool us_timeout(us_t start, us_t timeout_us) {
    return (us_t)(delay_ops.get_us() - start) >= timeout_us;
}
