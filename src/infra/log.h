#ifndef _log_h_
#define _log_h_

/**
 * @file log.h
 * @brief 轻量级 printf 风格日志模块
 *
 * 本模块只依赖一个底层写接口 LogPortOps.write，可对接阻塞 UART、UART DMA、RTT、USB CDC、文件或 mock buffer
 * 日志层负责格式化、级别过滤、可选 ANSI 颜色，以及异步输出时的缓冲区生命周期管理
 *
 * 最小同步用法
 *
 * @code
 * static bool board_log_write(const char* data, uint32_t len) {
 *     return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
 * }
 *
 * static const LogPortOps log_ops = {
 *     .write = board_log_write,
 * };
 *
 * void app_init(void) {
 *     LogConfig config = {
 *         .ops = &log_ops,
 *         .level = LOG_LEVEL_INFO,
 *         .enable_color = true,
 *         .async_write = false,
 *     };
 *
 *     log_init(&config);
 *     log_info("system started");
 * }
 * @endcode
 *
 * UART DMA 异步用法
 *
 * @code
 * static bool board_log_write(const char* data, uint32_t len) {
 *     return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) == HAL_OK;
 * }
 *
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 *     if(huart == &huart1) {
 *         log_write_complete();
 *     }
 * }
 *
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
 *     if(huart == &huart1) {
 *         log_write_complete();
 *     }
 * }
 * @endcode
 *
 * 注意
 * - async_write=false 时 write() 必须在返回前完成对 data 的读取
 * - async_write=true 时 write() 可在返回后继续读取 data，发送完成或错误后必须调用 log_write_complete()
 * - 异步模式内部带有小队列，连续多条日志会排队发送，队列满时返回 LOG_STATUS_BUSY
 * - LOG_BUFFER_SIZE 控制单条日志最大长度，LOG_QUEUE_DEPTH 控制异步队列深度，二者可在编译选项中覆盖
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 如果编译器支持 C11 标准，则此处定义为 true 以启用 log_vofa(...) 宏，否则改为 false
 */
#ifndef LOG_USE_C11
#define LOG_USE_C11 true
#endif

/**
 * @brief 日志输出缓冲区大小，单位 byte
 */
#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE 256u
#endif

/**
 * @brief 日志异步输出队列深度
 */
#ifndef LOG_QUEUE_DEPTH
#define LOG_QUEUE_DEPTH 16u
#endif

/**
 * @brief log_vofa() 单次最多支持的变量数量
 */
#ifndef LOG_VOFA_MAX_ARGS
#define LOG_VOFA_MAX_ARGS 16u
#endif

#if defined(__GNUC__)
/**
 * @brief 为日志格式化函数启用 printf 风格编译期检查
 */
#define LOG_PRINTF_FORMAT(format_index, first_arg) __attribute__((format(printf, format_index, first_arg)))
#else
#define LOG_PRINTF_FORMAT(format_index, first_arg)
#endif

/**
 * @brief 日志模块状态码
 */
typedef enum {
    /** 操作成功 */
    LOG_STATUS_OK = 0,
    /** 参数为空或参数值不合法 */
    LOG_STATUS_INVALID_PARAM,
    /** 底层输出端口返回错误 */
    LOG_STATUS_PORT_ERROR,
    /** 尚未调用 log_init() 或 PortOps 未绑定 */
    LOG_STATUS_NOT_INITIALIZED,
    /** 异步输出队列已满 */
    LOG_STATUS_BUSY,
} LogStatus;

/**
 * @brief 日志输出级别
 */
typedef enum {
    /** 关闭所有日志输出 */
    LOG_LEVEL_NONE = 0,
    /** 只输出 error */
    LOG_LEVEL_ERROR = 1,
    /** 输出 warn 和 error */
    LOG_LEVEL_WARN = 2,
    /** 输出 info、warn 和 error */
    LOG_LEVEL_INFO = 3,
} LogLevel;

/**
 * @brief VOFA 变量值类型
 */
typedef enum {
    LOG_VOFA_VALUE_I64 = 0,
    LOG_VOFA_VALUE_U64,
    LOG_VOFA_VALUE_F64,
    LOG_VOFA_VALUE_BOOL,
    LOG_VOFA_VALUE_CSTR,
    LOG_VOFA_VALUE_PTR,
} LogVofaValueType;

/**
 * @brief VOFA 变量值
 */
typedef struct {
    LogVofaValueType type;
    union {
        long long i64;
        unsigned long long u64;
        double f64;
        bool bool_value;
        const char* cstr;
        const void* ptr;
    } data;
} LogVofaValue;

/**
 * @brief 日志底层输出端口函数表
 */
typedef struct {
    /**
     * @brief 写出一段日志文本
     * @param data 文本缓冲区指针
     * @param len 文本长度，单位 byte
     * @return true 表示成功，false 表示输出失败
     */
    bool (*write)(const char* data, uint32_t len);
} LogPortOps;

/**
 * @brief 日志初始化配置
 */
typedef struct {
    /** 底层输出端口函数表，不能为空 */
    const LogPortOps* ops;
    /** 初始日志级别 */
    LogLevel level;
    /** 是否输出 ANSI 颜色转义序列 */
    bool enable_color;
    /**
     * @brief true 表示 write() 返回后底层仍可能继续读取 data
     *
     * UART DMA 这类异步端口应设为 true，并在发送完成中断中调用 log_write_complete()
     * 阻塞 UART、RTT、mock buffer 这类同步端口保持 false 即可
     */
    bool async_write;
} LogConfig;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化日志模块并绑定输出端口
 * @param config 日志配置，必须提供有效的 LogPortOps.write
 * @return LogStatus 状态码
 */
LogStatus log_init(const LogConfig* config);

/**
 * @brief 修改当前日志输出级别
 * @param level 新日志级别
 * @return LogStatus 状态码
 */
LogStatus log_set_level(LogLevel level);

/**
 * @brief 通知日志模块上一段异步输出已经完成
 *
 * 仅当 LogConfig.async_write 为 true 时需要调用，通常放在 UART DMA Tx complete 回调里
 */
void log_write_complete(void);

/**
 * @brief 输出 info 级别日志
 * @param format printf 风格格式字符串
 * @return LogStatus 状态码
 */
LogStatus log_info(const char* format, ...) LOG_PRINTF_FORMAT(1, 2);

/**
 * @brief 输出 warn 级别日志
 * @param format printf 风格格式字符串
 * @return LogStatus 状态码
 */
LogStatus log_warn(const char* format, ...) LOG_PRINTF_FORMAT(1, 2);

/**
 * @brief 输出 error 级别日志
 * @param format printf 风格格式字符串
 * @return LogStatus 状态码
 */
LogStatus log_error(const char* format, ...) LOG_PRINTF_FORMAT(1, 2);

#if LOG_USE_C11

/**
 * @brief 构造 VOFA 变量值
 *
 * 这些函数主要供 log_vofa(...) 宏内部调用，业务代码通常不需要直接调用
 */
LogVofaValue log_vofa_value_i64(long long value);
LogVofaValue log_vofa_value_u64(unsigned long long value);
LogVofaValue log_vofa_value_f64(double value);
LogVofaValue log_vofa_value_bool(bool value);
LogVofaValue log_vofa_value_cstr(const char* value);
LogVofaValue log_vofa_value_ptr(const void* value);

/**
 * @brief VOFA+ 自动变量名输出接口
 * @param names 变量名字符串，通常由 log_vofa(...) 宏通过 #__VA_ARGS__ 自动生成
 * @param count 变量数量
 * @param values 变量值数组
 * @return LogStatus 状态码
 */
LogStatus log_vofa_write(const char* names, uint32_t count, const LogVofaValue* values);

/**
 * @brief 将日志状态码转换为静态字符串
 * @param status 日志状态码
 * @return const char* 状态码名称
 */
const char* log_status_str(LogStatus status);

#undef LOG_PRINTF_FORMAT

#define LOG_VOFA_CAT_IMPL(a, b) a##b
#define LOG_VOFA_CAT(a, b) LOG_VOFA_CAT_IMPL(a, b)

#define LOG_VOFA_ARG_COUNT_IMPL(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, N, ...) N
#define LOG_VOFA_ARG_COUNT(...) \
    LOG_VOFA_ARG_COUNT_IMPL(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define LOG_VOFA_VALUE(value)                   \
    _Generic((value),                           \
        bool: log_vofa_value_bool,              \
        char: log_vofa_value_i64,               \
        signed char: log_vofa_value_i64,        \
        unsigned char: log_vofa_value_u64,      \
        short: log_vofa_value_i64,              \
        unsigned short: log_vofa_value_u64,     \
        int: log_vofa_value_i64,                \
        unsigned int: log_vofa_value_u64,       \
        long: log_vofa_value_i64,               \
        unsigned long: log_vofa_value_u64,      \
        long long: log_vofa_value_i64,          \
        unsigned long long: log_vofa_value_u64, \
        float: log_vofa_value_f64,              \
        double: log_vofa_value_f64,             \
        long double: log_vofa_value_f64,        \
        char*: log_vofa_value_cstr,             \
        const char*: log_vofa_value_cstr,       \
        void*: log_vofa_value_ptr,              \
        const void*: log_vofa_value_ptr,        \
        default: log_vofa_value_i64)(value)
#else
#error "log_vofa(...) requires C11 _Generic support. Please compile with -std=c11 or newer."
#endif

#define LOG_VOFA_VALUES_1(a) LOG_VOFA_VALUE(a)
#define LOG_VOFA_VALUES_2(a, b) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b)
#define LOG_VOFA_VALUES_3(a, b, c) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c)
#define LOG_VOFA_VALUES_4(a, b, c, d) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d)
#define LOG_VOFA_VALUES_5(a, b, c, d, e) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e)
#define LOG_VOFA_VALUES_6(a, b, c, d, e, f) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f)
#define LOG_VOFA_VALUES_7(a, b, c, d, e, f, g) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g)
#define LOG_VOFA_VALUES_8(a, b, c, d, e, f, g, h) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h)
#define LOG_VOFA_VALUES_9(a, b, c, d, e, f, g, h, i) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i)
#define LOG_VOFA_VALUES_10(a, b, c, d, e, f, g, h, i, j) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j)
#define LOG_VOFA_VALUES_11(a, b, c, d, e, f, g, h, i, j, k) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k)
#define LOG_VOFA_VALUES_12(a, b, c, d, e, f, g, h, i, j, k, l) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k), LOG_VOFA_VALUE(l)
#define LOG_VOFA_VALUES_13(a, b, c, d, e, f, g, h, i, j, k, l, m) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k), LOG_VOFA_VALUE(l), LOG_VOFA_VALUE(m)
#define LOG_VOFA_VALUES_14(a, b, c, d, e, f, g, h, i, j, k, l, m, n) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k), LOG_VOFA_VALUE(l), LOG_VOFA_VALUE(m), LOG_VOFA_VALUE(n)
#define LOG_VOFA_VALUES_15(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k), LOG_VOFA_VALUE(l), LOG_VOFA_VALUE(m), LOG_VOFA_VALUE(n), LOG_VOFA_VALUE(o)
#define LOG_VOFA_VALUES_16(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p) LOG_VOFA_VALUE(a), LOG_VOFA_VALUE(b), LOG_VOFA_VALUE(c), LOG_VOFA_VALUE(d), LOG_VOFA_VALUE(e), LOG_VOFA_VALUE(f), LOG_VOFA_VALUE(g), LOG_VOFA_VALUE(h), LOG_VOFA_VALUE(i), LOG_VOFA_VALUE(j), LOG_VOFA_VALUE(k), LOG_VOFA_VALUE(l), LOG_VOFA_VALUE(m), LOG_VOFA_VALUE(n), LOG_VOFA_VALUE(o), LOG_VOFA_VALUE(p)

#define LOG_VOFA_VALUES(...) LOG_VOFA_CAT(LOG_VOFA_VALUES_, LOG_VOFA_ARG_COUNT(__VA_ARGS__))(__VA_ARGS__)

/**
 * @brief 自动输出 VOFA 格式数据
 * @param ... 变量列表，如 log_vofa(pos, vel, tor) 打印出 "pos, vel, tor: 1.23, 4.56, 0.78"
 * @note 至少传入 1 个参数，最多默认支持 16 个参数
 * @note 变量名通过 #__VA_ARGS__ 获取，因此请传入简单变量名或简单表达式
 */
#define log_vofa(...)                                         \
    log_vofa_write(#__VA_ARGS__,                              \
                   (uint32_t)LOG_VOFA_ARG_COUNT(__VA_ARGS__), \
                   (const LogVofaValue[]){ LOG_VOFA_VALUES(__VA_ARGS__) })

#endif

#endif
