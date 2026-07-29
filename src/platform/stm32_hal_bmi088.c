#include "stm32_hal_bmi088.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "delay.h"
#include "stm32_hal_dwt.h"
#include "stm32_hal_exti.h"
#include "stm32_hal_spi.h"

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void bmi088_accel_cs_low(void);
static void bmi088_accel_cs_high(void);
static void bmi088_gyro_cs_low(void);
static void bmi088_gyro_cs_high(void);
static uint8_t bmi088_read_write_byte(uint8_t tx_data);
static bool bmi088_transmit_receive_dma(uint8_t* tx_data, uint8_t* rx_data, uint16_t len);
static void* bmi088_get_spi_handle(void);
static void bmi088_delay_us(uint16_t us);
static void bmi088_cache_clean(const void* addr, uint32_t len);
static void bmi088_cache_invalidate(const void* addr, uint32_t len);
static void stm32_bmi088_exti_callback(uint16_t gpio_pin, void* user);

// ! ========================= 变 量 声 明 ========================= ! //

static const Bmi088PortOps bmi088_ops = {
    .accel_cs_low = bmi088_accel_cs_low,
    .accel_cs_high = bmi088_accel_cs_high,
    .gyro_cs_low = bmi088_gyro_cs_low,
    .gyro_cs_high = bmi088_gyro_cs_high,
    .read_write_byte = bmi088_read_write_byte,
    .transmit_receive_dma = bmi088_transmit_receive_dma,
    .get_spi_handle = bmi088_get_spi_handle,
    .now_ms = HAL_GetTick,
    .now_us = dwt_get_us,
    .delay_ms = delay_ms,
    .delay_us = bmi088_delay_us,
    .cache_clean = bmi088_cache_clean,
    .cache_invalidate = bmi088_cache_invalidate,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const Bmi088PortOps* stm32_bmi088_get_ops(void) {
    return &bmi088_ops;
}

uint16_t stm32_bmi088_get_accel_int_pin(void) {
    return ACC_INT_Pin;
}

uint16_t stm32_bmi088_get_gyro_int_pin(void) {
    return GYRO_INT_Pin;
}

void stm32_bmi088_register_callbacks(void) {
    (void)exti_register_callback(ACC_INT_Pin, stm32_bmi088_exti_callback, NULL);
    (void)exti_register_callback(GYRO_INT_Pin, stm32_bmi088_exti_callback, NULL);
    spi_register_txrx_complete_callback(&hspi2, stm32_bmi088_spi_txrx_complete_callback);
    spi_register_error_callback(&hspi2, stm32_bmi088_spi_error_callback);
}

void stm32_bmi088_spi_txrx_complete_callback(SPI_HandleTypeDef* hspi) {
    bmi088_spi_txrx_cplt_callback(hspi);
}

void stm32_bmi088_spi_error_callback(SPI_HandleTypeDef* hspi) {
    bmi088_spi_error_callback(hspi);
}

static void stm32_bmi088_exti_callback(uint16_t gpio_pin, void* user) {
    (void)user;
    bmi088_exti_callback(gpio_pin);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void bmi088_accel_cs_low(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

static void bmi088_accel_cs_high(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

static void bmi088_gyro_cs_low(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

static void bmi088_gyro_cs_high(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

static uint8_t bmi088_read_write_byte(uint8_t tx_data) {
    uint8_t rx_data = 0U;

    (void)HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1U, 1000U);
    return rx_data;
}

static bool bmi088_transmit_receive_dma(uint8_t* tx_data, uint8_t* rx_data, uint16_t len) {
    return HAL_SPI_TransmitReceive_DMA(&hspi2, tx_data, rx_data, len) == HAL_OK;
}

static void* bmi088_get_spi_handle(void) {
    return &hspi2;
}

static void bmi088_delay_us(uint16_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t told = SysTick->VAL;
    uint32_t tcnt = 0U;
    uint32_t reload = SysTick->LOAD;

    while(1) {
        uint32_t tnow = SysTick->VAL;
        if(tnow != told) {
            if(tnow < told) {
                tcnt += told - tnow;
            }
            else {
                tcnt += reload - tnow + told;
            }

            told = tnow;
            if(tcnt >= ticks) {
                break;
            }
        }
    }
}

/**
 * @brief 在 DMA 启动前清理发送缓冲对应的 cache 行
 * @param addr 缓冲区起始地址
 * @param len 缓冲区长度，单位字节
 */
static void bmi088_cache_clean(const void* addr, uint32_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    int32_t size = (int32_t)((((uintptr_t)addr + len + 31U) & ~(uintptr_t)31U) - start);

    SCB_CleanDCache_by_Addr((uint32_t*)start, size);
#else
    (void)addr;
    (void)len;
#endif
}

/**
 * @brief 在 DMA 结束后失效接收缓冲对应的 cache 行
 * @param addr 缓冲区起始地址
 * @param len 缓冲区长度，单位字节
 */
static void bmi088_cache_invalidate(const void* addr, uint32_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31U;
    int32_t size = (int32_t)((((uintptr_t)addr + len + 31U) & ~(uintptr_t)31U) - start);

    SCB_InvalidateDCache_by_Addr((uint32_t*)start, size);
#else
    (void)addr;
    (void)len;
#endif
}
