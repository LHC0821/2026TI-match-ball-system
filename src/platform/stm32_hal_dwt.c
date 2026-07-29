#include "stm32_hal_dwt.h"

#include "main.h" // IWYU pragma: keep

// ! ========================= 变 量 声 明 ========================= ! //

#define DWT_LAR_UNLOCK_KEY 0xC5ACCE55U

static uint32_t s_cycles_per_us = 0u;
static uint32_t s_last_cyccnt = 0u;
static uint64_t s_total_cycles = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

void dwt_init(void) {
    s_cycles_per_us = SystemCoreClock / 1000000u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(DWT_LAR)
    DWT->LAR = DWT_LAR_UNLOCK_KEY;
#endif

    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_cyccnt = 0u;
    s_total_cycles = 0u;
}

uint32_t dwt_get_us(void) {
    uint32_t now_cycles;
    uint32_t delta_cycles;

    if(s_cycles_per_us == 0u) return 0u;

    now_cycles = DWT->CYCCNT;
    delta_cycles = now_cycles - s_last_cyccnt;
    s_last_cyccnt = now_cycles;
    s_total_cycles += (uint64_t)delta_cycles;

    return (uint32_t)(s_total_cycles / s_cycles_per_us);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


