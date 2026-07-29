#include "stm32_hal_spi.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief SPI6 发送完成回调
 */
static void(*spi_tx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
/**
 * @brief SPI2 收发完成回调
 */
static void(*spi_txrx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
/**
 * @brief SPI2 错误回调
 */
static void(*spi_error_callback)(SPI_HandleTypeDef* hspi) = 0;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 通过 SPI6 发送一段 DMA 数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool spi_write(const uint8_t* data, uint32_t len) {
    if((data == 0) || (len == 0U) || (len > UINT16_MAX)) {
        return false;
    }

    return HAL_SPI_Transmit_DMA(&hspi6, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

/**
 * @brief 注册 SPI 发送完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_tx_complete_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi6) {
        spi_tx_complete_callback = callback;
    }
}

/**
 * @brief 注册 SPI 收发完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_txrx_complete_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_txrx_complete_callback = callback;
    }
}

/**
 * @brief 注册 SPI 错误回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_error_callback(SPI_HandleTypeDef* hspi, void(*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_error_callback = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL SPI 发送完成回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi6) {
        if(spi_tx_complete_callback != 0) {
            spi_tx_complete_callback(hspi);
        }
    }
}

/**
 * @brief HAL SPI 收发完成回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_txrx_complete_callback != 0) {
            spi_txrx_complete_callback(hspi);
        }
    }
}

/**
 * @brief HAL SPI 错误回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_error_callback != 0) {
            spi_error_callback(hspi);
        }
    }

    if(hspi == &hspi6) {
        if(spi_tx_complete_callback != 0) {
            spi_tx_complete_callback(hspi);
        }
    }
}
