#include "stm32_hal_can.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define CAN_RX_CALLBACK_SLOT_NUM 2u
#define CAN_RX_MAX_FRAMES_PER_IRQ 8u

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    FDCAN_HandleTypeDef* hcan;
    STM32HalCanRxCallback callback;
    void* user;
} CanRxCallbackSlot;

// ! ========================= 变 量 声 明 ========================= ! //

static CanRxCallbackSlot s_rx_slots[CAN_RX_CALLBACK_SLOT_NUM] = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t can_len_to_dlc(uint8_t len);
static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan,
                            const FDCAN_RxHeaderTypeDef* header,
                            const uint8_t data[8]);
static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BspCanStatus can_filter_init(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hcan,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK) {
        return STM32_HAL_CAN_FILTER_CONFIG_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

BspCanStatus can_start(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigInterruptLines(hcan,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      FDCAN_INTERRUPT_LINE0) != HAL_OK) {
        return STM32_HAL_CAN_NOTIFICATION_FAILED;
    }

    if(HAL_FDCAN_ActivateNotification(hcan,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      0u) != HAL_OK) {
        return STM32_HAL_CAN_NOTIFICATION_FAILED;
    }

    can_enable_transceiver(hcan);
    if(HAL_FDCAN_Start(hcan) != HAL_OK) {
        return STM32_HAL_CAN_START_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    FDCAN_TxHeaderTypeDef tx_header = { 0 };

    if(hcan == NULL || data == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }
    if(len > 8u) {
        return STM32_HAL_CAN_INVALID_DLC;
    }

    tx_header.Identifier = id;
    tx_header.IdType = id <= 0x7FFu ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = can_len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0u;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0u) {
        uint32_t pending = hcan->Instance->TXBRP;
        if(pending != 0u) {
            (void)HAL_FDCAN_AbortTxRequest(hcan, pending);
        }
        return STM32_HAL_CAN_TX_MAILBOX_TIMEOUT;
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, (uint8_t*)data) != HAL_OK) {
        return STM32_HAL_CAN_TX_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan,
                                      STM32HalCanRxCallback callback,
                                      void* user) {
    uint8_t i;

    if(hcan == NULL || callback == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback == callback) {
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].callback == NULL) {
            s_rx_slots[i].hcan = hcan;
            s_rx_slots[i].callback = callback;
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    return STM32_HAL_CAN_NO_CALLBACK_SLOT;
}

#define X(name, str)           \
    case STM32_HAL_CAN_##name: \
        return str;
const char* can_error_code_to_str(BspCanStatus status) {
    switch(status) {
        STM32_HAL_CAN_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= HAL 回 调 实 现 ========================= ! //

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t frame_count = 0u;

    if((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u &&
          frame_count < CAN_RX_MAX_FRAMES_PER_IRQ) {
        memset(rx_data, 0, sizeof(rx_data));
        if(HAL_FDCAN_GetRxMessage(hfdcan,
                                  FDCAN_RX_FIFO0,
                                  &rx_header,
                                  rx_data) != HAL_OK) {
            return;
        }
        can_dispatch_rx(hfdcan, &rx_header, rx_data);
        frame_count++;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t can_len_to_dlc(uint8_t len) {
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };

    return dlc_table[len];
}

static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan,
                            const FDCAN_RxHeaderTypeDef* header,
                            const uint8_t data[8]) {
    uint8_t i;

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback != NULL) {
            s_rx_slots[i].callback(hcan, header, data, s_rx_slots[i].user);
        }
    }
}

static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan) {
    if(hcan != NULL && hcan->Instance == FDCAN1) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
}
