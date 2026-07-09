//
// Created by CaoKangqi on 2026/6/22.
//
#include "Comm_DualBoard.h"
#include "Message_Center.h"
#include "BSP_FDCAN.h"
#include "BSP_DWT.h"
#include <string.h>

B2B_Tx_t Tx_Data = {0};
B2B_Rx_t Rx_Data = {0};

// CAN 传输层切片状态机
typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t total_len;
    uint16_t offset;
    uint8_t  seq;
    uint32_t last_tick;
    bool     is_sending;
} CAN_TP_Tx_State_t;

typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t current_len;
    uint8_t  next_seq;
    bool     is_active;
} CAN_TP_Rx_t;

static CAN_TP_Tx_State_t can_tx = {0};
static CAN_TP_Rx_t       can_rx = {0};

// UART 暂存缓冲区
static uint8_t  uart_rx_buf[DUALBOARD_MAX_PAYLOAD];
static uint16_t uart_rx_len = 0;

void DualBoard_Comm_Init(void)
{
    memset(&Tx_Data, 0, sizeof(B2B_Tx_t));
    memset(&Rx_Data, 0, sizeof(B2B_Rx_t));
}

/**
 * @brief 裸数据发送接口
 */
uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len)
{
    if (data_ptr == NULL || len > DUALBOARD_MAX_PAYLOAD) return 1;

    if (link == LINK_UART) {
        // 串口：直接把裸结构体扔给 DMA 发送
        // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data_ptr, len);
        return 0;
    }
    if (link == LINK_CAN) {
        if (can_tx.is_sending) return 2;
        can_tx.total_len = len;
        memcpy(can_tx.buf, data_ptr, len);
        can_tx.offset = 0;
        can_tx.seq = 0;
        can_tx.is_sending = true;
        return 0;
    }
    return 1;
}

/**
 * @brief 负责 CAN 硬件单帧 (8字节) 的非阻塞切片续发
 */
void DualBoard_Task_Poll(void)
{
    if (!can_tx.is_sending) return;

    uint32_t now = DWT->CYCCNT;
    uint32_t interval_ticks = 100 * 550; // 帧间隔延迟
    if (can_tx.seq > 0 && (now - can_tx.last_tick) < interval_ticks) return;

    uint8_t tx_buf[8];
    uint16_t remain = can_tx.total_len - can_tx.offset;
    uint8_t chunk_size = (remain > 7) ? 7 : (uint8_t)remain;
    uint8_t is_last = (remain <= 7) ? 1 : 0;

    // CAN 传输层控制头：最高位代表是否为最后一帧，低7位为序列号
    tx_buf[0] = (is_last << 7) | (can_tx.seq & 0x7F);
    memcpy(&tx_buf[1], &can_tx.buf[can_tx.offset], chunk_size);
    if (chunk_size < 7) memset(&tx_buf[1 + chunk_size], 0, 7 - chunk_size);

    if (FDCAN_Send_Msg(&hfdcan1, 0x500, tx_buf, 8) == 0) {
        can_tx.offset += chunk_size;
        can_tx.seq++;
        can_tx.last_tick = now;
        if (is_last) can_tx.is_sending = false;
    }
}

/**
 * @brief CAN 切片重组回调
 */
void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data)
{
    if (data == NULL) return;

    uint8_t dlc_len = 8;
    uint8_t ctrl = data[0];
    uint8_t is_last = (ctrl >> 7) & 0x01;
    uint8_t seq = ctrl & 0x7F;
    uint8_t payload_len = dlc_len - 1;

    if (seq == 0) {
        can_rx.is_active = true;
        can_rx.current_len = 0;
        can_rx.next_seq = 0;
    }

    if (can_rx.is_active && seq == can_rx.next_seq) {
        if (can_rx.current_len + payload_len <= sizeof(can_rx.buf)) {
            memcpy(&can_rx.buf[can_rx.current_len], &data[1], payload_len);
            can_rx.current_len += payload_len;
            can_rx.next_seq++;

            if (is_last) {
                if (can_rx.current_len >= sizeof(B2B_Rx_t)) {
                    memcpy(&Rx_Data, can_rx.buf, sizeof(B2B_Rx_t));
                }
                can_rx.is_active = false;
            }
        } else {
            can_rx.is_active = false;
        }
    }
}

/**
 * @brief UART 裸流接收回调
 */
void DualBoard_UART_Rx(uint8_t *Buff, uint16_t Size)
{
    if (Buff == NULL || Size == 0) return;

    if (Size == sizeof(B2B_Rx_t)) {
        memcpy(&Rx_Data, Buff, sizeof(B2B_Rx_t));
        return;
    }
    for (uint16_t i = 0; i < Size; i++) {
        uart_rx_buf[uart_rx_len++] = Buff[i];

        if (uart_rx_len >= sizeof(B2B_Rx_t)) {
            memcpy(&Rx_Data, uart_rx_buf, sizeof(B2B_Rx_t));
            uart_rx_len = 0;
        }
    }
}