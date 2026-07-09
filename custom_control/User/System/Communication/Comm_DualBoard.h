//
// Created by CaoKangqi on 2026/6/22.
//

#ifndef H7_FRAMEWORK_COMM_DUALBOARD_H
#define H7_FRAMEWORK_COMM_DUALBOARD_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LINK_CAN,
    LINK_UART
} Comm_Link_e;

#define DUALBOARD_MAX_PAYLOAD 256

// 双板通讯发送结构体
typedef struct __attribute__((packed)) {
    int16_t ch0:11;
    int16_t ch1:11;
    int16_t ch2:11;
    int16_t ch3:11;
    uint8_t s1:2;
    uint8_t s2:2;
} B2B_Tx_t;

// 双板通讯接收结构体
typedef struct __attribute__((packed)) {
    int16_t ch0:11;
    int16_t ch1:11;
    int16_t ch2:11;
    int16_t ch3:11;
    uint8_t s1:2;
    uint8_t s2:2;
} B2B_Rx_t;

extern B2B_Tx_t Tx_Data;
extern B2B_Rx_t Rx_Data;

void DualBoard_Comm_Init(void);

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len);

void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *rx_buf, uint16_t len);

void DualBoard_Task_Poll(void);

#endif //H7_FRAMEWORK_COMM_DUALBOARD_H