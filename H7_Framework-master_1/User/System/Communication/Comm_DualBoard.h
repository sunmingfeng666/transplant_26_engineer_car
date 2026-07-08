//
// Created by CaoKangqi on 2026/6/22.
//

#ifndef H7_FRAMEWORK_COMM_DUALBOARD_H
#define H7_FRAMEWORK_COMM_DUALBOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef enum {
    LINK_CAN,
    LINK_UART
} Comm_Link_e;

#define DUALBOARD_MAX_PAYLOAD 256
#define DUALBOARD_CHASSIS_FRAME_LEN 12
#define DUALBOARD_CHASSIS_TIMEOUT_MS 100U

// 底盘命令模式，使用 12 字节 USART10 小协议传输。
typedef enum {
    DUALBOARD_CHASSIS_SAFE = 0,
    DUALBOARD_CHASSIS_FREE = 1,
    DUALBOARD_CHASSIS_SPIN = 2
} DualBoard_Chassis_Mode_e;

// 反馈帧类型值。命令帧的第 4 字节仍然直接使用 DUALBOARD_CHASSIS_xxx。
#define DUALBOARD_FRAME_TYPE_FEEDBACK 0x81U

// 底盘反馈状态，用于（1）底盘板回传给（2）遥控板。
typedef enum {
    DUALBOARD_FB_SAFE = 0,
    DUALBOARD_FB_RUN = 1,
    DUALBOARD_FB_LOST = 2,
    DUALBOARD_FB_ERROR = 3
} DualBoard_Chassis_Feedback_Status_e;

// 固定 12 字节底盘帧:
// A5 01 seq type data0_i16 data1_i16 data2_i16 checksum 5A
// 命令帧 type=底盘模式，data0/1 为 vx/vy(mm/s)，data2 为 vw(mrad/s)。
// 反馈帧 type=0x81，data0 为状态，data1 为电机在线 bit，data2 为错误码。
typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t version;
    uint8_t seq;
    uint8_t mode;
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t vw_mrad_s;
    uint8_t checksum;
    uint8_t tail;
} B2B_Chassis_Frame_t;

// 接收端最近一次有效底盘命令，由底盘控制任务读取。
typedef struct {
    DualBoard_Chassis_Mode_e mode;
    float vx_mm_s;
    float vy_mm_s;
    float vw_mrad_s;
    uint32_t last_update_ms;
    uint8_t last_seq;
    bool is_online;
} B2B_Chassis_Cmd_t;

// 接收端最近一次有效底盘反馈，由遥控板后续 UI/调试逻辑读取。
typedef struct {
    DualBoard_Chassis_Feedback_Status_e status;
    uint8_t motor_online_bits;
    int16_t error_code;
    uint8_t last_seq;
    uint32_t last_update_ms;
    bool is_online;
} B2B_Chassis_Feedback_t;

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
extern B2B_Chassis_Cmd_t B2B_Chassis_Cmd;
extern B2B_Chassis_Feedback_t B2B_Chassis_Feedback;

void DualBoard_Comm_Init(void);

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len);
// 通过指定 UART 发送一帧底盘命令，例如遥控板使用 huart10。
uint8_t DualBoard_Send_Chassis(UART_HandleTypeDef *huart,
                               DualBoard_Chassis_Mode_e mode,
                               float vx_mm_s,
                               float vy_mm_s,
                               float vw_mrad_s);
// 通过指定 UART 发送一帧底盘反馈，例如底盘板使用 huart10。
uint8_t DualBoard_Send_Chassis_Feedback(UART_HandleTypeDef *huart,
                                        DualBoard_Chassis_Feedback_Status_e status,
                                        uint8_t motor_online_bits,
                                        int16_t error_code);
// 超过 DUALBOARD_CHASSIS_TIMEOUT_MS 没有有效命令时返回 false。
bool DualBoard_Chassis_Is_Online(void);
// 超过 DUALBOARD_CHASSIS_TIMEOUT_MS 没有有效反馈时返回 false。
bool DualBoard_Chassis_Feedback_Is_Online(void);

void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *Buff, uint16_t Size);
void DualBoard_UART_Rx_Callback(uint8_t *Buff, void *device_ptr, uint16_t Size);

void DualBoard_Task_Poll(void);

#endif //H7_FRAMEWORK_COMM_DUALBOARD_H
