#ifndef H7_FRAMEWORK_COMM_DUALBOARD_H
#define H7_FRAMEWORK_COMM_DUALBOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef enum {
    LINK_CAN,
    LINK_UART
} Comm_Link_e;

#define DUALBOARD_MAX_PAYLOAD 256U
#define DUALBOARD_CHASSIS_FRAME_LEN 12U
#define DUALBOARD_ENGINEER_FRAME_LEN 20U
#define DUALBOARD_CHASSIS_TIMEOUT_MS 100U

typedef enum {
    DUALBOARD_CHASSIS_SAFE = 0,
    DUALBOARD_CHASSIS_FREE = 1,
    DUALBOARD_CHASSIS_SPIN = 2
} DualBoard_Chassis_Mode_e;

#define DUALBOARD_FRAME_TYPE_FEEDBACK 0x81U

typedef enum {
    DUALBOARD_FB_SAFE = 0,
    DUALBOARD_FB_RUN = 1,
    DUALBOARD_FB_LOST = 2,
    DUALBOARD_FB_ERROR = 3
} DualBoard_Chassis_Feedback_Status_e;

/* Legacy 12-byte chassis command / feedback frame. */
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

/* Engineer 20-byte command frame: chassis command + picture targets. */
typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t version;
    uint8_t seq;
    uint8_t mode;
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t vw_mrad_s;
    int32_t picture_lift;
    int32_t picture_transverse;
    uint8_t checksum;
    uint8_t tail;
} B2B_Engineer_Frame_t;

typedef struct {
    DualBoard_Chassis_Mode_e mode;
    float vx_mm_s;
    float vy_mm_s;
    float vw_mrad_s;
    uint32_t last_update_ms;
    uint8_t last_seq;
    bool is_online;
} B2B_Chassis_Cmd_t;

typedef struct {
    int32_t picture_lift;
    int32_t picture_transverse;
    uint32_t last_update_ms;
    uint8_t last_seq;
    bool is_online;
} B2B_Picture_Cmd_t;

typedef struct {
    DualBoard_Chassis_Feedback_Status_e status;
    uint8_t motor_online_bits;
    int16_t error_code;
    uint8_t last_seq;
    uint32_t last_update_ms;
    bool is_online;
} B2B_Chassis_Feedback_t;

typedef struct __attribute__((packed)) {
    int16_t ch0:11;
    int16_t ch1:11;
    int16_t ch2:11;
    int16_t ch3:11;
    uint8_t s1:2;
    uint8_t s2:2;
} B2B_Tx_t;

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
extern B2B_Picture_Cmd_t B2B_Picture_Cmd;
extern B2B_Chassis_Feedback_t B2B_Chassis_Feedback;

void DualBoard_Comm_Init(void);

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len);
uint8_t DualBoard_Send_Chassis(UART_HandleTypeDef *huart,
                               DualBoard_Chassis_Mode_e mode,
                               float vx_mm_s,
                               float vy_mm_s,
                               float vw_mrad_s);
uint8_t DualBoard_Send_Engineer(UART_HandleTypeDef *huart,
                                DualBoard_Chassis_Mode_e mode,
                                float vx_mm_s,
                                float vy_mm_s,
                                float vw_mrad_s,
                                int32_t picture_lift,
                                int32_t picture_transverse);
uint8_t DualBoard_Send_Chassis_Feedback(UART_HandleTypeDef *huart,
                                        DualBoard_Chassis_Feedback_Status_e status,
                                        uint8_t motor_online_bits,
                                        int16_t error_code);
bool DualBoard_Chassis_Is_Online(void);
bool DualBoard_Picture_Is_Online(void);
bool DualBoard_Chassis_Feedback_Is_Online(void);

void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *Buff, uint16_t Size);
void DualBoard_UART_Rx_Callback(uint8_t *Buff, void *device_ptr, uint16_t Size);

void DualBoard_Task_Poll(void);

#endif
