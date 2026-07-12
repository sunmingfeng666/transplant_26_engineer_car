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
#define DUALBOARD_ENGINEER_FRAME_LEN 24U
#define DUALBOARD_ENGINEER_FEEDBACK_FRAME_LEN 24U
#define DUALBOARD_CHASSIS_TIMEOUT_MS 100U

typedef enum {
    DUALBOARD_CHASSIS_SAFE = 0,
    DUALBOARD_CHASSIS_FREE = 1,
    DUALBOARD_CHASSIS_SPIN = 2
} DualBoard_Chassis_Mode_e;

#define DUALBOARD_FRAME_TYPE_FEEDBACK 0x81U
#define DUALBOARD_FRAME_TYPE_ENGINEER_FEEDBACK 0x82U

typedef enum {
    DUALBOARD_ACTION_HOLD = 0,
    DUALBOARD_ACTION_EXECUTE = 1,
    DUALBOARD_ACTION_HOME_PICTURE = 2,
    DUALBOARD_ACTION_STOP_ALL = 3,
    DUALBOARD_ACTION_CLEAR_FAULT = 4
} DualBoard_Mechanism_Action_e;

#define DUALBOARD_MECHANISM_LIFT_ONLINE       (1U << 0)
#define DUALBOARD_MECHANISM_TRANSVERSE_ONLINE (1U << 1)
#define DUALBOARD_MECHANISM_STORE_ONLINE      (1U << 2)
#define DUALBOARD_LIMIT_LIFT_BOTTOM            (1U << 0)
#define DUALBOARD_LIMIT_TRANSVERSE_ZERO        (1U << 1)
#define DUALBOARD_ACTION_LIFT_DONE              (1U << 0)
#define DUALBOARD_ACTION_TRANSVERSE_DONE        (1U << 1)
#define DUALBOARD_ACTION_STORE_DONE             (1U << 2)
#define DUALBOARD_ACTION_HOMING_ACTIVE          (1U << 3)
#define DUALBOARD_ACTION_HOMING_DONE            (1U << 4)
#define DUALBOARD_ACTION_FAULT                  (1U << 7)
#define DUALBOARD_UI_CLAMP_CLOSED               (1U << 0)

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

/* 工程车 V2 命令帧：运输序号用于链路观测，动作序号用于动作去重。 */
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
    uint8_t mechanism_action;
    uint8_t store_slot;
    uint8_t action_seq;
    uint8_t ui_flags;
    uint8_t checksum;
    uint8_t tail;
} B2B_Engineer_Frame_t;

typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t version;
    uint8_t seq;
    uint8_t type;
    uint8_t status;
    uint8_t chassis_online_bits;
    uint8_t mechanism_online_bits;
    uint8_t limit_bits;
    uint8_t action_bits;
    int16_t error_code;
    uint8_t completed_action_seq;
    int32_t picture_lift_pos;
    int32_t picture_transverse_pos;
    int16_t store_pos_mrad;
    uint8_t checksum;
    uint8_t tail;
} B2B_Engineer_Feedback_Frame_t;

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
    DualBoard_Mechanism_Action_e mechanism_action;
    uint8_t store_slot;
    uint8_t action_seq;
    uint8_t ui_flags;
    uint32_t last_update_ms;
    uint8_t last_seq;
    bool is_online;
} B2B_Picture_Cmd_t;

typedef struct {
    DualBoard_Chassis_Feedback_Status_e status;
    uint8_t chassis_online_bits;
    uint8_t mechanism_online_bits;
    uint8_t limit_bits;
    uint8_t action_bits;
    int16_t error_code;
    uint8_t completed_action_seq;
    int32_t picture_lift_pos;
    int32_t picture_transverse_pos;
    int16_t store_pos_mrad;
    uint32_t last_update_ms;
    bool is_online;
} B2B_Engineer_Feedback_t;

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
extern B2B_Engineer_Feedback_t B2B_Engineer_Feedback;

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
                                int32_t picture_transverse,
                                DualBoard_Mechanism_Action_e mechanism_action,
                                uint8_t store_slot,
                                uint8_t action_seq,
                                uint8_t ui_flags);
uint8_t DualBoard_Send_Chassis_Feedback(UART_HandleTypeDef *huart,
                                        DualBoard_Chassis_Feedback_Status_e status,
                                        uint8_t motor_online_bits,
                                        int16_t error_code);
uint8_t DualBoard_Send_Engineer_Feedback(UART_HandleTypeDef *huart,
                                         const B2B_Engineer_Feedback_t *feedback);
bool DualBoard_Chassis_Is_Online(void);
bool DualBoard_Picture_Is_Online(void);
bool DualBoard_Chassis_Feedback_Is_Online(void);
bool DualBoard_Engineer_Feedback_Is_Online(void);

void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *Buff, uint16_t Size);
void DualBoard_UART_Rx_Callback(uint8_t *Buff, void *device_ptr, uint16_t Size);

// 取出一帧已通过通信校验的原始反馈帧（中断安全）。
// 通信层只负责收帧+校验+交付；帧到 B2B_Engineer_Feedback 的解析由 App 命令层完成。
// 返回 1 表示取到一帧并写入 out/out_len，0 表示当前无新帧。
uint8_t DualBoard_Take_Feedback_Frame(uint8_t *out, uint16_t out_cap, uint16_t *out_len);

void DualBoard_Task_Poll(void);

#endif
