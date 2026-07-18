#include "Custom_Controller_Link.h"

#include <string.h>

#include "CRC_DJI.h"
#include "Comm_DualBoard.h"
#include "usart.h"

#define CUSTOM_CONTROLLER_SOF              0xA5U
#define CUSTOM_CONTROLLER_PAYLOAD_LENGTH   30U
#define CUSTOM_CONTROLLER_CMD_CONTROL      0x0302U
#define CUSTOM_CONTROLLER_CMD_FEEDBACK     0x0309U
#define CUSTOM_CONTROLLER_ALL_JOINT_ONLINE 0x3FU
#define CUSTOM_CONTROLLER_TX_PERIOD_MS     100U

typedef struct __attribute__((packed)) {
    int16_t joint_mrad[6];
    uint8_t clamp;
    uint8_t main_switch;
    int8_t reserved[16];
} Controller_Command_Payload_t;

typedef struct __attribute__((packed)) {
    uint8_t arm_ready;
    float joint_rad[6];
    int8_t reserved[5];
} Controller_Feedback_Payload_t;

typedef union {
    struct __attribute__((packed)) {
        uint8_t sof;
        uint16_t data_length;
        uint8_t sequence;
        uint8_t crc8;
        uint16_t command_id;
        Controller_Feedback_Payload_t payload;
        uint16_t crc16;
    } fields;
    uint8_t bytes[CUSTOM_CONTROLLER_FRAME_LENGTH];
} Controller_Feedback_Frame_t;

_Static_assert(sizeof(Controller_Command_Payload_t) == CUSTOM_CONTROLLER_PAYLOAD_LENGTH,
               "controller command payload size mismatch");
_Static_assert(sizeof(Controller_Feedback_Payload_t) == CUSTOM_CONTROLLER_PAYLOAD_LENGTH,
               "controller feedback payload size mismatch");
_Static_assert(sizeof(Controller_Feedback_Frame_t) == CUSTOM_CONTROLLER_FRAME_LENGTH,
               "controller feedback frame size mismatch");

volatile Custom_Controller_Link_Debug_t custom_controller_link_debug = {0};

/* 中断发送期间不能修改帧，所以使用独立静态缓冲。 */
static Controller_Feedback_Frame_t feedback_tx_frame;
static uint32_t last_send_ms;

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

void Custom_Controller_Link_Init(void)
{
    memset((void *)&custom_controller_link_debug, 0, sizeof(custom_controller_link_debug));
    memset(&feedback_tx_frame, 0, sizeof(feedback_tx_frame));
    last_send_ms = HAL_GetTick();
}

void Custom_Controller_Link_Rx_Callback(uint8_t *data, void *device_ptr, uint16_t size)
{
    (void)device_ptr;
    if (data == NULL || size != CUSTOM_CONTROLLER_FRAME_LENGTH) {
        ++custom_controller_link_debug.rx_format_error_count;
        return;
    }
    if (data[0] != CUSTOM_CONTROLLER_SOF ||
        read_u16_le(&data[1]) != CUSTOM_CONTROLLER_PAYLOAD_LENGTH ||
        read_u16_le(&data[5]) != CUSTOM_CONTROLLER_CMD_CONTROL) {
        ++custom_controller_link_debug.rx_format_error_count;
        return;
    }
    if (!Verify_CRC8_Check_Sum(data, 5U) ||
        !Verify_CRC16_Check_Sum(data, CUSTOM_CONTROLLER_FRAME_LENGTH)) {
        ++custom_controller_link_debug.rx_crc_error_count;
        return;
    }

    const Controller_Command_Payload_t *payload =
        (const Controller_Command_Payload_t *)&data[7];
    memcpy((void *)custom_controller_link_debug.controller_joint_mrad,
           payload->joint_mrad, sizeof(payload->joint_mrad));
    custom_controller_link_debug.controller_clamp = payload->clamp;
    custom_controller_link_debug.controller_main_switch = payload->main_switch;
    custom_controller_link_debug.controller_sequence = data[3];
    custom_controller_link_debug.last_rx_ms = HAL_GetTick();
    ++custom_controller_link_debug.rx_count;
}

static void build_feedback_frame(void)
{
    const uint8_t arm_ready =
        (B2B_Arm_Feedback.is_online &&
         B2B_Arm_Feedback.online_mask == CUSTOM_CONTROLLER_ALL_JOINT_ONLINE) ? 1U : 0U;

    feedback_tx_frame.fields.sof = CUSTOM_CONTROLLER_SOF;
    feedback_tx_frame.fields.data_length = CUSTOM_CONTROLLER_PAYLOAD_LENGTH;
    feedback_tx_frame.fields.sequence = (uint8_t)(feedback_tx_frame.fields.sequence + 1U);
    feedback_tx_frame.fields.crc8 = 0U;
    Append_CRC8_Check_Sum(feedback_tx_frame.bytes, 5U);
    feedback_tx_frame.fields.command_id = CUSTOM_CONTROLLER_CMD_FEEDBACK;
    feedback_tx_frame.fields.payload.arm_ready = arm_ready;
    memcpy(feedback_tx_frame.fields.payload.joint_rad,
           B2B_Arm_Feedback.position,
           sizeof(feedback_tx_frame.fields.payload.joint_rad));
    memset(feedback_tx_frame.fields.payload.reserved, 0,
           sizeof(feedback_tx_frame.fields.payload.reserved));
    /* 保留字节0明确携带六轴在线位图，便于控制器区分具体掉线关节。 */
    feedback_tx_frame.fields.payload.reserved[0] = (int8_t)B2B_Arm_Feedback.online_mask;
    feedback_tx_frame.fields.payload.reserved[1] = B2B_Arm_Feedback.is_online ? 1 : 0;
    feedback_tx_frame.fields.crc16 = 0U;
    Append_CRC16_Check_Sum(feedback_tx_frame.bytes, CUSTOM_CONTROLLER_FRAME_LENGTH);

    memcpy((void *)custom_controller_link_debug.arm_position_rad,
           B2B_Arm_Feedback.position, sizeof(B2B_Arm_Feedback.position));
    custom_controller_link_debug.arm_online_mask = B2B_Arm_Feedback.online_mask;
    custom_controller_link_debug.arm_ready = arm_ready;
    custom_controller_link_debug.tx_sequence = feedback_tx_frame.fields.sequence;
    memcpy((void *)custom_controller_link_debug.tx_frame,
           feedback_tx_frame.bytes, CUSTOM_CONTROLLER_FRAME_LENGTH);
}

void Custom_Controller_Link_Update(void)
{
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_send_ms) < CUSTOM_CONTROLLER_TX_PERIOD_MS) return;
    last_send_ms = now;

    if (huart1.gState != HAL_UART_STATE_READY) {
        ++custom_controller_link_debug.tx_busy_count;
        return;
    }

    build_feedback_frame();
    const HAL_StatusTypeDef status = HAL_UART_Transmit_IT(
        &huart1, feedback_tx_frame.bytes, CUSTOM_CONTROLLER_FRAME_LENGTH);
    if (status == HAL_OK) {
        custom_controller_link_debug.last_tx_ms = now;
        ++custom_controller_link_debug.tx_count;
    } else if (status == HAL_BUSY) {
        ++custom_controller_link_debug.tx_busy_count;
    } else {
        ++custom_controller_link_debug.tx_error_count;
    }
}
