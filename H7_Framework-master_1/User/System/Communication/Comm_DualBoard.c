#include "Comm_DualBoard.h"
#include "BSP_FDCAN.h"
#include "BSP_DWT.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define DUALBOARD_SOF 0xA5U
#define DUALBOARD_TAIL 0x5AU
#define DUALBOARD_VERSION 1U
#define DUALBOARD_ENGINEER_VERSION 2U
#define CHASSIS_CHECKSUM_INDEX 10U
#define ENGINEER_CHECKSUM_INDEX 22U
#define ENGINEER_FEEDBACK_CHECKSUM_INDEX 22U

B2B_Tx_t Tx_Data = {0};
B2B_Rx_t Rx_Data = {0};
B2B_Chassis_Cmd_t B2B_Chassis_Cmd = {0};
B2B_Picture_Cmd_t B2B_Picture_Cmd = {0};
B2B_Chassis_Feedback_t B2B_Chassis_Feedback = {0};
B2B_Engineer_Feedback_t B2B_Engineer_Feedback = {0};

typedef struct {
    uint8_t buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t total_len;
    uint16_t offset;
    uint8_t seq;
    uint32_t last_tick;
    bool is_sending;
} CAN_TP_Tx_State_t;

typedef struct {
    uint8_t buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t current_len;
    uint8_t next_seq;
    bool is_active;
} CAN_TP_Rx_t;

static CAN_TP_Tx_State_t can_tx = {0};
static CAN_TP_Rx_t can_rx = {0};
static uint8_t uart_rx_buf[DUALBOARD_MAX_PAYLOAD];
static uint16_t uart_rx_len = 0;

static uint8_t chassis_tx_seq = 0;
static uint8_t engineer_tx_seq = 0;
static uint8_t chassis_feedback_tx_seq = 0;
static uint8_t engineer_feedback_tx_seq = 0;
static B2B_Chassis_Frame_t chassis_tx_frame;
static B2B_Engineer_Frame_t engineer_tx_frame;
static B2B_Chassis_Frame_t chassis_feedback_tx_frame;
static B2B_Engineer_Feedback_Frame_t engineer_feedback_tx_frame;

_Static_assert(sizeof(B2B_Engineer_Frame_t) == DUALBOARD_ENGINEER_FRAME_LEN,
               "B2B engineer command frame size mismatch");
_Static_assert(sizeof(B2B_Engineer_Feedback_Frame_t) == DUALBOARD_ENGINEER_FEEDBACK_FRAME_LEN,
               "B2B engineer feedback frame size mismatch");

static int16_t Float_To_Int16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

static uint8_t Frame_Checksum(const void *frame, uint8_t checksum_index)
{
    const uint8_t *bytes = (const uint8_t *)frame;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < checksum_index; i++) {
        sum += bytes[i];
    }
    return (uint8_t)sum;
}

static bool Chassis_Frame_Is_Valid(const B2B_Chassis_Frame_t *frame)
{
    if (frame == NULL) return false;
    if (frame->sof != DUALBOARD_SOF || frame->tail != DUALBOARD_TAIL) return false;
    if (frame->version != DUALBOARD_VERSION) return false;
    return frame->checksum == Frame_Checksum(frame, CHASSIS_CHECKSUM_INDEX);
}

static bool Engineer_Frame_Is_Valid(const B2B_Engineer_Frame_t *frame)
{
    if (frame == NULL) return false;
    if (frame->sof != DUALBOARD_SOF || frame->tail != DUALBOARD_TAIL) return false;
    if (frame->version != DUALBOARD_ENGINEER_VERSION) return false;
    return frame->checksum == Frame_Checksum(frame, ENGINEER_CHECKSUM_INDEX);
}

// 命令帧收件箱：通信层收到并通过校验的原始帧暂存于此，等 App 命令层取走解析。
// 单缓冲、后到覆盖先到（控制场景取最新命令即可），pending 最后置位保证数据完整。
static uint8_t  cmd_inbox_buf[DUALBOARD_MAX_PAYLOAD];
static uint16_t cmd_inbox_len = 0;
static volatile uint8_t cmd_inbox_pending = 0;

static void Inbox_Store(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U || len > sizeof(cmd_inbox_buf)) return;
    memcpy(cmd_inbox_buf, buf, len);
    cmd_inbox_len = len;
    cmd_inbox_pending = 1U;
}

uint8_t DualBoard_Take_Cmd_Frame(uint8_t *out, uint16_t out_cap, uint16_t *out_len)
{
    if (out == NULL || out_len == NULL) return 0U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t got = 0U;
    if (cmd_inbox_pending && cmd_inbox_len <= out_cap) {
        memcpy(out, cmd_inbox_buf, cmd_inbox_len);
        *out_len = cmd_inbox_len;
        cmd_inbox_pending = 0U;
        got = 1U;
    }
    if (primask == 0U) __enable_irq();
    return got;
}

void DualBoard_Comm_Init(void)
{
    memset(&Tx_Data, 0, sizeof(Tx_Data));
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(&B2B_Chassis_Cmd, 0, sizeof(B2B_Chassis_Cmd));
    memset(&B2B_Picture_Cmd, 0, sizeof(B2B_Picture_Cmd));
    memset(&B2B_Chassis_Feedback, 0, sizeof(B2B_Chassis_Feedback));
    memset(&B2B_Engineer_Feedback, 0, sizeof(B2B_Engineer_Feedback));
    cmd_inbox_len = 0;
    cmd_inbox_pending = 0U;
}

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len)
{
    if (data_ptr == NULL || len > DUALBOARD_MAX_PAYLOAD) return 1U;

    if (link == LINK_UART) {
        return 0U;
    }
    if (link == LINK_CAN) {
        if (can_tx.is_sending) return 2U;
        can_tx.total_len = len;
        memcpy(can_tx.buf, data_ptr, len);
        can_tx.offset = 0;
        can_tx.seq = 0;
        can_tx.is_sending = true;
        return 0U;
    }
    return 1U;
}

uint8_t DualBoard_Send_Chassis(UART_HandleTypeDef *huart,
                               DualBoard_Chassis_Mode_e mode,
                               float vx_mm_s,
                               float vy_mm_s,
                               float vw_mrad_s)
{
    if (huart == NULL) return 1U;
    if (huart->gState != HAL_UART_STATE_READY) return 2U;

    chassis_tx_frame.sof = DUALBOARD_SOF;
    chassis_tx_frame.version = DUALBOARD_VERSION;
    chassis_tx_frame.seq = chassis_tx_seq++;
    chassis_tx_frame.mode = (uint8_t)mode;
    chassis_tx_frame.vx_mm_s = Float_To_Int16(vx_mm_s);
    chassis_tx_frame.vy_mm_s = Float_To_Int16(vy_mm_s);
    chassis_tx_frame.vw_mrad_s = Float_To_Int16(vw_mrad_s);
    chassis_tx_frame.checksum = 0U;
    chassis_tx_frame.tail = DUALBOARD_TAIL;
    chassis_tx_frame.checksum = Frame_Checksum(&chassis_tx_frame, CHASSIS_CHECKSUM_INDEX);

    return (HAL_UART_Transmit(huart, (uint8_t *)&chassis_tx_frame, sizeof(chassis_tx_frame), 2U) == HAL_OK) ? 0U : 2U;
}

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
                                uint8_t ui_flags)
{
    if (huart == NULL) return 1U;
    if (huart->gState != HAL_UART_STATE_READY) return 2U;
    if (mechanism_action > DUALBOARD_ACTION_CLEAR_FAULT || store_slot > 3U) return 1U;

    engineer_tx_frame.sof = DUALBOARD_SOF;
    engineer_tx_frame.version = DUALBOARD_ENGINEER_VERSION;
    engineer_tx_frame.seq = engineer_tx_seq++;
    engineer_tx_frame.mode = (uint8_t)mode;
    engineer_tx_frame.vx_mm_s = Float_To_Int16(vx_mm_s);
    engineer_tx_frame.vy_mm_s = Float_To_Int16(vy_mm_s);
    engineer_tx_frame.vw_mrad_s = Float_To_Int16(vw_mrad_s);
    engineer_tx_frame.picture_lift = picture_lift;
    engineer_tx_frame.picture_transverse = picture_transverse;
    engineer_tx_frame.mechanism_action = (uint8_t)mechanism_action;
    engineer_tx_frame.store_slot = store_slot;
    engineer_tx_frame.action_seq = action_seq;
    engineer_tx_frame.ui_flags = ui_flags;
    engineer_tx_frame.checksum = 0U;
    engineer_tx_frame.tail = DUALBOARD_TAIL;
    engineer_tx_frame.checksum = Frame_Checksum(&engineer_tx_frame, ENGINEER_CHECKSUM_INDEX);

    return (HAL_UART_Transmit(huart, (uint8_t *)&engineer_tx_frame, sizeof(engineer_tx_frame), 2U) == HAL_OK) ? 0U : 2U;
}

uint8_t DualBoard_Send_Engineer_Feedback(UART_HandleTypeDef *huart,
                                         const B2B_Engineer_Feedback_t *feedback)
{
    if (huart == NULL || feedback == NULL) return 1U;
    if (huart->gState != HAL_UART_STATE_READY) return 2U;

    engineer_feedback_tx_frame.sof = DUALBOARD_SOF;
    engineer_feedback_tx_frame.version = DUALBOARD_ENGINEER_VERSION;
    engineer_feedback_tx_frame.seq = engineer_feedback_tx_seq++;
    engineer_feedback_tx_frame.type = DUALBOARD_FRAME_TYPE_ENGINEER_FEEDBACK;
    engineer_feedback_tx_frame.status = (uint8_t)feedback->status;
    engineer_feedback_tx_frame.chassis_online_bits = feedback->chassis_online_bits;
    engineer_feedback_tx_frame.mechanism_online_bits = feedback->mechanism_online_bits;
    engineer_feedback_tx_frame.limit_bits = feedback->limit_bits;
    engineer_feedback_tx_frame.action_bits = feedback->action_bits;
    engineer_feedback_tx_frame.error_code = feedback->error_code;
    engineer_feedback_tx_frame.completed_action_seq = feedback->completed_action_seq;
    engineer_feedback_tx_frame.picture_lift_pos = feedback->picture_lift_pos;
    engineer_feedback_tx_frame.picture_transverse_pos = feedback->picture_transverse_pos;
    engineer_feedback_tx_frame.store_pos_mrad = feedback->store_pos_mrad;
    engineer_feedback_tx_frame.checksum = 0U;
    engineer_feedback_tx_frame.tail = DUALBOARD_TAIL;
    engineer_feedback_tx_frame.checksum =
        Frame_Checksum(&engineer_feedback_tx_frame, ENGINEER_FEEDBACK_CHECKSUM_INDEX);

    // 中断发送，避免在 1kHz 控制任务里阻塞(轮询发 24B@115200 约 2ms)。
    // 帧缓冲为静态，gState 就绪判断已防重入/覆盖。
    return (HAL_UART_Transmit_IT(huart, (uint8_t *)&engineer_feedback_tx_frame,
                                 sizeof(engineer_feedback_tx_frame)) == HAL_OK) ? 0U : 2U;
}

uint8_t DualBoard_Send_Chassis_Feedback(UART_HandleTypeDef *huart,
                                        DualBoard_Chassis_Feedback_Status_e status,
                                        uint8_t motor_online_bits,
                                        int16_t error_code)
{
    if (huart == NULL) return 1U;
    if (huart->gState != HAL_UART_STATE_READY) return 2U;

    chassis_feedback_tx_frame.sof = DUALBOARD_SOF;
    chassis_feedback_tx_frame.version = DUALBOARD_VERSION;
    chassis_feedback_tx_frame.seq = chassis_feedback_tx_seq++;
    chassis_feedback_tx_frame.mode = DUALBOARD_FRAME_TYPE_FEEDBACK;
    chassis_feedback_tx_frame.vx_mm_s = (int16_t)status;
    chassis_feedback_tx_frame.vy_mm_s = (int16_t)motor_online_bits;
    chassis_feedback_tx_frame.vw_mrad_s = error_code;
    chassis_feedback_tx_frame.checksum = 0U;
    chassis_feedback_tx_frame.tail = DUALBOARD_TAIL;
    chassis_feedback_tx_frame.checksum = Frame_Checksum(&chassis_feedback_tx_frame, CHASSIS_CHECKSUM_INDEX);

    return (HAL_UART_Transmit(huart, (uint8_t *)&chassis_feedback_tx_frame, sizeof(chassis_feedback_tx_frame), 2U) == HAL_OK) ? 0U : 2U;
}

bool DualBoard_Chassis_Is_Online(void)
{
    if (!B2B_Chassis_Cmd.is_online) return false;
    if ((HAL_GetTick() - B2B_Chassis_Cmd.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Chassis_Cmd.is_online = false;
        return false;
    }
    return true;
}

bool DualBoard_Picture_Is_Online(void)
{
    if (!B2B_Picture_Cmd.is_online) return false;
    if ((HAL_GetTick() - B2B_Picture_Cmd.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Picture_Cmd.is_online = false;
        return false;
    }
    return true;
}

bool DualBoard_Chassis_Feedback_Is_Online(void)
{
    if (!B2B_Chassis_Feedback.is_online) return false;
    if ((HAL_GetTick() - B2B_Chassis_Feedback.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Chassis_Feedback.is_online = false;
        return false;
    }
    return true;
}

bool DualBoard_Engineer_Feedback_Is_Online(void)
{
    if (!B2B_Engineer_Feedback.is_online) return false;
    if ((HAL_GetTick() - B2B_Engineer_Feedback.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Engineer_Feedback.is_online = false;
        return false;
    }
    return true;
}

void DualBoard_Task_Poll(void)
{
    if (!can_tx.is_sending) return;

    uint32_t now = DWT->CYCCNT;
    uint32_t interval_ticks = 100U * 550U;
    if (can_tx.seq > 0U && (now - can_tx.last_tick) < interval_ticks) return;

    uint8_t tx_buf[8];
    uint16_t remain = can_tx.total_len - can_tx.offset;
    uint8_t chunk_size = (remain > 7U) ? 7U : (uint8_t)remain;
    uint8_t is_last = (remain <= 7U) ? 1U : 0U;

    tx_buf[0] = (uint8_t)((is_last << 7U) | (can_tx.seq & 0x7FU));
    memcpy(&tx_buf[1], &can_tx.buf[can_tx.offset], chunk_size);
    if (chunk_size < 7U) memset(&tx_buf[1 + chunk_size], 0, 7U - chunk_size);

    if (FDCAN_Send_Msg(&hfdcan1, 0x500, tx_buf, 8) == 0) {
        can_tx.offset += chunk_size;
        can_tx.seq++;
        can_tx.last_tick = now;
        if (is_last) can_tx.is_sending = false;
    }
}

void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data)
{
    (void)device_ptr;
    if (data == NULL) return;

    uint8_t ctrl = data[0];
    uint8_t is_last = (uint8_t)((ctrl >> 7U) & 0x01U);
    uint8_t seq = ctrl & 0x7FU;
    uint8_t payload_len = 7U;

    if (seq == 0U) {
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
                DualBoard_UART_Rx(can_rx.buf, can_rx.current_len);
                can_rx.is_active = false;
            }
        } else {
            can_rx.is_active = false;
        }
    }
}

void DualBoard_UART_Rx(uint8_t *Buff, uint16_t Size)
{
    if (Buff == NULL || Size == 0U) return;

    // 通信层只做完整性校验+入库，不解析业务字段。解析交给 App 命令层。
    // 24B：工程车命令帧或整车反馈帧。反馈帧本板(执行板)不消费，直接丢弃。
    if (Size == sizeof(B2B_Engineer_Frame_t)) {
        if (Buff[3] == DUALBOARD_FRAME_TYPE_ENGINEER_FEEDBACK) return;
        if (Engineer_Frame_Is_Valid((const B2B_Engineer_Frame_t *)Buff)) {
            Inbox_Store(Buff, Size);
        }
        return;
    }

    // 12B：旧版底盘命令帧。
    if (Size == sizeof(B2B_Chassis_Frame_t)) {
        if (Chassis_Frame_Is_Valid((const B2B_Chassis_Frame_t *)Buff)) {
            Inbox_Store(Buff, Size);
        }
        return;
    }

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

void DualBoard_UART_Rx_Callback(uint8_t *Buff, void *device_ptr, uint16_t Size)
{
    (void)device_ptr;
    DualBoard_UART_Rx(Buff, Size);
}
