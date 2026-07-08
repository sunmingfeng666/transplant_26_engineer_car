//
// Created by CaoKangqi on 2026/6/22.
//
#include "Comm_DualBoard.h"
#include "Message_Center.h"
#include "BSP_FDCAN.h"
#include "BSP_DWT.h"
#include "stm32h7xx_hal.h"
#include <string.h>

B2B_Tx_t Tx_Data = {0};
B2B_Rx_t Rx_Data = {0};
// 接收端共享底盘命令，底盘控制任务会读取它。
B2B_Chassis_Cmd_t B2B_Chassis_Cmd = {0};
// 接收端共享底盘反馈，遥控板后续 UI/调试逻辑会读取它。
B2B_Chassis_Feedback_t B2B_Chassis_Feedback = {0};

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
// 每发送一帧底盘命令自增一次，便于后续统计丢帧。
static uint8_t chassis_tx_seq = 0;
// 每发送一帧底盘反馈自增一次，和命令帧分开计数。
static uint8_t chassis_feedback_tx_seq = 0;
static B2B_Chassis_Frame_t chassis_tx_frame;
static B2B_Chassis_Frame_t chassis_feedback_tx_frame;

static int16_t Float_To_Int16(float value)
{
    // 打包到 12 字节帧前先限幅，避免 float 超出 int16_t 范围。
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

static uint8_t Chassis_Checksum(const B2B_Chassis_Frame_t *frame)
{
    // 前 10 字节累加和的低 8 位，checksum 和 tail 不参与计算。
    const uint8_t *bytes = (const uint8_t *)frame;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 10; i++) {
        sum += bytes[i];
    }
    return (uint8_t)sum;
}

static void Parse_Chassis_Frame(const B2B_Chassis_Frame_t *frame)
{
    if (frame == NULL) return;
    // 先校验帧头、帧尾、版本和校验和，坏帧不更新共享数据。
    if (frame->sof != 0xA5U || frame->tail != 0x5AU || frame->version != 1U) return;
    if (frame->checksum != Chassis_Checksum(frame)) return;

    if (frame->mode == DUALBOARD_FRAME_TYPE_FEEDBACK) {
        B2B_Chassis_Feedback.status = (DualBoard_Chassis_Feedback_Status_e)frame->vx_mm_s;
        B2B_Chassis_Feedback.motor_online_bits = (uint8_t)frame->vy_mm_s;
        B2B_Chassis_Feedback.error_code = frame->vw_mrad_s;
        B2B_Chassis_Feedback.last_seq = frame->seq;
        B2B_Chassis_Feedback.last_update_ms = HAL_GetTick();
        B2B_Chassis_Feedback.is_online = true;
        return;
    }

    // 其他 0/1/2 类型按底盘命令处理，超过已知模式的帧丢弃。
    if (frame->mode > DUALBOARD_CHASSIS_SPIN) return;
    B2B_Chassis_Cmd.mode = (DualBoard_Chassis_Mode_e)frame->mode;
    B2B_Chassis_Cmd.vx_mm_s = (float)frame->vx_mm_s;
    B2B_Chassis_Cmd.vy_mm_s = (float)frame->vy_mm_s;
    B2B_Chassis_Cmd.vw_mrad_s = (float)frame->vw_mrad_s;
    B2B_Chassis_Cmd.last_seq = frame->seq;
    B2B_Chassis_Cmd.last_update_ms = HAL_GetTick();
    B2B_Chassis_Cmd.is_online = true;
}

void DualBoard_Comm_Init(void)
{
    memset(&Tx_Data, 0, sizeof(B2B_Tx_t));
    memset(&Rx_Data, 0, sizeof(B2B_Rx_t));
    memset(&B2B_Chassis_Cmd, 0, sizeof(B2B_Chassis_Cmd));
    memset(&B2B_Chassis_Feedback, 0, sizeof(B2B_Chassis_Feedback));
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

uint8_t DualBoard_Send_Chassis(UART_HandleTypeDef *huart,
                               DualBoard_Chassis_Mode_e mode,
                               float vx_mm_s,
                               float vy_mm_s,
                               float vw_mrad_s)
{
    if (huart == NULL) return 1;
    // 避免 UART 忙时重入发送。当前用短阻塞发送，配合 5ms/20ms 周期足够轻。
    if (huart->gState != HAL_UART_STATE_READY) return 2;

    // 底盘命令帧和旧的 B2B_Tx_t 裸结构保持独立，避免互相污染。
    chassis_tx_frame.sof = 0xA5U;
    chassis_tx_frame.version = 1U;
    chassis_tx_frame.seq = chassis_tx_seq++;
    chassis_tx_frame.mode = (uint8_t)mode;
    chassis_tx_frame.vx_mm_s = Float_To_Int16(vx_mm_s);
    chassis_tx_frame.vy_mm_s = Float_To_Int16(vy_mm_s);
    chassis_tx_frame.vw_mrad_s = Float_To_Int16(vw_mrad_s);
    chassis_tx_frame.checksum = 0U;
    chassis_tx_frame.tail = 0x5AU;
    chassis_tx_frame.checksum = Chassis_Checksum(&chassis_tx_frame);

    return (HAL_UART_Transmit(huart, (uint8_t *)&chassis_tx_frame, sizeof(chassis_tx_frame), 2U) == HAL_OK) ? 0U : 2U;
}

uint8_t DualBoard_Send_Chassis_Feedback(UART_HandleTypeDef *huart,
                                        DualBoard_Chassis_Feedback_Status_e status,
                                        uint8_t motor_online_bits,
                                        int16_t error_code)
{
    if (huart == NULL) return 1;
    // 反馈帧也很短，沿用短阻塞发送，避免 TX DMA circular 反复发送旧帧。
    if (huart->gState != HAL_UART_STATE_READY) return 2;

    chassis_feedback_tx_frame.sof = 0xA5U;
    chassis_feedback_tx_frame.version = 1U;
    chassis_feedback_tx_frame.seq = chassis_feedback_tx_seq++;
    chassis_feedback_tx_frame.mode = DUALBOARD_FRAME_TYPE_FEEDBACK;
    chassis_feedback_tx_frame.vx_mm_s = (int16_t)status;
    chassis_feedback_tx_frame.vy_mm_s = (int16_t)motor_online_bits;
    chassis_feedback_tx_frame.vw_mrad_s = error_code;
    chassis_feedback_tx_frame.checksum = 0U;
    chassis_feedback_tx_frame.tail = 0x5AU;
    chassis_feedback_tx_frame.checksum = Chassis_Checksum(&chassis_feedback_tx_frame);

    return (HAL_UART_Transmit(huart, (uint8_t *)&chassis_feedback_tx_frame, sizeof(chassis_feedback_tx_frame), 2U) == HAL_OK) ? 0U : 2U;
}

bool DualBoard_Chassis_Is_Online(void)
{
    if (!B2B_Chassis_Cmd.is_online) return false;
    // 底盘控制任务会把超时当成安全模式处理。
    if ((HAL_GetTick() - B2B_Chassis_Cmd.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Chassis_Cmd.is_online = false;
        return false;
    }
    return true;
}

bool DualBoard_Chassis_Feedback_Is_Online(void)
{
    if (!B2B_Chassis_Feedback.is_online) return false;
    // 遥控板可用这个接口判断底盘板回传是否还在线。
    if ((HAL_GetTick() - B2B_Chassis_Feedback.last_update_ms) > DUALBOARD_CHASSIS_TIMEOUT_MS) {
        B2B_Chassis_Feedback.is_online = false;
        return false;
    }
    return true;
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

    // 新移植路径：解析遥控板/底盘板之间的固定长度底盘帧。
    if (Size == sizeof(B2B_Chassis_Frame_t)) {
        Parse_Chassis_Frame((const B2B_Chassis_Frame_t *)Buff);
        return;
    }

    // 兼容旧框架路径：仍然保留原始 B2B_Rx_t 裸结构接收。
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
