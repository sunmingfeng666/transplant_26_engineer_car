//
// Created by CaoKangqi on 2026/1/19.
//
#include "Vofa.h"
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include "usbd_cdc_if.h"

/**
 * @brief 内部底层发送路由函数
 * @param huart 目标串口句柄，为 NULL 时默认走 USB CDC
 */
static void VOFA_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (huart == NULL) {
        // 句柄为空，默认使用 USB CDC
        CDC_Transmit_HS(data, len);
    } else {
        // 控制任务内禁止等待：上一帧未完成时直接丢弃本帧。
        if (huart->gState != HAL_UART_STATE_READY) return;
        HAL_UART_Transmit_IT(huart, data, len);
    }
}

/**
 * @brief VOFA+ JustFloat 协议可变参数发送函数
 * @param huart 串口句柄 (例如 &huart10)，填 NULL 走 USB CDC
 * @param channels_num 实际发送的通道数量 (1 ~ VOFA_MAX_CHANNELS)
 * @param ... 具体的 float 数据点
 * @note 不是浮点要强转成浮点，不能传整型，通道数上限由 VOFA_MAX_CHANNELS 定义
 */
void VOFA_JustFloat(UART_HandleTypeDef *huart, uint8_t channels_num, ...)
{
    if (channels_num == 0 || channels_num > VOFA_MAX_CHANNELS) return;

    static uint8_t send_buf[(VOFA_MAX_CHANNELS * 4) + 4];

    va_list args;
    va_start(args, channels_num);

    float temp_data;
    for (uint8_t i = 0; i < channels_num; i++) {
        temp_data = (float)va_arg(args, double);
        memcpy(&send_buf[i * 4], &temp_data, 4);
    }
    va_end(args);

    uint32_t tail_index = channels_num * 4;
    send_buf[tail_index]     = 0x00;
    send_buf[tail_index + 1] = 0x00;
    send_buf[tail_index + 2] = 0x80;
    send_buf[tail_index + 3] = 0x7F;

    VOFA_Transmit(huart, send_buf, tail_index + 4);
}

/**
 * @brief VOFA+ FireWater 协议可变参数发送函数
 * @param huart 串口句柄 (例如 &huart10)，填 NULL 走 USB CDC
 * @param channels_num 实际发送的通道数量
 * @param ... 具体的 float 数据点
 * @note 不是浮点要强转成浮点，不能传整型
 */
void VOFA_FireWater(UART_HandleTypeDef *huart, uint8_t channels_num, ...)
{
    if (channels_num == 0) return;

    static char text_buf[VOFA_TEXT_BUF_SIZE];
    uint32_t str_len = 0;

    va_list args;
    va_start(args, channels_num);
    for (uint8_t i = 0; i < channels_num; i++) {
        float temp_data = (float)va_arg(args, double);

        int written = snprintf(&text_buf[str_len], VOFA_TEXT_BUF_SIZE - str_len,
                               (i == channels_num - 1) ? "%.4f" : "%.4f,", temp_data);

        if (written < 0 || (str_len + written) >= VOFA_TEXT_BUF_SIZE - 2) {
            break;
        }
        str_len += written;
    }
    va_end(args);

    text_buf[str_len++] = '\n';
    text_buf[str_len]   = '\0';

    VOFA_Transmit(huart, (uint8_t*)text_buf, str_len);
}
