//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef H7_FRAMEWORK_VOFA_H
#define H7_FRAMEWORK_VOFA_H

#include <stdint.h>
#include "usart.h"

#define VOFA_MAX_CHANNELS      20    // JustFloat 单次发送的最大通道数
#define VOFA_TEXT_BUF_SIZE     256   // FireWater 文本协议的最大单行缓冲区大小

void VOFA_JustFloat(UART_HandleTypeDef *huart, uint8_t channels_num, ...);
void VOFA_FireWater(UART_HandleTypeDef *huart, uint8_t channels_num, ...);

#endif //H7_FRAMEWORK_VOFA_H