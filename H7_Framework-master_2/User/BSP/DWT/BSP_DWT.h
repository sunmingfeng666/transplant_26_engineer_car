//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef G4_FRAMEWORK_BSP_DWT_H
#define G4_FRAMEWORK_BSP_DWT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* --- 原有核心 API --- */
void DWT_Init(uint32_t CPU_Freq_MHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_SysTimeUpdate(void);
void DWT_Delay_s(float Delay);
void DWT_Delay_ms(float Delay);
void DWT_Delay_us(uint32_t uSec);

/* ================== 新增功能 ================== */

/* 1. 非阻塞超时定时器 (用于状态机或主循环轮询) */
typedef struct {
    uint32_t start_tick;
    uint32_t delay_ticks;
} DWT_Timeout_t;

void DWT_Set_Timeout(DWT_Timeout_t *timeout, float ms);
bool DWT_Check_Timeout(DWT_Timeout_t *timeout);

/* 2. 代码执行耗时评估器 (用于算法性能分析) */
typedef struct {
    uint32_t start_tick;
    float    cost_ms;
    float    cost_us;
} DWT_Profiler_t;

void DWT_Profile_Start(DWT_Profiler_t *profiler);
void DWT_Profile_Stop(DWT_Profiler_t *profiler);

extern DWT_Time_t SysTime;

#endif //G4_FRAMEWORK_BSP_DWT_H