//
// Created by CaoKangqi on 2026/1/19.
//
#include "BSP_DWT.h"

DWT_Time_t SysTime;

static uint32_t CPU_FREQ_Hz;
static uint32_t CPU_FREQ_Hz_ms;
static uint32_t CPU_FREQ_Hz_us;

static uint32_t CYCCNT_RoundCount = 0;
static uint32_t CYCCNT_LAST = 0;
uint64_t CYCCNT64 = 0;

static void DWT_CNT_Update(void);

void DWT_Init(uint32_t CPU_Freq_MHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    #if defined(__CORTEX_M) && (__CORTEX_M == 7 || __CORTEX_M == 33)
    DWT->LAR = 0xC5ACCE55;
    #endif

    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_MHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RoundCount = 0;
    CYCCNT_LAST = 0;
}

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST) {
        CYCCNT_RoundCount++;
    }
    CYCCNT_LAST = cnt_now;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    DWT_CNT_Update();
    // 计算开机以来的绝对总 Tick 数
    CYCCNT64 = ((uint64_t)CYCCNT_RoundCount << 32) + cnt_now;
    // 减少 64 位除法的次数，提高执行效率
    uint64_t total_us = CYCCNT64 / CPU_FREQ_Hz_us;

    SysTime.s  = total_us / 1000000;
    SysTime.ms = (total_us % 1000000) / 1000;
    SysTime.us = total_us % 1000;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = (float)((uint32_t)(cnt_now - *cnt_last)) / CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    DWT_CNT_Update();
    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = (double)((uint32_t)(cnt_now - *cnt_last)) / CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    DWT_CNT_Update();
    return dt;
}

float DWT_GetTimeline_s(void) { DWT_SysTimeUpdate(); return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f; }
float DWT_GetTimeline_ms(void){ DWT_SysTimeUpdate(); return SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f; }
uint64_t DWT_GetTimeline_us(void) { DWT_SysTimeUpdate(); return (uint64_t)SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us; }


void DWT_Delay_us(uint32_t uSec)
{
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = uSec * CPU_FREQ_Hz_us;

    while ((DWT->CYCCNT - start_tick) < delay_ticks) {
    }
}

void DWT_Delay_ms(float Delay) { DWT_Delay_us((uint32_t)(Delay * 1000.0f)); }
void DWT_Delay_s(float Delay)  { DWT_Delay_us((uint32_t)(Delay * 1000000.0f)); }


/**
 * @brief  设置一个非阻塞的超时定时器
 * @param  timeout: 定时器句柄
 * @param  ms: 超时时间(毫秒)
 */
void DWT_Set_Timeout(DWT_Timeout_t *timeout, float ms) {
    timeout->start_tick = DWT->CYCCNT;
    timeout->delay_ticks = (uint32_t)(ms * CPU_FREQ_Hz_ms);
}

/**
 * @brief  检查是否已经超时
 * @retval true: 已超时 / false: 未超时
 */
bool DWT_Check_Timeout(DWT_Timeout_t *timeout) {
    return ((DWT->CYCCNT - timeout->start_tick) >= timeout->delay_ticks);
}


/* ================== 代码执行性能分析 ================== */

void DWT_Profile_Start(DWT_Profiler_t *profiler) {
    profiler->start_tick = DWT->CYCCNT;
}

void DWT_Profile_Stop(DWT_Profiler_t *profiler) {
    uint32_t delta_tick = DWT->CYCCNT - profiler->start_tick;
    profiler->cost_us = (float)delta_tick / CPU_FREQ_Hz_us;
    profiler->cost_ms = (float)delta_tick / CPU_FREQ_Hz_ms;
}