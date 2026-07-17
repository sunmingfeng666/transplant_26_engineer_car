#include "Arm_MatlabDebug.h"

#if ARM_MATLAB_DEBUG_ENABLE

#include <math.h>
#include <string.h>

#include "stm32h7xx_hal.h"

/* 机械臂 J2/J4/J5 在 6 维数组中的下标（从 0 开始） */
#define ARM_AXIS_J2 1U
#define ARM_AXIS_J4 3U
#define ARM_AXIS_J5 4U

/* JustFloat 帧尾：0x00 0x00 0x80 0x7F（即 float +Inf 的小端字节）。 */
static const uint8_t JF_FOOTER[4] = {0x00, 0x00, 0x80, 0x7F};

/* 运行时使能开关：默认 0。上车时先机械支撑好，再设为 1，逐轴小幅度验证。 */
volatile uint8_t Arm_MatlabDebug_Enable = 0U;

/* 最近一帧解析出的 6 个关节角（弧度）。 */
static volatile float s_rx_joint[ARM_MATLAB_DEBUG_CHANNELS];
static volatile uint32_t s_last_frame_tick;  /* 最后一帧时间戳，用于掉线判断 */
static volatile uint32_t s_frame_count;      /* 累计收到的有效帧数 */
static volatile uint8_t s_frame_valid;       /* 至少成功解析过一帧（用于首帧检测） */

/* 逐字节解析器状态（在 UART 接收回调上下文中运行，注意溢出防护） */
static uint8_t s_parse_buf[ARM_MATLAB_DEBUG_RX_SIZE];
static uint16_t s_parse_idx;

/*
 * 命中帧尾后处理一帧：仅接受恰好 ARM_MATLAB_DEBUG_CHANNELS 个通道的帧。
 * memcpy 避免 Cortex 非对齐访问异常，缓冲区可能不是 4 字节对齐。
 * 整帧含 NaN/Inf 则丢弃，不更新时间戳（防止 MATLAB 脚本错误导致关节跑飞）。
 */
static void Arm_MatlabDebug_HandleFrame(uint16_t data_bytes)
{
    if (data_bytes != ARM_MATLAB_DEBUG_CHANNELS * 4U) return;

    float parsed[ARM_MATLAB_DEBUG_CHANNELS];
    /* memcpy 避免 Cortex 非对齐访问；缓冲区可能不是 4 字节对齐。 */
    memcpy(parsed, s_parse_buf, data_bytes);

    for (uint8_t i = 0U; i < ARM_MATLAB_DEBUG_CHANNELS; i++) {
        if (!isfinite(parsed[i])) return; /* 整帧含非法值则丢弃，不更新时间戳 */
    }
    /* 有效帧才更新：写入关节角缓存，刷新时间戳，计数器+1 */
    for (uint8_t i = 0U; i < ARM_MATLAB_DEBUG_CHANNELS; i++) {
        s_rx_joint[i] = parsed[i];
    }
    s_last_frame_tick = HAL_GetTick();
    s_frame_count++;
    s_frame_valid = 1U;
}

/*
 * 单字节喂入解析器：命中 JustFloat 帧尾即切帧。
 * 防丢帧卡死：缓冲区溢出时复位索引（丢弃当前半帧，从下一帧重新同步）。
 */
static void Arm_MatlabDebug_ParseByte(uint8_t byte)
{
    if (s_parse_idx >= ARM_MATLAB_DEBUG_RX_SIZE) {
        s_parse_idx = 0U; /* 溢出复位，防丢帧卡死 */
    }
    s_parse_buf[s_parse_idx++] = byte;

    /* 滑动窗口匹配帧尾：最近 4 字节等于 JF_FOOTER */
    if (s_parse_idx >= 4U &&
        s_parse_buf[s_parse_idx - 4U] == JF_FOOTER[0] &&
        s_parse_buf[s_parse_idx - 3U] == JF_FOOTER[1] &&
        s_parse_buf[s_parse_idx - 2U] == JF_FOOTER[2] &&
        s_parse_buf[s_parse_idx - 1U] == JF_FOOTER[3]) {
        Arm_MatlabDebug_HandleFrame(s_parse_idx - 4U);  /* 帧尾前的数据字节数 */
        s_parse_idx = 0U;  /* 切帧后复位，准备接收下一帧 */
    }
}

/* BSP_UART 接收回调：逐字节喂入解析器。回调上下文，保持轻量。 */
void Arm_MatlabDebug_Resolve(uint8_t *pData, void *device_ptr, uint16_t Size)
{
    (void)device_ptr;
    if (pData == NULL) return;
    for (uint16_t i = 0U; i < Size; i++) {
        Arm_MatlabDebug_ParseByte(pData[i]);
    }
}

/* 链路在线判断：使能 且 解析过至少一帧 且 距上一帧未超时 */
uint8_t Arm_MatlabDebug_IsOnline(void)
{
    if (!Arm_MatlabDebug_Enable || !s_frame_valid) return 0U;
    return (HAL_GetTick() - s_last_frame_tick) <= ARM_MATLAB_DEBUG_TIMEOUT_MS;
}

/*
 * 目标注入：仅在链路在线时覆盖 J2/J4/J5 的目标关节角，其余关节保持 DBUS 逻辑。
 * 调用点在 Arm_UpdateDbusTarget 之后，覆盖后再钳一次限位保证安全。
 */
uint8_t Arm_MatlabDebug_ApplyTarget(float *target, uint8_t count)
{
    if (target == NULL || !Arm_MatlabDebug_IsOnline()) return 0U;
    if (count <= ARM_AXIS_J5) return 0U; /* 需覆盖到 J5 下标 */

    /* 仅注入已联调的三轴，其余关节保持 DBUS 逻辑不动。 */
    target[ARM_AXIS_J2] = s_rx_joint[ARM_AXIS_J2];
    target[ARM_AXIS_J4] = s_rx_joint[ARM_AXIS_J4];
    target[ARM_AXIS_J5] = s_rx_joint[ARM_AXIS_J5];
    return 1U;  /* 返回 1 表示本次有注入，调用侧会再钳一次限位 */
}

#endif /* ARM_MATLAB_DEBUG_ENABLE */
