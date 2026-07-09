#ifndef H7_FRAMEWORK_ARM_MATLAB_DEBUG_H
#define H7_FRAMEWORK_ARM_MATLAB_DEBUG_H

#include <stdint.h>

#include "Arm_JointController.h"

/*
 * 机械臂 MATLAB 联调模块（仅测试用）
 *
 * 【编译总开关】ARM_MATLAB_DEBUG_ENABLE
 *   0 = 整套联调从固件中消失，Engineer_Arm_Task 走纯 DBUS 逻辑，
 *       UART7 接收不注册。UART7 发送（VOFA 波形/遥测）不受影响，照常工作。
 *       ➤ 比赛固件用此配置：零侵入、零占用。
 *   1 = 编入 UART7 接收解析 + 目标注入。
 *       ➤ 测试固件用此配置：联调 J2/J4/J5 三轴。
 *
 * 【架构说明】
 *   - UART7 发送是 board2 的公用上位机口（见 All_Task.c），不归本模块管。
 *   - 本模块只负责”接收 MATLAB 下发的关节角”和”把目标注入 Arm_Ctrl”。
 *   - 通信协议：VOFA+ JustFloat，115200 波特率。
 *     下发：MATLAB → MCU，single([J1..J6, Inf])，当前仅采用 J2/J4/J5。
 *     回传：MCU → MATLAB，20 通道，ch0-5 = J1..J6 实际位置供 3D 显示。
 */
#ifndef ARM_MATLAB_DEBUG_ENABLE
#define ARM_MATLAB_DEBUG_ENABLE 1
#endif

/* MATLAB 下发帧通道数：[J1..J6] 六个弧度值 + Inf 帧尾（Inf 的 single 小端即 JustFloat 帧尾）。 */
#define ARM_MATLAB_DEBUG_CHANNELS 6U

/* 链路新鲜度超时(ms)：超过此时长没收到新帧即判定链路掉线，自动回退 DBUS 控制。 */
#define ARM_MATLAB_DEBUG_TIMEOUT_MS 200U

#if ARM_MATLAB_DEBUG_ENABLE

/* UART7 接收缓冲区大小：6 通道 * 4 字节 + 4 字节帧尾，留裕量。 */
#define ARM_MATLAB_DEBUG_RX_SIZE 64U

/* 运行时使能开关：默认 0。分阶段上车时先设为 1 再逐轴验证方向/零位。 */
extern volatile uint8_t Arm_MatlabDebug_Enable;

/* BSP_UART 接收回调（挂到 UART7 的 UART_RX_NODE）。逐字节解析 JustFloat 帧。 */
void Arm_MatlabDebug_Resolve(uint8_t *pData, void *device_ptr, uint16_t Size);

/*
 * 若运行时使能且链路新鲜，则把 MATLAB 下发的 J2/J4/J5 目标写入 target[]。
 * 其余关节以及未使能/掉线时都不修改 target[]（自动回退 DBUS 逻辑）。
 *
 * @param target 目标关节角数组(rad)，长度至少 ARM_JOINT_COUNT
 * @param count  数组长度
 * @return 1 = 本次有注入，0 = 未注入（未使能/掉线/参数异常）
 */
uint8_t Arm_MatlabDebug_ApplyTarget(float *target, uint8_t count);

/* 链路是否在线（使能 且 距上一帧未超时）。用于遥测上报和状态观察。 */
uint8_t Arm_MatlabDebug_IsOnline(void);

#endif /* ARM_MATLAB_DEBUG_ENABLE */

#endif /* H7_FRAMEWORK_ARM_MATLAB_DEBUG_H */
