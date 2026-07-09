#include "Arm_OneClick.h"

#include <math.h>

#include "Arm_JointController.h"
#include "BSP_DWT.h"

#define ARM_ONECLICK_LEAD_IN_S 0.8f  /* 前导段时长(s)：当前位置 -> 轨迹起点 */

/* 引擎运行时状态（模块内私有）。 */
static uint8_t s_phase;              /* Arm_OneClick_Phase_e */
static uint8_t s_traj;               /* 当前 Arm_Traj_e */
static float   s_start_time;         /* 本阶段起始 DWT 秒 */
static float   s_lead_start[ARM_JOINT_COUNT]; /* 前导段起点(触发瞬间位置) */
static float   s_traj_start[ARM_JOINT_COUNT]; /* 轨迹 t=0 的关节角 */

void Arm_OneClick_Init(void)
{
    s_phase = ARM_ONECLICK_PHASE_IDLE;
    s_traj = 0U;
    s_start_time = 0.0f;
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        s_lead_start[i] = 0.0f;
        s_traj_start[i] = 0.0f;
    }
    Arm_Control_Debug.oneclick_active = 0U;
    Arm_Control_Debug.oneclick_id = ARM_ONECLICK_NONE;
    Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_IDLE;
}

/* 把 request 编号映射到轨迹枚举；非法返回 0xFF。 */
static uint8_t Arm_OneClick_ReqToTraj(uint8_t req)
{
    switch (req) {
        case ARM_ONECLICK_UNFOLD: return ARM_TRAJ_UNFOLD;
        case ARM_ONECLICK_FOLD:   return ARM_TRAJ_FOLD;
        case ARM_ONECLICK_SELF_1: return ARM_TRAJ_SELF_1;
        case ARM_ONECLICK_SELF_2: return ARM_TRAJ_SELF_2;
        default:                  return 0xFFU;
    }
}

/* 结束当前动作，回到空闲并清除触发/中止标志。 */
static void Arm_OneClick_Finish(void)
{
    s_phase = ARM_ONECLICK_PHASE_IDLE;
    Arm_Control_Config.oneclick_request = ARM_ONECLICK_NONE;
    Arm_Control_Config.oneclick_abort = 0U;
    Arm_Control_Debug.oneclick_active = 0U;
    Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_IDLE;
}

/* 尝试启动一个新动作：捕获前导起点与轨迹起点，进入前导段。返回是否成功。 */
static uint8_t Arm_OneClick_Start(const float *position, uint8_t count)
{
    uint8_t traj = Arm_OneClick_ReqToTraj(Arm_Control_Config.oneclick_request);

    if (traj == 0xFFU || count < ARM_JOINT_COUNT) return 0U;

    s_traj = traj;
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        s_lead_start[i] = isfinite(position[i]) ? position[i] : 0.0f;
        s_traj_start[i] = Arm_Traj_GetJoint((Arm_Traj_e)traj, i, 0.0f);
    }
    s_start_time = DWT_GetTimeline_s();
    s_phase = ARM_ONECLICK_PHASE_LEAD_IN;
    Arm_Control_Debug.oneclick_active = 1U;
    Arm_Control_Debug.oneclick_id = Arm_Control_Config.oneclick_request;
    Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_LEAD_IN;
    return 1U;
}

uint8_t Arm_OneClick_Update(const float *position, float *target, uint8_t count)
{
    float now, elapsed;

    if (position == NULL || target == NULL || count < ARM_JOINT_COUNT) return 0U;

    /* 中止：任意阶段收到 abort 立即停在当前位置并退出。 */
    if (Arm_Control_Config.oneclick_abort) {
        Arm_OneClick_Finish();
        return 0U;
    }

    /* 空闲：等待新触发。 */
    if (s_phase == ARM_ONECLICK_PHASE_IDLE) {
        if (Arm_Control_Config.oneclick_request == ARM_ONECLICK_NONE) return 0U;
        if (!Arm_OneClick_Start(position, count)) {
            /* 非法编号：清掉请求，避免反复重试。 */
            Arm_Control_Config.oneclick_request = ARM_ONECLICK_NONE;
            return 0U;
        }
    }

    now = DWT_GetTimeline_s();
    elapsed = now - s_start_time;
    if (elapsed < 0.0f) elapsed = 0.0f;  /* DWT 计时器回绕保护 */

    if (s_phase == ARM_ONECLICK_PHASE_LEAD_IN) {
        float ratio = elapsed / ARM_ONECLICK_LEAD_IN_S;
        if (ratio > 1.0f) ratio = 1.0f;
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
            target[i] = s_lead_start[i] +
                        (s_traj_start[i] - s_lead_start[i]) * ratio;
        }
        if (ratio >= 1.0f) {
            s_phase = ARM_ONECLICK_PHASE_PLAYBACK;
            s_start_time = now;
            Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_PLAYBACK;
        }
        return 1U;
    }

    /* ARM_ONECLICK_PHASE_PLAYBACK */
    {
        float total = Arm_Traj_TotalTime((Arm_Traj_e)s_traj);
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
            target[i] = Arm_Traj_GetJoint((Arm_Traj_e)s_traj, i, elapsed);
        }
        if (elapsed >= total) {
            Arm_OneClick_Finish();
        }
    }
    return 1U;
}
