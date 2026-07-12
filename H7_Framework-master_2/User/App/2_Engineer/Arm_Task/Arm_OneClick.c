#include "Arm_OneClick.h"

#include <math.h>
#include <string.h>

#include "Arm_JointController.h"
#include "BSP_DWT.h"
#include "Comm_DualBoard.h"
#include "Picture_Servo.h"

#define ARM_ONECLICK_LEAD_IN_S          0.8f
#define ARM_ONECLICK_STAGE_TRANSITION_S 0.8f
#define ARM_ONECLICK_RETURN_S           1.2f
#define ARM_ONECLICK_MECHANISM_TIMEOUT_S 3.0f
#define ARM_ONECLICK_TRACK_TIMEOUT_S     2.0f
#define ARM_ONECLICK_JOINT_TOLERANCE     0.08f

/* 存取矿复合动作请求对端机构(底盘板)到达的行程常量。集中在此便于随实车标定，
 * 仍经命令通道(Arm_OneClick_Output_t)下发，机械臂代码不直接操作对端电机。 */
#define ARM_ONECLICK_COMPOSITE_LIFT       0          // 图传升降目标(编码器计数)
#define ARM_ONECLICK_COMPOSITE_TRANSVERSE (-600000)  // 图传横移目标(编码器计数)
#define ARM_ONECLICK_COMPOSITE_YAW_US     500U       // 相机舵机 yaw 脉宽(us)
#define ARM_ONECLICK_COMPOSITE_PITCH_US   2300U      // 相机舵机 pitch 脉宽(us)

static uint8_t s_phase;
static uint8_t s_request;
static uint8_t s_first_traj;
static uint8_t s_second_traj;
static uint8_t s_playing_second;
static float s_phase_start;
static float s_origin[ARM_JOINT_COUNT];
static float s_segment_start[ARM_JOINT_COUNT];
static float s_segment_end[ARM_JOINT_COUNT];
static uint8_t s_mechanism_ready;
static uint8_t s_mechanism_completed;
static uint8_t s_mechanism_fault;
static uint8_t s_active_slot;
static uint8_t s_store_occupied_mask;
static Arm_OneClick_Output_t s_output;

static uint8_t Arm_OneClick_IsComposite(uint8_t request)
{
    return request == ARM_ONECLICK_STORE || request == ARM_ONECLICK_TAKE;
}

static uint8_t Arm_OneClick_UsesMechanism(uint8_t request)
{
    return Arm_OneClick_IsComposite(request) || request == ARM_ONECLICK_RESET;
}

static uint8_t Arm_OneClick_MapSingle(uint8_t request)
{
    switch (request) {
        case ARM_ONECLICK_UNFOLD: return ARM_TRAJ_UNFOLD;
        case ARM_ONECLICK_FOLD: return ARM_TRAJ_FOLD;
        case ARM_ONECLICK_SELF_1: return ARM_TRAJ_SELF_1;
        case ARM_ONECLICK_SELF_2: return ARM_TRAJ_SELF_2;
        default: return 0xFFU;
    }
}

static void Arm_OneClick_SetResult(uint8_t result)
{
    Arm_Control_Debug.oneclick_result = result;
}

static void Arm_OneClick_Finish(uint8_t result, uint8_t mechanism_action)
{
    if (result == ARM_ONECLICK_RESULT_DONE) {
        if (s_request == ARM_ONECLICK_STORE) {
            s_store_occupied_mask |= (uint8_t)(1U << s_active_slot);
        } else if (s_request == ARM_ONECLICK_TAKE) {
            s_store_occupied_mask &= (uint8_t)~(1U << s_active_slot);
        }
        Arm_Control_Debug.store_occupied_mask = s_store_occupied_mask;
    }
    s_phase = ARM_ONECLICK_PHASE_IDLE;
    Arm_Control_Config.oneclick_request = ARM_ONECLICK_NONE;
    Arm_Control_Config.oneclick_abort = 0U;
    Arm_Control_Debug.oneclick_active = 0U;
    Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_IDLE;
    Arm_OneClick_SetResult(result);
    memset(&s_output, 0, sizeof(s_output));
    s_output.mechanism_action = mechanism_action;
}

static void Arm_OneClick_BeginPhase(uint8_t phase)
{
    s_phase = phase;
    s_phase_start = DWT_GetTimeline_s();
    Arm_Control_Debug.oneclick_phase = phase;
}

static void Arm_OneClick_Capture(const float *source, float *destination)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        destination[i] = isfinite(source[i]) ? source[i] : 0.0f;
    }
}

static void Arm_OneClick_LoadTrajectoryStart(uint8_t traj, float *destination)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        destination[i] = Arm_Traj_GetJoint((Arm_Traj_e)traj, i, 0.0f);
    }
}

static void Arm_OneClick_Interpolate(const float *start,
                                     const float *end,
                                     float ratio,
                                     float *target)
{
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        target[i] = start[i] + (end[i] - start[i]) * ratio;
    }
}

static uint8_t Arm_OneClick_TargetReached(const float *position, const float *target)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
        if (fabsf(position[i] - target[i]) > ARM_ONECLICK_JOINT_TOLERANCE) return 0U;
    }
    return 1U;
}

static void Arm_OneClick_PrepareComposite(uint8_t request)
{
    s_first_traj = request == ARM_ONECLICK_STORE ? ARM_TRAJ_STORE_1 : ARM_TRAJ_TAKE_1;
    s_second_traj = request == ARM_ONECLICK_STORE ? ARM_TRAJ_STORE_2 : ARM_TRAJ_TAKE_2;

    s_output.active = 1U;
    s_output.picture_override = 1U;
    s_output.clamp_override = 1U;
    s_output.clamp_close = request == ARM_ONECLICK_STORE ? 1U : 0U;
    s_output.mechanism_action = DUALBOARD_ACTION_EXECUTE;
    s_output.store_slot = s_active_slot;
    s_output.picture_lift = ARM_ONECLICK_COMPOSITE_LIFT;
    s_output.picture_transverse = ARM_ONECLICK_COMPOSITE_TRANSVERSE;
    s_output.yaw_us = ARM_ONECLICK_COMPOSITE_YAW_US;
    s_output.pitch_us = ARM_ONECLICK_COMPOSITE_PITCH_US;
}

static uint8_t Arm_OneClick_Start(const float *position, uint8_t count)
{
    uint8_t request = Arm_Control_Config.oneclick_request;
    uint8_t traj;

    if (count < ARM_JOINT_COUNT) return 0U;
    if (request == ARM_ONECLICK_RESET) {
        if (!s_mechanism_ready) return 0U;
        s_request = request;
        Arm_OneClick_Capture(position, s_origin);
        memset(&s_output, 0, sizeof(s_output));
        s_output.active = 1U;
        s_output.picture_override = 1U;
        s_output.mechanism_action = DUALBOARD_ACTION_HOME_PICTURE;
        s_output.store_slot = Arm_Control_Config.oneclick_store_slot;
        Arm_Control_Debug.oneclick_active = 1U;
        Arm_Control_Debug.oneclick_id = request;
        Arm_OneClick_SetResult(ARM_ONECLICK_RESULT_RUNNING);
        Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_RESET_WAIT);
        return 1U;
    }
    if (Arm_OneClick_IsComposite(request)) {
        if (!s_mechanism_ready) {
            return 0U;
        }
        if (request == ARM_ONECLICK_STORE) {
            for (s_active_slot = 0U; s_active_slot < 4U; s_active_slot++) {
                if ((s_store_occupied_mask & (uint8_t)(1U << s_active_slot)) == 0U) break;
            }
        } else {
            s_active_slot = 4U;
            while (s_active_slot > 0U) {
                s_active_slot--;
                if (s_store_occupied_mask & (uint8_t)(1U << s_active_slot)) break;
            }
            if (s_store_occupied_mask == 0U) s_active_slot = 4U;
        }
        if (s_active_slot >= 4U) {
            // 满库(存)/空库(取)：拒绝动作。用 NO_SLOT 结果，Finish 仅在 DONE 时改位图，
            // 故此处不会污染占用位图，也不会误报成功。s_active_slot 复位避免越界残留。
            s_active_slot = 0U;
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_NO_SLOT, DUALBOARD_ACTION_HOLD);
            return 0U;
        }
        Arm_Control_Config.oneclick_store_slot = s_active_slot;
        Arm_OneClick_PrepareComposite(request);
        traj = s_first_traj;
    } else {
        traj = Arm_OneClick_MapSingle(request);
        if (traj == 0xFFU) return 0U;
        memset(&s_output, 0, sizeof(s_output));
        s_output.active = 1U;
        s_first_traj = traj;
        s_second_traj = 0xFFU;
    }

    s_request = request;
    s_playing_second = 0U;
    Arm_OneClick_Capture(position, s_origin);
    Arm_OneClick_Capture(position, s_segment_start);
    Arm_OneClick_LoadTrajectoryStart(traj, s_segment_end);
    Arm_Control_Debug.oneclick_active = 1U;
    Arm_Control_Debug.oneclick_id = request;
    Arm_OneClick_SetResult(ARM_ONECLICK_RESULT_RUNNING);
    Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_LEAD_IN);
    return 1U;
}

void Arm_OneClick_Init(void)
{
    s_phase = ARM_ONECLICK_PHASE_IDLE;
    s_request = ARM_ONECLICK_NONE;
    s_first_traj = 0U;
    s_second_traj = 0xFFU;
    s_playing_second = 0U;
    s_phase_start = 0.0f;
    s_mechanism_ready = 0U;
    s_mechanism_completed = 0U;
    s_mechanism_fault = 0U;
    s_active_slot = 0U;
    s_store_occupied_mask = 0U;
    memset(s_origin, 0, sizeof(s_origin));
    memset(s_segment_start, 0, sizeof(s_segment_start));
    memset(s_segment_end, 0, sizeof(s_segment_end));
    memset(&s_output, 0, sizeof(s_output));
    Arm_Control_Debug.oneclick_active = 0U;
    Arm_Control_Debug.oneclick_id = ARM_ONECLICK_NONE;
    Arm_Control_Debug.oneclick_phase = ARM_ONECLICK_PHASE_IDLE;
    Arm_OneClick_SetResult(ARM_ONECLICK_RESULT_IDLE);
    Arm_Control_Debug.store_occupied_mask = 0U;
}

void Arm_OneClick_SetMechanismState(uint8_t ready, uint8_t completed, uint8_t fault)
{
    s_mechanism_ready = ready ? 1U : 0U;
    s_mechanism_completed = completed ? 1U : 0U;
    s_mechanism_fault = fault ? 1U : 0U;
}

const Arm_OneClick_Output_t *Arm_OneClick_GetOutput(void)
{
    return &s_output;
}

uint8_t Arm_OneClick_Update(const float *position, float *target, uint8_t count)
{
    float now;
    float elapsed;

    if (position == NULL || target == NULL || count < ARM_JOINT_COUNT) return 0U;

    if (Arm_Control_Config.oneclick_abort ||
        (Arm_OneClick_UsesMechanism(s_request) && s_phase != ARM_ONECLICK_PHASE_IDLE &&
         (!s_mechanism_ready || s_mechanism_fault))) {
        Arm_OneClick_Capture(position, target);
        Arm_OneClick_Finish(s_mechanism_fault ? ARM_ONECLICK_RESULT_PEER_FAULT :
                                               ARM_ONECLICK_RESULT_ABORTED,
                            DUALBOARD_ACTION_STOP_ALL);
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_IDLE) {
        if (Arm_Control_Config.oneclick_request == ARM_ONECLICK_NONE) return 0U;
        if (!Arm_OneClick_Start(position, count)) return 0U;
    }

    now = DWT_GetTimeline_s();
    elapsed = now - s_phase_start;
    if (elapsed < 0.0f) elapsed = 0.0f;

    if (s_phase == ARM_ONECLICK_PHASE_RESET_WAIT) {
        Arm_OneClick_Capture(position, target);
        if (s_mechanism_completed) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_DONE, DUALBOARD_ACTION_HOLD);
        } else if (elapsed >= ARM_ONECLICK_MECHANISM_TIMEOUT_S) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
        }
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_LEAD_IN) {
        Arm_OneClick_Interpolate(s_segment_start, s_segment_end,
                                 elapsed / ARM_ONECLICK_LEAD_IN_S, target);
        if (elapsed >= ARM_ONECLICK_LEAD_IN_S &&
            Arm_OneClick_TargetReached(position, s_segment_end)) {
            Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_PLAYBACK);
        } else if (elapsed >= ARM_ONECLICK_LEAD_IN_S + ARM_ONECLICK_TRACK_TIMEOUT_S) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
        }
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_PLAYBACK) {
        float total = Arm_Traj_TotalTime((Arm_Traj_e)s_first_traj);
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++) {
            target[i] = Arm_Traj_GetJoint((Arm_Traj_e)s_first_traj, i, elapsed);
        }
        if (elapsed < total) return 1U;
        if (!Arm_OneClick_TargetReached(position, target)) {
            if (elapsed >= total + ARM_ONECLICK_TRACK_TIMEOUT_S) {
                Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
            }
            return 1U;
        }

        if (!Arm_OneClick_IsComposite(s_request)) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_DONE, DUALBOARD_ACTION_HOLD);
            return 1U;
        }
        if (s_playing_second) {
            Arm_OneClick_Capture(position, s_segment_start);
            Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_RETURN);
            return 1U;
        }
        Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_WAIT_MECHANISM);
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_WAIT_MECHANISM) {
        if (!s_mechanism_completed) {
            if (elapsed >= ARM_ONECLICK_MECHANISM_TIMEOUT_S) {
                Arm_OneClick_Capture(position, target);
                Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
            }
            return 1U;
        }

        s_output.clamp_close = s_request == ARM_ONECLICK_STORE ? 0U : 1U;
        s_output.mechanism_action = DUALBOARD_ACTION_HOLD;
        Arm_OneClick_Capture(position, s_segment_start);
        Arm_OneClick_LoadTrajectoryStart(s_second_traj, s_segment_end);
        Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_STAGE_TRANSITION);
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_STAGE_TRANSITION) {
        Arm_OneClick_Interpolate(s_segment_start, s_segment_end,
                                 elapsed / ARM_ONECLICK_STAGE_TRANSITION_S, target);
        if (elapsed >= ARM_ONECLICK_STAGE_TRANSITION_S &&
            Arm_OneClick_TargetReached(position, s_segment_end)) {
            s_first_traj = s_second_traj;
            s_second_traj = 0xFFU;
            s_playing_second = 1U;
            Arm_OneClick_BeginPhase(ARM_ONECLICK_PHASE_PLAYBACK);
        } else if (elapsed >= ARM_ONECLICK_STAGE_TRANSITION_S + ARM_ONECLICK_TRACK_TIMEOUT_S) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
        }
        return 1U;
    }

    if (s_phase == ARM_ONECLICK_PHASE_RETURN) {
        Arm_OneClick_Interpolate(s_segment_start, s_origin,
                                 elapsed / ARM_ONECLICK_RETURN_S, target);
        s_output.picture_lift = 1000000;
        s_output.yaw_us = 1800U;
        if (elapsed >= ARM_ONECLICK_RETURN_S && Arm_OneClick_TargetReached(position, s_origin)) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_DONE, DUALBOARD_ACTION_HOLD);
        } else if (elapsed >= ARM_ONECLICK_RETURN_S + ARM_ONECLICK_TRACK_TIMEOUT_S) {
            Arm_OneClick_Finish(ARM_ONECLICK_RESULT_TIMEOUT, DUALBOARD_ACTION_STOP_ALL);
        }
        return 1U;
    }

    Arm_OneClick_Capture(position, target);
    Arm_OneClick_Finish(ARM_ONECLICK_RESULT_ABORTED, DUALBOARD_ACTION_STOP_ALL);
    return 1U;
}
