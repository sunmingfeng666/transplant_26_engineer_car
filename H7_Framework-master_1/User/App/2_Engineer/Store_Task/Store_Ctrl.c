#include "Store_Ctrl.h"

#include <math.h>

#include "Comm_DualBoard.h"
#include "DM_Motor.h"
#include "fdcan.h"

#define STORE_MOTOR_ID             0x03U
#define STORE_POSITION_PER_SLOT    1.963495f
#define STORE_SPEED_RAD_S          2.61799f
#define STORE_POSITION_TOLERANCE   0.05f
#define STORE_SPEED_TOLERANCE      0.10f
#define STORE_ACTION_TIMEOUT_MS    5000U
#define STORE_ENABLE_TIMEOUT_MS    500U

typedef struct {
    Engineer_Store_State_e state;
    uint8_t enabled;
    uint8_t slot;
    uint8_t action_seq;
    uint8_t done;
    uint8_t fault;
    float target;
    uint32_t action_start_ms;
    uint32_t enable_start_ms;
} Engineer_Store_Ctrl_t;

static Engineer_Store_Ctrl_t store_ctrl;

static void Store_Disable(void)
{
    if (store_ctrl.enabled) {
        Motor_Mode(&hfdcan2, STORE_MOTOR_ID, POS_MODE, DM_CMD_RESET_MODE);
        store_ctrl.enabled = 0U;
    }
}

static void Store_Enable(void)
{
    Motor_Mode(&hfdcan2, STORE_MOTOR_ID, POS_MODE, DM_CMD_CLEAR_ERROR);
    Motor_Mode(&hfdcan2, STORE_MOTOR_ID, POS_MODE, DM_CMD_MOTOR_MODE);
    store_ctrl.enabled = 1U;
}

uint8_t Engineer_Store_Init(void)
{
    store_ctrl.state = ENGINEER_STORE_WAIT_FEEDBACK;
    store_ctrl.enabled = 0U;
    store_ctrl.slot = 0U;
    store_ctrl.action_seq = 0xFFU;
    store_ctrl.done = 0U;
    store_ctrl.fault = 0U;
    store_ctrl.target = 0.0f;
    store_ctrl.action_start_ms = 0U;
    store_ctrl.enable_start_ms = 0U;
    return 1U;
}

void Engineer_Store_Task(const Store_Motor_Group_t *motors)
{
    // 全局前置条件1：本机电机离线 -> 安全失能，回等待反馈态
    if (motors == NULL || !motors->DM4310_Store.offline.is_online) {
        Store_Disable();
        store_ctrl.state = ENGINEER_STORE_WAIT_FEEDBACK;
        store_ctrl.done = 0U;
        return;
    }

    // 全局前置条件2：对端失联/安全模式/急停 -> 安全失能，停机
    if (!DualBoard_Picture_Is_Online() || !DualBoard_Chassis_Is_Online() ||
        B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE ||
        B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_STOP_ALL) {
        Store_Disable();
        store_ctrl.state = ENGINEER_STORE_STOPPED;
        store_ctrl.done = 0U;
        return;
    }

    // 人工中止/清故障：仅故障态响应，统一回到使能态重新上电
    if (store_ctrl.fault && B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_CLEAR_FAULT) {
        store_ctrl.fault = 0U;
        store_ctrl.state = ENGINEER_STORE_ENABLING;
    }

    switch (store_ctrl.state) {
    case ENGINEER_STORE_WAIT_FEEDBACK:
        // 进入条件：刚从离线恢复。转使能态。
        store_ctrl.state = ENGINEER_STORE_ENABLING;
        break;

    case ENGINEER_STORE_ENABLING:
        // 进入即发使能帧并记时，以电机反馈 state==1 作为使能确认。
        if (!store_ctrl.enabled) {
            Store_Enable();
            store_ctrl.enable_start_ms = HAL_GetTick();
            store_ctrl.target = motors->DM4310_Store.pos; // 保位，防突跳
            store_ctrl.done = 0U;
        }
        if (motors->DM4310_Store.state == 1) {
            store_ctrl.done = 1U;                          // 完成条件：使能确认
            store_ctrl.state = ENGINEER_STORE_HOLDING;
        } else if ((HAL_GetTick() - store_ctrl.enable_start_ms) > STORE_ENABLE_TIMEOUT_MS) {
            store_ctrl.fault = 1U;                         // 超时 -> 故障
            store_ctrl.state = ENGINEER_STORE_FAULT;
        }
        break;

    case ENGINEER_STORE_HOLDING:
        // 保位待命；收到新执行动作(序号变化)转入移动。
        store_ctrl.done = 1U;
        if (B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_EXECUTE &&
            B2B_Picture_Cmd.action_seq != store_ctrl.action_seq) {
            store_ctrl.action_seq = B2B_Picture_Cmd.action_seq;
            store_ctrl.slot = (B2B_Picture_Cmd.store_slot <= 3U) ? B2B_Picture_Cmd.store_slot : 3U;
            store_ctrl.target = (float)store_ctrl.slot * STORE_POSITION_PER_SLOT;
            store_ctrl.action_start_ms = HAL_GetTick();
            store_ctrl.done = 0U;
            store_ctrl.state = ENGINEER_STORE_MOVING;
        }
        break;

    case ENGINEER_STORE_MOVING: {
        // 完成条件：位置到位且速度收敛 -> 保位；超时 -> 故障。
        const float position_error = fabsf(motors->DM4310_Store.pos - store_ctrl.target);
        const float speed = fabsf(motors->DM4310_Store.vel);
        if (position_error <= STORE_POSITION_TOLERANCE && speed <= STORE_SPEED_TOLERANCE) {
            store_ctrl.done = 1U;
            store_ctrl.state = ENGINEER_STORE_HOLDING;
        } else if ((HAL_GetTick() - store_ctrl.action_start_ms) > STORE_ACTION_TIMEOUT_MS) {
            store_ctrl.fault = 1U;
            store_ctrl.state = ENGINEER_STORE_FAULT;
        }
        break;
    }

    case ENGINEER_STORE_STOPPED:
        // 停机恢复：已过前置条件(对端在线/非安全/非急停) -> 回使能态。
        store_ctrl.state = ENGINEER_STORE_ENABLING;
        break;

    case ENGINEER_STORE_FAULT:
        // 故障锁存，安全失能；仅 CLEAR_FAULT 可退出(见前置)。
        Store_Disable();
        return;

    default:
        Store_Disable();
        return;
    }

    // 安全输出：仅在已使能且非故障态发送位置速度指令。
    if (store_ctrl.enabled && store_ctrl.state != ENGINEER_STORE_FAULT) {
        Pos_Speed_Ctrl(&hfdcan2, STORE_MOTOR_ID, store_ctrl.target, STORE_SPEED_RAD_S);
    }
}

Engineer_Store_Status_t Engineer_Store_Get_Status(void)
{
    Engineer_Store_Status_t status = {
        .state = store_ctrl.state,
        .online = store_motors.DM4310_Store.offline.is_online ? 1U : 0U,
        .done = store_ctrl.done,
        .fault = store_ctrl.fault,
        .slot = store_ctrl.slot,
        .position = store_motors.DM4310_Store.pos,
        .target = store_ctrl.target,
    };
    return status;
}
