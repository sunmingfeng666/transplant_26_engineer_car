#include "Engineer_Feedback.h"

#include "Chassis_Ctrl.h"
#include "Comm_DualBoard.h"
#include "Picture_Ctrl.h"
#include "Store_Ctrl.h"
#include "usart.h"

#define ENGINEER_FEEDBACK_DIVIDER 10U

static uint8_t feedback_divider;

void Engineer_Feedback_Init(void)
{
    feedback_divider = 0U;
}

void Engineer_Feedback_Task(const Chassis_Motor_Group_t *chassis,
                            const Picture_Motor_Group_t *picture,
                            const Store_Motor_Group_t *store)
{
    B2B_Engineer_Feedback_t feedback = {0};
    Engineer_Picture_Status_t picture_status;
    Engineer_Store_Status_t store_status;
    uint8_t all_done;

    if (++feedback_divider < ENGINEER_FEEDBACK_DIVIDER) return;
    feedback_divider = 0U;
    if (chassis == NULL || picture == NULL || store == NULL) return;

    picture_status = Engineer_Picture_Get_Status();
    store_status = Engineer_Store_Get_Status();

    for (uint8_t i = 0U; i < 4U; i++) {
        if (chassis->DJI_3508_Chassis[i].offline.is_online) {
            feedback.chassis_online_bits |= (uint8_t)(1U << i);
        }
    }
    if (picture->DJI_2006_Lift.offline.is_online) {
        feedback.mechanism_online_bits |= DUALBOARD_MECHANISM_LIFT_ONLINE;
    }
    if (picture->DJI_2006_Transverse.offline.is_online) {
        feedback.mechanism_online_bits |= DUALBOARD_MECHANISM_TRANSVERSE_ONLINE;
    }
    if (store_status.online) {
        feedback.mechanism_online_bits |= DUALBOARD_MECHANISM_STORE_ONLINE;
    }

    if (picture_status.lift_bottom) feedback.limit_bits |= DUALBOARD_LIMIT_LIFT_BOTTOM;
    if (picture_status.transverse_zero) feedback.limit_bits |= DUALBOARD_LIMIT_TRANSVERSE_ZERO;
    if (picture_status.lift_done) feedback.action_bits |= DUALBOARD_ACTION_LIFT_DONE;
    if (picture_status.transverse_done) feedback.action_bits |= DUALBOARD_ACTION_TRANSVERSE_DONE;
    if (store_status.done) feedback.action_bits |= DUALBOARD_ACTION_STORE_DONE;
    if (picture_status.homing_active) feedback.action_bits |= DUALBOARD_ACTION_HOMING_ACTIVE;
    if (picture_status.homing_done) feedback.action_bits |= DUALBOARD_ACTION_HOMING_DONE;

    if (picture_status.fault || store_status.fault) {
        feedback.action_bits |= DUALBOARD_ACTION_FAULT;
        feedback.status = DUALBOARD_FB_ERROR;
        feedback.error_code = picture_status.fault ? 1 : 2;
    } else if (!DualBoard_Chassis_Is_Online() || !DualBoard_Picture_Is_Online()) {
        feedback.status = DUALBOARD_FB_LOST;
    } else if (B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE) {
        feedback.status = DUALBOARD_FB_SAFE;
    } else {
        feedback.status = DUALBOARD_FB_RUN;
    }

    all_done = (picture_status.lift_done && picture_status.transverse_done && store_status.done) ? 1U : 0U;
    if (B2B_Picture_Cmd.mechanism_action == DUALBOARD_ACTION_HOME_PICTURE) {
        all_done = picture_status.homing_done;
    }
    if (all_done) feedback.completed_action_seq = B2B_Picture_Cmd.action_seq;

    feedback.picture_lift_pos = picture_status.lift_position;
    feedback.picture_transverse_pos = picture_status.transverse_position;
    feedback.store_pos_mrad = (int16_t)(store_status.position * 1000.0f);

    // 底盘反馈帧：Chassis_Ctrl 只记录状态，这里统一上报。
    // 先发底盘帧(阻塞轮询，发完 gState 复位)，再发整车帧(中断)，避免同一 UART 上 IT 冲突。
    Engineer_Chassis_Feedback_t chassis_fb = Engineer_Chassis_Get_Feedback();
    (void)DualBoard_Send_Chassis_Feedback(&huart10,
                                          chassis_fb.status,
                                          chassis_fb.motor_online_bits,
                                          chassis_fb.error_code);
    (void)DualBoard_Send_Engineer_Feedback(&huart10, &feedback);
}
