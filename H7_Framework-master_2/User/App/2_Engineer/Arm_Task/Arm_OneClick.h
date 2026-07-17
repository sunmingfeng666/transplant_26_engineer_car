#ifndef H7_FRAMEWORK_ARM_ONECLICK_H
#define H7_FRAMEWORK_ARM_ONECLICK_H

#include <stdint.h>

#include "Arm_Trajectory.h"

typedef enum {
    ARM_ONECLICK_NONE = 0,
    ARM_ONECLICK_UNFOLD = 1,
    ARM_ONECLICK_FOLD = 2,
    ARM_ONECLICK_SELF_1 = 3,
    ARM_ONECLICK_SELF_2 = 4,
    ARM_ONECLICK_STORE = 5,
    ARM_ONECLICK_TAKE = 6,
    ARM_ONECLICK_RESET = 7,
} Arm_OneClick_Req_e;

typedef enum {
    ARM_ONECLICK_PHASE_IDLE = 0,
    ARM_ONECLICK_PHASE_LEAD_IN,
    ARM_ONECLICK_PHASE_PLAYBACK,
    ARM_ONECLICK_PHASE_WAIT_MECHANISM,
    ARM_ONECLICK_PHASE_STAGE_TRANSITION,
    ARM_ONECLICK_PHASE_RETURN,
    ARM_ONECLICK_PHASE_RESET_WAIT,
} Arm_OneClick_Phase_e;

typedef enum {
    ARM_ONECLICK_RESULT_IDLE = 0,
    ARM_ONECLICK_RESULT_RUNNING,
    ARM_ONECLICK_RESULT_DONE,
    ARM_ONECLICK_RESULT_ABORTED,
    ARM_ONECLICK_RESULT_TIMEOUT,
    ARM_ONECLICK_RESULT_PEER_FAULT,
    ARM_ONECLICK_RESULT_NO_SLOT,   /* 存矿满库/取矿空库：拒绝动作，不更新占用位图 */
} Arm_OneClick_Result_e;

/* 动作协调器给命令层和末端夹爪控制使用的只读输出。 */
typedef struct {
    uint8_t active;
    uint8_t picture_override;
    uint8_t clamp_override;
    uint8_t clamp_close;
    uint8_t mechanism_action;
    uint8_t store_slot;
    int32_t picture_lift;
    int32_t picture_transverse;
    uint16_t yaw_us;
    uint16_t pitch_us;
} Arm_OneClick_Output_t;

void Arm_OneClick_Init(void);
uint8_t Arm_OneClick_Update(const float *position, float *target, uint8_t count);
void Arm_OneClick_SetMechanismState(uint8_t ready, uint8_t completed, uint8_t fault);
const Arm_OneClick_Output_t *Arm_OneClick_GetOutput(void);

#endif
