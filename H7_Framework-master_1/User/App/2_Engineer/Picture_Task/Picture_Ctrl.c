#include "Picture_Ctrl.h"
#include "Classic_Control.h"
#include "Comm_DualBoard.h"
#include "DJI_Motor.h"
#include "fdcan.h"
#include "main.h"

#define PICTURE_LIFT_MIN 0
#define PICTURE_LIFT_MAX 1025000
#define PICTURE_TRANSVERSE_MIN (-624358)
#define PICTURE_TRANSVERSE_MAX 0
#define PICTURE_SPEED_LIMIT 7000.0f
#define PICTURE_MAX_CURRENT 10000.0f
#define PICTURE_PID_I_LIMIT 2000.0f
#define PICTURE_LIFT_DIR 1.0f
#define PICTURE_TRANSVERSE_DIR 1.0f

typedef enum {
    PICTURE_AXIS_LIFT = 0,
    PICTURE_AXIS_TRANSVERSE = 1,
    PICTURE_AXIS_NUM = 2
} Picture_Axis_e;

typedef struct {
    PID_t pos_pid[PICTURE_AXIS_NUM];
    PID_t speed_pid[PICTURE_AXIS_NUM];
    int32_t target[PICTURE_AXIS_NUM];
    int32_t zero_offset[PICTURE_AXIS_NUM];
    uint8_t bottom_switch_last;
    uint8_t transverse_zero_switch_last;
    uint8_t is_init;
} Engineer_Picture_Ctrl_t;

static Engineer_Picture_Ctrl_t picture_ctrl = {0};

static void Picture_Clear_Output(void);
static int32_t Picture_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value);
static float Picture_Limit_Float(float value, float min_value, float max_value);
static uint8_t Picture_Read_Lift_Bottom(void);
static uint8_t Picture_Read_Transverse_Zero(void);
static void Picture_Update_Switch_Zero(const Picture_Motor_Group_t *p_motor);
static int16_t Picture_Axis_Calc(Picture_Axis_e axis,
                                 const DJI_MOTOR_DATA_Typedef *motor,
                                 int32_t target,
                                 float dir);

uint8_t Engineer_Picture_Init(void)
{
    float pos_pid_param[3] = {0.006f, 0.0f, 0.0f};
    float speed_pid_param[3] = {2.0f, 0.05f, 0.0f};

    for (uint8_t i = 0; i < PICTURE_AXIS_NUM; i++) {
        PID_Init(&picture_ctrl.pos_pid[i],
                 PICTURE_SPEED_LIMIT,
                 0.0f,
                 pos_pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 NONE);
        PID_Init(&picture_ctrl.speed_pid[i],
                 PICTURE_MAX_CURRENT,
                 PICTURE_PID_I_LIMIT,
                 speed_pid_param,
                 0.0f, 0.0f,
                 0.0f, 0.0f,
                 0,
                 Integral_Limit);
    }

    picture_ctrl.is_init = 1U;
    return 1U;
}

void Engineer_Picture_Task(const Picture_Motor_Group_t *p_motor)
{
    if (!picture_ctrl.is_init) {
        (void)Engineer_Picture_Init();
    }

    if (p_motor == NULL ||
        !DualBoard_Picture_Is_Online() ||
        !DualBoard_Chassis_Is_Online() ||
        B2B_Chassis_Cmd.mode == DUALBOARD_CHASSIS_SAFE) {
        Picture_Clear_Output();
        return;
    }

    picture_ctrl.target[PICTURE_AXIS_LIFT] =
        Picture_Limit_Int32(B2B_Picture_Cmd.picture_lift, PICTURE_LIFT_MIN, PICTURE_LIFT_MAX);
    picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] =
        Picture_Limit_Int32(B2B_Picture_Cmd.picture_transverse, PICTURE_TRANSVERSE_MIN, PICTURE_TRANSVERSE_MAX);

    Picture_Update_Switch_Zero(p_motor);

    int16_t lift_out = 0;
    int16_t transverse_out = 0;

    if (p_motor->DJI_2006_Lift.offline.is_online) {
        lift_out = Picture_Axis_Calc(PICTURE_AXIS_LIFT,
                                     &p_motor->DJI_2006_Lift,
                                     picture_ctrl.target[PICTURE_AXIS_LIFT],
                                     PICTURE_LIFT_DIR);
    } else {
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_LIFT]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_LIFT]);
    }

    if (p_motor->DJI_2006_Transverse.offline.is_online) {
        transverse_out = Picture_Axis_Calc(PICTURE_AXIS_TRANSVERSE,
                                           &p_motor->DJI_2006_Transverse,
                                           picture_ctrl.target[PICTURE_AXIS_TRANSVERSE],
                                           PICTURE_TRANSVERSE_DIR);
    } else {
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_TRANSVERSE]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_TRANSVERSE]);
    }

    if (Picture_Read_Lift_Bottom() && lift_out < 0) {
        lift_out = 0;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_LIFT]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_LIFT]);
    }

    if (Picture_Read_Transverse_Zero() && transverse_out > 0) {
        transverse_out = 0;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_TRANSVERSE]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_TRANSVERSE]);
    }

    DJI_Motor_Send(&hfdcan3, 0x200, 0, 0, lift_out, transverse_out);
}

static void Picture_Clear_Output(void)
{
    for (uint8_t i = 0; i < PICTURE_AXIS_NUM; i++) {
        PID_Clear(&picture_ctrl.pos_pid[i]);
        PID_Clear(&picture_ctrl.speed_pid[i]);
        picture_ctrl.target[i] = 0;
    }
    DJI_Motor_Send(&hfdcan3, 0x200, 0, 0, 0, 0);
}

static int16_t Picture_Axis_Calc(Picture_Axis_e axis,
                                 const DJI_MOTOR_DATA_Typedef *motor,
                                 int32_t target,
                                 float dir)
{
    float position = ((float)motor->Angle_Infinite - (float)picture_ctrl.zero_offset[axis]) * dir;
    float speed = (float)motor->Speed_now * dir;

    float target_speed = PID_Calculate(&picture_ctrl.pos_pid[axis], position, (float)target);
    target_speed = Picture_Limit_Float(target_speed, -PICTURE_SPEED_LIMIT, PICTURE_SPEED_LIMIT);

    float current = PID_Calculate(&picture_ctrl.speed_pid[axis], speed, target_speed);
    current = Picture_Limit_Float(current, -PICTURE_MAX_CURRENT, PICTURE_MAX_CURRENT);
    current *= dir;

    return (int16_t)current;
}

static void Picture_Update_Switch_Zero(const Picture_Motor_Group_t *p_motor)
{
    uint8_t lift_bottom = Picture_Read_Lift_Bottom();
    uint8_t transverse_zero = Picture_Read_Transverse_Zero();

    if (lift_bottom && !picture_ctrl.bottom_switch_last) {
        picture_ctrl.zero_offset[PICTURE_AXIS_LIFT] = p_motor->DJI_2006_Lift.Angle_Infinite;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_LIFT]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_LIFT]);
    }
    if (lift_bottom && picture_ctrl.target[PICTURE_AXIS_LIFT] <= PICTURE_LIFT_MIN) {
        picture_ctrl.target[PICTURE_AXIS_LIFT] = PICTURE_LIFT_MIN;
    }

    if (transverse_zero && !picture_ctrl.transverse_zero_switch_last) {
        picture_ctrl.zero_offset[PICTURE_AXIS_TRANSVERSE] = p_motor->DJI_2006_Transverse.Angle_Infinite;
        PID_Clear(&picture_ctrl.pos_pid[PICTURE_AXIS_TRANSVERSE]);
        PID_Clear(&picture_ctrl.speed_pid[PICTURE_AXIS_TRANSVERSE]);
    }
    if (transverse_zero && picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] >= PICTURE_TRANSVERSE_MAX) {
        picture_ctrl.target[PICTURE_AXIS_TRANSVERSE] = PICTURE_TRANSVERSE_MAX;
    }

    picture_ctrl.bottom_switch_last = lift_bottom;
    picture_ctrl.transverse_zero_switch_last = transverse_zero;
}

static uint8_t Picture_Read_Lift_Bottom(void)
{
#if defined(PICTURE_LIFT_BOTTOM_Pin) && defined(PICTURE_LIFT_BOTTOM_GPIO_Port)
    return (HAL_GPIO_ReadPin(PICTURE_LIFT_BOTTOM_GPIO_Port, PICTURE_LIFT_BOTTOM_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return 0U;
#endif
}

static uint8_t Picture_Read_Transverse_Zero(void)
{
#if defined(PICTURE_TRANSVERSE_ZERO_Pin) && defined(PICTURE_TRANSVERSE_ZERO_GPIO_Port)
    return (HAL_GPIO_ReadPin(PICTURE_TRANSVERSE_ZERO_GPIO_Port, PICTURE_TRANSVERSE_ZERO_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return 0U;
#endif
}

static int32_t Picture_Limit_Int32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float Picture_Limit_Float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
