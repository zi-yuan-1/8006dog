#include "posture_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "hwt906.h"
#include "serial.h"
#include "ahrs_task.h"
#include "jump_task.h"
#include "gait_param.h"

//常量定义
#define MOTOR_ANGLE_SCALE       0.5f
#define CONTROL_PERIOD_MS       10
#define TRANSITION_DELAY_MS     500
#define RC_CH_SCALE             1320.0f

#define LEG_RADIUS_BASE         9.0f
#define LEG_RADIUS_VAR          15.0f
#define LEG_RADIUS_MIN          6.0f    //最小半径限幅
#define LEG_RADIUS_MAX          26.0f   //最大半径限幅

#define CLIMBING_X_OFFSET       2.0f
#define STEP_LENGTH_MAX         20.0f
#define STEP_LENGTH_MIN         0.0f

//全局变量
float aim_x[4] = {0.0f};
float aim_y[4] = {0.0f};
float now_time = 0.0f;
int time_cnt = 0;

extern float revise_LegLenth[4];
extern osThreadId_t JumpHandle;
extern osThreadId_t JumpUphillHandle;
extern uint8_t flag_Standup;
extern uint8_t flag_imu_line;

extern float CH0_Offset, CH1_Offset, CH2_Offset, CH3_Offset;

/* ==================== 函数声明 ==================== */
static void Calculate_Gait(Robohorse_State state);
static void Target_toMotor(void);
static void IKUN_State(Robohorse_State target, RC_ctrl_t *pRCData);
static inline float Limit_StepLength(float val);
static void Update_PhaseOnModeChange(void);

extern void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);

//内联工具函数

/**
 * @brief 步长限幅，防止负数或过大
 */
static inline float Limit_StepLength(float val)
{
    if (val > STEP_LENGTH_MAX) return STEP_LENGTH_MAX;
    if (val < STEP_LENGTH_MIN) return STEP_LENGTH_MIN;
    return val;
}

/**
 * @brief 模式切换时更新相位基准
 */
static void Update_PhaseOnModeChange(void)
{
    Last_Time = HAL_GetTick() / 1000.0f;
    for (uint8_t i = 0; i < 4; i++) {
        Cycloid_Gen[i].last_time = Last_Time;
    }
}

//主任务
void Posture_Task(void *argument)
{
    static portTickType xLastWakeTime = 0;
    const portTickType xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    
    (void)argument;
    
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        gait_params = state_gait_params[NowState];
        RcDetachedParam = state_detached_params[NowState];

        if (Mode_Change_Flag) {
            Update_PhaseOnModeChange();
            Mode_Change_Flag = 0;
        }

        for (uint8_t i = 0; i < 4; i++) {
            Cycloid_Gen[i].param = gait_params;
        }

        Ctrl_byRemoctrl(Cycloid_Gen, &my_rc_ctrl);

        switch (NowState)
        {
        case STOP:
            for (uint8_t i = 0; i < 4; i++) {
                aim_x[i] = 0.0f;
                aim_y[i] = init_h;
            }
            Target_toMotor();
            break;

        case NORMAL:
            IKUN_State(NORMAL, &my_rc_ctrl);
            break;

        case CLIMBING:
        case WALK:
        case TROT:
        case BRIDGE:
        case TURN_LEFT:
        case TURN_RIGHT:
            Calculate_Gait(NowState);
            Target_toMotor();
            break;

        case JUMP_GROUND:
        case JUMP_UPHILL:
            osThreadResume(JumpHandle);
            break;

        case RELEASE:
        default:
            osDelay(TRANSITION_DELAY_MS);
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//状态处理函数
void IKUN_State(Robohorse_State target, RC_ctrl_t *pRCData)
{
    (void)target;
    
    static Polar_Coord_Data_t Action_Polar[4];
    uint8_t i;
    
    const float ch1_offset = pRCData->rc.ch[1] / RC_CH_SCALE;
    const float ch3_offset = pRCData->rc.ch[3] / RC_CH_SCALE;
    
    CH1_Offset = ch1_offset;
    CH3_Offset = ch3_offset;

    for (i = 0; i < 4; i++) {
        const float offset = (i < 2) ? ch1_offset : ch3_offset;

        float raw_radius = LEG_RADIUS_BASE + LEG_RADIUS_VAR * offset;

        if (raw_radius < 0.0f) raw_radius = -raw_radius;
        if (raw_radius > LEG_RADIUS_MAX) raw_radius = LEG_RADIUS_MAX;
        if (raw_radius < LEG_RADIUS_MIN) raw_radius = LEG_RADIUS_MIN;

        Action_Polar[i].radius = raw_radius;
        Action_Polar[i].theta = 0.0f;
    }
    
    Polar_setCoord(Action_Polar);
}

//步态切换

void Change_NowState(Robohorse_State target)
{
    if (!flag_Standup) {
        return;
    }
    
    if (NowState != target) {
        NowState = target;
        Mode_Change_Flag = 1;
    }
}

//步态计算
static Cycloid_Generator_Solution_t cyc_solution[4];

/**
 * @brief IMU自适应步长调整
 */
static inline void Apply_IMU_Correction(uint8_t leg_idx)
{
    if (flag_imu_line != 1) {
        return;
    }
    
    const float correction = (leg_idx < 2) ? (-step_len_dev) : step_len_dev;
    Cycloid_Gen[leg_idx].param.step_length += correction;
    Cycloid_Gen[leg_idx].param.step_length = 
        Limit_StepLength(Cycloid_Gen[leg_idx].param.step_length);
}

static void Calculate_Gait(Robohorse_State state)
{
    now_time = HAL_GetTick() / 1000.0f;
    uint8_t i;

    switch (state)
    {
    case WALK:
        for (i = 0; i < 4; i++) {
            Apply_IMU_Correction(i);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case TROT:
        for (i = 0; i < 4; i++) {
            Apply_IMU_Correction(i);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case TURN_LEFT:
    case TURN_RIGHT:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.76f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.26f, &cyc_solution[3]);
        break;

    case CLIMBING:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case BRIDGE:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.01f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    default:
        break;
    }

    const float x_offset = (state == CLIMBING) ? CLIMBING_X_OFFSET : 0.0f;
    for (i = 0; i < 4; i++) {
        aim_x[i] = cyc_solution[i].x - x_offset;
        aim_y[i] = cyc_solution[i].y;
    }
}

//电机输出

#define CALC_MOTOR_ANGLE(leg_idx, motor_out, motor_in, sign) \
    do { \
        Aim_Angle[motor_out] = (sign) * (pi / 2.0f - Gait_Data[leg_idx].angle[1]) / MOTOR_ANGLE_SCALE; \
        Aim_Angle[motor_in]  = (sign) * (pi / 2.0f + Gait_Data[leg_idx].angle[0]) / MOTOR_ANGLE_SCALE; \
    } while(0)

static void Target_toMotor(void)
{
    uint8_t i;
    
    for (i = 0; i < 4; i++) {
        Stanford_Type_Lite_SetPosition(&Gait_Data[i], aim_x[i], aim_y[i]);
        Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i]);

        switch (i) {
    case 0:  // 腿0：ID 3,2
    Aim_Angle[3] = -(pi / 2.0f - Gait_Data[0].angle[1]) / MOTOR_ANGLE_SCALE;
    Aim_Angle[2] = -(pi / 2.0f + Gait_Data[0].angle[0]) / MOTOR_ANGLE_SCALE;
    break;

case 1:  // 腿1：ID 5,4
    Aim_Angle[5] = -(pi / 2.0f - Gait_Data[1].angle[1]) / MOTOR_ANGLE_SCALE;
    Aim_Angle[4] = -(pi / 2.0f + Gait_Data[1].angle[0]) / MOTOR_ANGLE_SCALE;
    break;

case 2:  // 腿2：ID 7,6
    Aim_Angle[7] = (pi / 2.0f - Gait_Data[2].angle[1]) / MOTOR_ANGLE_SCALE;
    Aim_Angle[6] = (pi / 2.0f + Gait_Data[2].angle[0]) / MOTOR_ANGLE_SCALE;
    break;

case 3:  // 腿3：ID 1,0
    Aim_Angle[1] =  (pi / 2.0f + Gait_Data[3].angle[1]) / MOTOR_ANGLE_SCALE;
    Aim_Angle[0] =  (pi / 2.0f - Gait_Data[3].angle[0]) / MOTOR_ANGLE_SCALE;
    break;
        }
    }
    
    Is_Aim_Angle_Get = 1;
}

#undef CALC_MOTOR_ANGLE
