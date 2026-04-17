#include "remoctrl_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include "pid.h"
#include "app_btdebug.h"
#include "serial.h"
#include "posture_task.h"
#include "hwt906.h"

#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"

/* ===================== 常量定义 ===================== */

#define TX_DATAF32_SIZE         (8U)
#define TX_DATA8_SIZE           (TX_DATAF32_SIZE * 4U)

#define RC_CH_HIGH_THRESH       (1600U)
#define RC_CH_LOW_THRESH        (400U)
#define RC_CH_DEADZONE          (10U)       // 增大死区到10，提高稳定性
#define SWITCH_DEBOUNCE_MS      (50U)       // 减小到50ms，提高响应

#define TURN_THRESHOLD          (0.01f)
#define BRIDGE_CH0_THRESHOLD    (0.05f)

#define AMP_BASE_UP             (2.8f)
#define AMP_BASE_DOWN           (1.5f)
#define AMP_TURN_UP_GAIN        (1.2f)
#define AMP_TURN_DOWN_GAIN      (0.7f)
#define AMP_WALK_UP_BASE        (3.0f)
#define AMP_WALK_UP_GAIN        (1.0f)
#define AMP_WALK_DOWN_BASE      (0.8f)
#define AMP_WALK_DOWN_GAIN      (0.3f)

/* ===================== 类型定义 ===================== */

typedef union {
    uint8_t  data8[TX_DATA8_SIZE];
    float    dataf32[TX_DATAF32_SIZE];
} BtTxData_t;

/* ===================== 外部变量 ===================== */

extern float Target_Angle;
extern AHRS_Angle_t Now_Angle;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t rc_data[18];
extern RC_ctrl_t my_rc_ctrl;
extern RC_Key rc_sw_state;

/* ===================== 全局变量 ===================== */

float CH0_Offset = 0.0f;
float CH1_Offset = 0.0f;
float CH2_Offset = 0.0f;
float CH3_Offset = 0.0f;

uint32_t get_num = 0;
uint8_t flag_imu_line = 0;

/* ===================== 静态变量 ===================== */

static BtTxData_t bt_tx_data;
static BTDebug_Operate_t btdbg_opt;

/* ===================== 静态函数声明 ===================== */

static void UART_Transmit_Bytes(uint8_t *pData, uint32_t Size);
static void UART_Receive_Bytes(uint8_t *pData, uint32_t Size);
static inline float NormalizeChannel(uint16_t ch_value);
static inline uint8_t IsChannelNeutral(uint16_t ch_value);
static inline float LimitFloat(float value, float min, float max);
static uint8_t IsRcNeutral(const RC_ctrl_t *pRCData);
static void UpdateChOffset(const RC_ctrl_t *pRCData);
static void SetAllLegStepLength(Cycloid_Generator_t *pCycGenerato, float step_length);
static void SetAllLegFreq(Cycloid_Generator_t *pCycGenerato, float freq_norm, float freq_max);
static void SetTurnStepLength(Cycloid_Generator_t *pCycGenerato, float ch3_norm, float ch2_norm, float max_length);
static void UpdateTurnAmplitude(Cycloid_Generator_t *pCycGenerato, float turn_intensity);
static void UpdateWalkAmplitude(Cycloid_Generator_t *pCycGenerato, float speed_intensity);
static void HandleTurningOrStraight(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float ch2_norm, float ch3_norm);
static void HandleNormalLocomotion(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float ch1_norm, float ch2_norm, float ch3_norm);
static void HandleLocomotionMode(Cycloid_Generator_t *pCycGenerato, const RC_ctrl_t *pRCData, Robohorse_State state, float ch0_norm, float ch1_norm, float ch2_norm, float ch3_norm);
static void HandleBridgeMode(Cycloid_Generator_t *pCycGenerato, float ch0_norm, float ch3_norm);
static void UpdateClimbingHeight(Cycloid_Generator_t *pCycGenerato);
static void UpdateBridgeHeight(Cycloid_Generator_t *pCycGenerato, uint8_t idx);

/* ===================== UART操作函数 ===================== */

static void UART_Transmit_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Transmit(&huart6, pData, Size, 100);
}

static void UART_Receive_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Receive(&huart6, pData, Size, 100);
}

/* ===================== 内联辅助函数 ===================== */

/**
 * @brief 将通道值归一化到 [-1, 1]
 * @param ch_value 原始通道值 (364~1684)
 * @return 归一化值 [-1, 1]
 */
static inline float NormalizeChannel(uint16_t ch_value)
{
    return ((float)ch_value - RC_CH_MID) / RC_CH_RANGE;
}

/**
 * @brief 判断通道是否在死区内（中性位置）
 * @param ch_value 通道值
 * @return 1=中性, 0=非中性
 */
static inline uint8_t IsChannelNeutral(uint16_t ch_value)
{
    return (ch_value <= (RC_CH_MID + RC_CH_DEADZONE)) && 
           (ch_value >= (RC_CH_MID - RC_CH_DEADZONE));
}

/**
 * @brief 限制浮点值到指定范围
 */
static inline float LimitFloat(float value, float min, float max)
{
    return (value < min) ? min : ((value > max) ? max : value);
}

/* ===================== 主任务 ===================== */

void RemoCtrl_Task(void *argument)
{
    /* 初始化调试接口 */
    btdbg_opt.ReceiveBytes = UART_Receive_Bytes;
    btdbg_opt.TransmitBytes = UART_Transmit_Bytes;
    
    (void)argument;  // 消除未使用参数警告
    
    for (;;)
    {
        /* 解析遥控器数据 - 临界区保护 */
        taskENTER_CRITICAL();
        RC_Transform(rc_data, &my_rc_ctrl);
        taskEXIT_CRITICAL();

        /* 发送调试数据 */
        BTDebug_TransmitPackage(&btdbg_opt, bt_tx_data.data8, TX_DATA8_SIZE);
        
        osDelay(10);  // 100Hz
    }
}

/* ===================== 遥控器数据处理 ===================== */

void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData)
{
    static RC_Key now_sw_state = RC_NULL;
    static RC_Key last_sw_state;
    static uint32_t last_sw_time = 0;
    
    /* 防御性检查 */
    if (pCycGenerato == NULL || pRCData == NULL)
    {
        return;
    }
    
    /* 急停处理 - 最高优先级 */
    if (rc_sw_state == RC_22)
    {
        if (now_sw_state != RC_22)  // 状态变化时才执行
        {
            now_sw_state = RC_22;
            Change_NowState(STOP);
            dm_j8009_all_disable();
            flag_imu_line = 0;
        }
        return;
    }

    /* 使能电机（非急停状态） */
    dm_j8009_all_enable();

    /* 检查摇杆是否全部回中 */
    if (IsRcNeutral(pRCData))
    {
        osDelay(5);
        if (IsRcNeutral(pRCData))  // 二次确认，防抖
        {
            Change_NowState(STOP);
        }
        return;
    }

    /* 开关状态消抖处理 */
    if (now_sw_state != rc_sw_state)
    {
        last_sw_state = rc_sw_state;
        uint32_t current_time = osKernelGetTickCount();
        
        if ((current_time - last_sw_time) >= SWITCH_DEBOUNCE_MS)
        {
            if (rc_sw_state == last_sw_state)
            {
                now_sw_state = rc_sw_state;
                last_sw_time = current_time;
            }
        }
    }

    /* 更新通道偏移值 */
    taskENTER_CRITICAL();
    UpdateChOffset(pRCData);

    /* 根据开关状态执行对应模式 */
    switch (now_sw_state)
    {
        case RC_33:  // S1下, S2下 -> 步行模式
            Change_RmVAL(pCycGenerato, pRCData, WALK);
            break;

        case RC_23:  // S1中, S2下 -> 小跑模式
            Change_RmVAL(pCycGenerato, pRCData, TROT);
            break;

        case RC_11:  // S1上, S2上 -> 跳跃模式
        {
            uint8_t ch1_high = (pRCData->rc.ch[1] > RC_CH_HIGH_THRESH);
            uint8_t ch3_high = (pRCData->rc.ch[3] > RC_CH_HIGH_THRESH);
            uint8_t ch1_low = (pRCData->rc.ch[1] < RC_CH_LOW_THRESH);
            uint8_t ch3_low = (pRCData->rc.ch[3] < RC_CH_LOW_THRESH);
            
            if (ch1_high && ch3_high)
            {
                Change_NowState(JUMP_UPHILL);
            }
            else if (ch1_low && ch3_low)
            {
                Change_NowState(JUMP_GROUND);
            }
            break;
        }

        case RC_21:  // S1中, S2上 -> 攀爬模式
            Change_RmVAL(pCycGenerato, pRCData, CLIMBING);
            break;

        case RC_12:  // S1上, S2中 -> 过桥模式
            Change_RmVAL(pCycGenerato, pRCData, BRIDGE);
            break;

        default:
            break;
    }
    taskEXIT_CRITICAL();
}

/* ===================== 步态参数更新 ===================== */

void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state)
{
    if (pCycGenerato == NULL || pRCData == NULL)
    {
        return;
    }
    
    switch (state)
    {
        case TROT:
        case WALK:
        case CLIMBING:
            HandleLocomotionMode(pCycGenerato, pRCData, state, 
                                CH0_Offset, CH1_Offset, CH2_Offset, CH3_Offset);
            break;
            
        case BRIDGE:
            HandleBridgeMode(pCycGenerato, CH0_Offset, CH3_Offset);
            break;
            
        default:
            Change_NowState(state);
            break;
    }
}

/* ===================== 运动模式处理 ===================== */

static void HandleLocomotionMode(Cycloid_Generator_t *pCycGenerato, 
                                 const RC_ctrl_t *pRCData, 
                                 Robohorse_State state, 
                                 float ch0_norm, float ch1_norm, 
                                 float ch2_norm, float ch3_norm)
{
    const uint8_t is_ch0_neutral = IsChannelNeutral(pRCData->rc.ch[0]);
    const uint8_t is_ch1_neutral = IsChannelNeutral(pRCData->rc.ch[1]);
    const uint8_t ch4_lock = (pRCData->rc.ch[4] >= 1200);

    /* CH4锁定时，CH2（转向）无效 */
    float ch2_eff = ch4_lock ? 0.0f : ch2_norm;

    if (is_ch0_neutral && is_ch1_neutral)
    {
        /* 双摇杆回中：处理转向或直行 */
        HandleTurningOrStraight(pCycGenerato, state, ch2_eff, ch3_norm);
    }
    else
    {
        /* 常规运动：处理速度和方向 */
        HandleNormalLocomotion(pCycGenerato, state, ch1_norm, ch2_eff, ch3_norm);
    }
}

static void HandleTurningOrStraight(Cycloid_Generator_t *pCycGenerato, 
                                    Robohorse_State state, 
                                    float ch2_norm, float ch3_norm)
{
    const float turn_intensity = fabsf(ch2_norm);
    const float abs_ch3 = fabsf(ch3_norm);

    if (ch2_norm > TURN_THRESHOLD)
    {
        /* 右转 */
        Change_NowState(TURN_RIGHT);
        SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, RC_CPLTCTRL_STEPLENTH_MAX);
    }
    else if (ch2_norm < -TURN_THRESHOLD)
    {
        /* 左转 */
        Change_NowState(TURN_LEFT);
        SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, RC_CPLTCTRL_STEPLENTH_MAX);
    }
    else
    {
        /* 直行 */
        Change_NowState(state);
        SetAllLegStepLength(pCycGenerato, RC_CPLTCTRL_STEPLENTH_MAX * ch3_norm);

        /* 步行模式下根据速度调整摆幅 */
        if (state == WALK && abs_ch3 > TURN_THRESHOLD)
        {
            UpdateWalkAmplitude(pCycGenerato, abs_ch3);
        }
    }

    /* 更新转向摆幅（转向时） */
    if (turn_intensity > TURN_THRESHOLD)
    {
        UpdateTurnAmplitude(pCycGenerato, turn_intensity);
    }
}

static void HandleNormalLocomotion(Cycloid_Generator_t *pCycGenerato, 
                                   Robohorse_State state, 
                                   float ch1_norm, float ch2_norm, float ch3_norm)
{
    /* 攀爬模式：设置前后腿不同高度 */
    if (state == CLIMBING)
    {
        UpdateClimbingHeight(pCycGenerato);
    }

    /* 设置运动状态 */
    Change_NowState(state);
    
    /* 设置转向步长（差速） */
    SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, RC_CPLTCTRL_STEPLENTH_MAX);

    /* 根据模式设置频率 */
    float freq_max = (state == TROT) ? RC_TROT_FREQ_MAX : RC_CPLTCTRL_FREQ_MAX;
    SetAllLegFreq(pCycGenerato, ch1_norm, freq_max);
}

static void HandleBridgeMode(Cycloid_Generator_t *pCycGenerato, 
                             float ch0_norm, float ch3_norm)
{
    /* 根据CH0选择姿态（左倾/右倾） */
    uint8_t idx;
    if (ch0_norm > BRIDGE_CH0_THRESHOLD)
    {
        idx = 0;  // 左高右低
    }
    else if (ch0_norm < -BRIDGE_CH0_THRESHOLD)
    {
        idx = 1;  // 右高左低
    }
    else
    {
        return;  // 在中位，不调整
    }

    UpdateBridgeHeight(pCycGenerato, idx);
    
    /* 设置步长和频率 */
    SetAllLegStepLength(pCycGenerato, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ch3_norm);
    SetAllLegFreq(pCycGenerato, ch0_norm, RC_TROT_FREQ_MAX);
}

/* ===================== 辅助函数 ===================== */

static void UpdateChOffset(const RC_ctrl_t *pRCData)
{
    CH0_Offset = NormalizeChannel(pRCData->rc.ch[0]);
    CH1_Offset = NormalizeChannel(pRCData->rc.ch[1]);
    CH2_Offset = NormalizeChannel(pRCData->rc.ch[2]);
    CH3_Offset = NormalizeChannel(pRCData->rc.ch[3]);
}

static uint8_t IsRcNeutral(const RC_ctrl_t *pRCData)
{
    for (int i = 0; i < 4; i++)
    {
        if (!IsChannelNeutral(pRCData->rc.ch[i]))
        {
            return 0;
        }
    }
    return 1;
}

static void SetAllLegStepLength(Cycloid_Generator_t *pCycGenerato, float step_length)
{
    float limited = LimitFloat(step_length, -RC_CPLTCTRL_STEPLENTH_MAX, RC_CPLTCTRL_STEPLENTH_MAX);
    
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.step_length = limited;
    }
}

static void SetAllLegFreq(Cycloid_Generator_t *pCycGenerato, float freq_norm, float freq_max)
{
    float freq = LimitFloat(freq_max * freq_norm, -freq_max, freq_max);
    
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.freq = freq;
    }
}

static void SetTurnStepLength(Cycloid_Generator_t *pCycGenerato, 
                              float ch3_norm, float ch2_norm, float max_length)
{
    const float left_len = LimitFloat(max_length * (ch3_norm + ch2_norm), 
                                      -max_length, max_length);
    const float right_len = LimitFloat(max_length * (ch3_norm - ch2_norm), 
                                       -max_length, max_length);
    
    pCycGenerato[0].param.step_length = left_len;   // 左前
    pCycGenerato[1].param.step_length = left_len;   // 左后
    pCycGenerato[2].param.step_length = right_len;  // 右前
    pCycGenerato[3].param.step_length = right_len;  // 右后
}

static void UpdateTurnAmplitude(Cycloid_Generator_t *pCycGenerato, float turn_intensity)
{
    const float up_amp = AMP_BASE_UP + turn_intensity * AMP_TURN_UP_GAIN;
    const float down_amp = AMP_BASE_DOWN - turn_intensity * AMP_TURN_DOWN_GAIN;
    
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.up_amp = up_amp;
        pCycGenerato[i].param.down_amp = down_amp;
    }
}

static void UpdateWalkAmplitude(Cycloid_Generator_t *pCycGenerato, float speed_intensity)
{
    const float up_amp = AMP_WALK_UP_BASE + speed_intensity * AMP_WALK_UP_GAIN;
    const float down_amp = AMP_WALK_DOWN_BASE - speed_intensity * AMP_WALK_DOWN_GAIN;
    
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.up_amp = up_amp;
        pCycGenerato[i].param.down_amp = down_amp;
    }
}

static void UpdateClimbingHeight(Cycloid_Generator_t *pCycGenerato)
{
    pCycGenerato[0].param.body_height = FORWARD_hight;
    pCycGenerato[1].param.body_height = BACK_hight;
    pCycGenerato[2].param.body_height = FORWARD_hight;
    pCycGenerato[3].param.body_height = BACK_hight;
}

static void UpdateBridgeHeight(Cycloid_Generator_t *pCycGenerato, uint8_t idx)
{
    if (idx > 1) 
    {
        return;  // 防御性检查
    }
    
    static const float body_h[2][4] = {
        {longK_hight, longK_hight, shortK_hight, shortK_hight},  // idx=0: 左高右低
        {shortK_hight, shortK_hight, longK_hight, longK_hight}   // idx=1: 右高左低
    };
    
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.body_height = body_h[idx][i];
    }
}
