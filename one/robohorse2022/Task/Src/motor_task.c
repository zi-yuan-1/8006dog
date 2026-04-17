//8009
#include <stdarg.h>
#include <stdint.h>

#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "dm8009.h"
#include "pid.h"
#include "gait_param.h"
#include "usart.h"
#include "app_ebf_piddebug.h"

#include "standup_task.h"
#include "dm_j8009_ctrl.h"

extern PID_t PID_Position[8];    //位置环PID
extern PID_t PID_Speed[8];       //速度环PID

extern float imu_Leg_offset;     //陀螺仪偏移量
float PID_Aim_Angle[8];          //将用于PID运算的目标角度

uint8_t DM_J8009_ENABLE_FLAG = 1;
uint8_t DM_J8009_ctrl_FLAG = 0;

#define PID_CONFIG 0

#if PID_CONFIG == 0

/**
 * 电机驱动任务
 * 流程：装载新的位置-> 双环PID运算->发送新的电流值
 * */
void Motor_Task(void *argument)
{
    static portTickType xLastWakeTime;
    static const portTickType xFrequency = pdMS_TO_TICKS(2);
    xLastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        if (Is_Aim_Angle_Get == 1)
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                // 腿0左前：保持原补偿（正方向未改）
        if (i == 3)
            PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
        else if (i == 2)
            PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegL_Offset + imu_Leg_offset;
        
        // 腿1左后：保持原补偿（负方向未改）
        else if (i == 5)
            PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
        else if (i == 4)
            PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegL_Offset + imu_Leg_offset;
        
        // 腿2右后：补偿符号反转（方向由正变负）
        else if (i == 7)
            PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegR_Offset + imu_Leg_offset;  // 原为 - -
        else if (i == 6)
            PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset - imu_Leg_offset;  // 原为 + +
        
        // 腿3右前：补偿符号反转（方向由负变正）
        else if (i == 0)
            PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegR_Offset - imu_Leg_offset;  // 原为 + -
        else if (i == 1)
            PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset;  // 原为 - +
            }
            Is_Aim_Angle_Get = 0;
        }

        //双环PID运算
        for (uint8_t i = 0; i < 8; i++)
        {
            // 位置环：角度误差 -> 目标速度
            PID_Calc(&PID_Position[i], 
                    J8009[i].Total_Angle * 360.0f / 8191.0f,
                    PID_Aim_Angle[i]);
            
            // 限制位置环输出（速度限制）
            if (PID_Position[i].pos_out > 720.0f) 
                PID_Position[i].pos_out = 720.0f;
            if (PID_Position[i].pos_out < -720.0f) 
                PID_Position[i].pos_out = -720.0f;
            
            // 速度环：速度误差 -> 力矩输出
            PID_Calc(&PID_Speed[i], 
                    J8009[i].Speed_RPM, 
                    PID_Position[i].pos_out);
            
            // 限制力矩输出
            if (PID_Speed[i].pos_out > 18.0f) 
                PID_Speed[i].pos_out = 18.0f;
            if (PID_Speed[i].pos_out < -18.0f) 
                PID_Speed[i].pos_out = -18.0f;
        }
        
        // 发送双环控制指令（使用MIT模式的vel和torque参数）
        dm_j8009_all_dual_ctrl(PID_Aim_Angle, PID_Position, PID_Speed);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#else

// PID调参模式（保持原代码不变）
static uint32_t Delay_Time = 5;

void _Set_Period(uint32_t period)    //设置周期
{
    Delay_Time = period;
}

void _Transmit(uint8_t *pdata, uint32_t data_size) //发送数据
{
    HAL_UART_Transmit(&huart7, pdata, data_size, 100);
}

void _SetPID(float p, float i, float d) //设置PID       
{
    int now_target = EBFPIDDebuger_Get_Target_Value(); //目标值用为设置PID的目标
    if (now_target >= 0 && now_target < 8)
    {
        PID_Position[now_target].p = p;
        PID_Position[now_target].i = i;
        PID_Position[now_target].d = d;
    }
}

EBFPIDDebuger_Opts pid_config_opts =
    {
        .Set_Period = _Set_Period,
        .Transmit = _Transmit,
        .SetPID = _SetPID,
};

/**
 * 电机驱动任务
 * 流程：装载新的位置-> 双环PID运算->发送新的电流值
 * */
void Motor_Task(void *argument)
{
    while (1)
    {
        if (Is_Aim_Angle_Get == 1) //是否已经获得了新的目标角度
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                PID_Aim_Angle[i] = Aim_Angle[i]; //数据转移
            }
            Is_Aim_Angle_Get = 0; //清零标志
        }
        for (uint8_t i = 0; i < 8; i++) //双环PID运算
        {
            // 位置PID
            PID_Calc(&PID_Position[i], J8009[i].Total_Angle * 360 / 8191, PID_Aim_Angle[i] * ReductionAndAngleRatio);
            // 速度PID
            PID_Calc(&PID_Speed[i], J8009[i].Speed_RPM, PID_Position[i].pos_out);
        }
        
        // 调参模式同样使用双环控制
        dm_j8009_all_dual_ctrl(PID_Aim_Angle, PID_Position, PID_Speed);

        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH1, &J8009[0].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH2, &J8009[1].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH3, &J8009[2].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH4, &J8009[3].Total_Angle, 4);

        EBFPIDDebuger_Receiving_Process(&pid_config_opts);
        osDelay(Delay_Time);
    }
}

#endif
