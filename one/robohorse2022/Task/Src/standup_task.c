#include "standup_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "pid.h"
#include "posture_task.h"

float Standup_LegR_Offset = 0;// 右腿偏移量，初始化为0
float Standup_LegL_Offset = 0;// 左腿偏移量，初始化为0
uint8_t flag_Standup = 0;     // 站立完成标志，初始化为0（未站立）

extern osThreadId_t StandupHandle;
extern PID_t PID_Speed[8];

void Standup_Task(void *argument)
{
    while (1)
    {
        uint8_t i = 0;

//        for (i = 0; i < 8; i++)
//        {
//            PID_Speed[i].MaxOutput = 9000;
//        }
//        Standup_LegL_Offset = (-60) * 3.14159f / 180.0f;
//        osDelay(1500);
//        Standup_LegR_Offset = (40) * 3.14159f / 180.0f;
//        Standup_LegL_Offset += (-60) * 3.14159f / 180.0f;
//        osDelay(1500);
//        Standup_LegR_Offset += (-20) * 3.14159f / 180.0f;
//        Standup_LegL_Offset += (10) * 3.14159f / 180.0f;
//        osDelay(500);
//        for (i = 0; i < 8; i++)
//        {
//            PID_Speed[i].MaxOutput = 12000;
//        }
        flag_Standup = 1;
        osThreadSuspend(StandupHandle);
    }
}
