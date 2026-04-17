#include "jump_task.h"
//#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gait_param.h"
#include "posture_task.h"

extern osThreadId_t JumpHandle;
extern PID_t PID_Speed[8];       //速度PID
extern PID_t PID_Position[8];    //位置PID

float start_time_jump = 0.0f;

void Jump_OnlyOnce(void);
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer);

/**
 * @brief 跳跃任务主函数（FreeRTOS任务）
 * @param argument 任务参数（未使用）
 * @note  上电首次调用会跳过，防止误触发
 */
void Jump_Task(void *argument)
{
    while (1)
    {
        static uint8_t flag_enterTwice = 0;

        /* 防止上电触发运行任务 */
        if (flag_enterTwice == 0)
        { /* 有两个jump任务调用Jump_Task */
            flag_enterTwice++;//第一次调用直接跳过
        }
        else
        {
            start_time_jump = HAL_GetTick(); //记录跳跃开始的时间
            Jump_OnlyOnce();                 //执行单次跳跃
        }
        osThreadSuspend(JumpHandle);         //暂停当前任务，等待下一次触发
    }
}

/**
 * @brief  执行单次完整跳跃周期
 * @note   跳跃一次，跳跃完成后会将NowState切换为STOP
 * @return void
 */
void Jump_OnlyOnce()
{
    const float prep_time      = 1.2f;  //储能下蹲时间
    const float launch_time    = 0.18f; //爆发起跳时间
    const float fall_time      = 0.55f;  //空中减速时间
    const float stabilize_time = 1.0f;  //落地稳定时间
    //总周期：2.93s
    float t = 0.0f;
    //平地
    if (NowState == JUMP_GROUND)
    {
        /* 初始极坐标：下蹲姿态 */
        Polar_Coord_Data_t Action_Polar[4] = {
            /*   r      θ       */
            { 12.0f, -22.0f },  // 腿0
            { 12.0f, -22.0f },  // 腿1
            { 12.0f, -22.0f },  // 腿2
            { 12.0f, -22.0f }   // 腿3
        };

        while (t <= prep_time + launch_time + fall_time + stabilize_time)
		{
            //计算当前相对时间
            t = (HAL_GetTick() - start_time_jump) / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
				for(int a = 0; a < 8; a++)
				{
                    PID_Reset(&PID_Position[a], 95.0f, 0.001f, 2.0f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 23.0f;
                    Action_Polar[i].theta = -18.0f;
                }
                for(int a = 0; a < 8; a++)
				{ 
					PID_Reset(&PID_Position[a], 115.0f, 0.0005f, 3.0f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 15.0f;
                    Action_Polar[i].theta  = 8.0f;
                }
                for(int i =0; i < 8; i++)
				{ 
					PID_Reset(&PID_Position[i], 85.0f, 0.001f, 3.0f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 17.5f;
                    Action_Polar[i].theta  = 3.0f;
                }
                for(int i = 0; i < 8; i++)
				{
                PID_Reset(&PID_Position[i], 70.0f, 0.002f, 1.8f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else
            {
                Change_NowState(STOP);
				for(int i = 0; i < 8; i++)
				{
                PID_Reset(&PID_Position[i], 85.0f, 0.001f, 2.2f);
				}
                break;
            }
        }
    }
    //上坡跳跃模式
    else if (NowState == JUMP_UPHILL)
    {
        t = 0.0f;
        //初始极坐标：适应坡度的下蹲姿态
        Polar_Coord_Data_t Action_Polar[4] = {
            /*   r      θ       */
            { 13.0f, -20.0f },  // 腿0
            { 13.0f, -20.0f },  // 腿1
            { 11.0f, -24.0f },  // 腿2
            { 11.0f, -24.0f }   // 腿3
        };
        
        while (t <= prep_time + launch_time + fall_time + stabilize_time)
        {
            t = (HAL_GetTick() - start_time_jump) / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                for(int i = 0; i < 8; i++)
				{
                    PID_Reset(&PID_Position[i], 80.0f, 0.001f, 2.2f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 26.0f;
                    Action_Polar[i].theta = -15.0f;
                }
                for(int i = 0; i < 8; i++)
				{
                    PID_Reset(&PID_Position[i], 100.0f, 0.0005f, 3.0f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 16.0f;
                    Action_Polar[i].theta  = 2.0f;
                }
                for(int i = 0; i < 8; i++)
				{
                    PID_Reset(&PID_Position[i], 70.0f, 0.001f, 2.5f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 16.5f;
                    Action_Polar[i].theta  = 3.0f;
                }
                for(int i = 0; i < 8; i++)
				{
                    PID_Reset(&PID_Position[i], 55.0f, 0.002f, 2.0f);
				}
                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else
            {
                Change_NowState(STOP);
                for(int i = 0; i < 8; i++)
				{
                    PID_Reset(&PID_Position[i], 85.0f, 0.001f, 2.2f);
				}
                break;
            }
        }
    }
}
/**
 * @brief 将目标极坐标转化成电机角度并输出到电机
 * @param Action_Polar_Buffer 目标极坐标 数组长度必须为4
 * @return void
 */
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer)
{
    /* 直角坐标暂存区 */
    static Cart_Coord_Data_t Action_Cart_Buffer[4] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        //极坐标 → 直角坐标
        Polar_toCartesian(&Action_Polar_Buffer[i], &Action_Cart_Buffer[i]);

        //设置目标位置
        Stanford_Type_Lite_SetPosition(&Gait_Data[i], Action_Cart_Buffer[i].cx, Action_Cart_Buffer[i].cy); //运动学解算待数据填充

        //逆运动学解算
        Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i]);

        //计算基础角度
        float angle_inner = (pi / 2.0f + Gait_Data[i].angle[0]);
        float angle_outer = (pi / 2.0f - Gait_Data[i].angle[1]);

        //映射到具体电机（考虑安装方向和腿部位）
        switch (i)
        {
            case 0:  /* 左前腿：电机2（内侧）, 3（外侧）*/
                Aim_Angle[2] =  angle_inner;
                Aim_Angle[3] =  angle_outer;
                break;

            case 1:  /* 左后腿：电机4（内侧）, 5（外侧）*/
                Aim_Angle[4] = -angle_inner;  // 反向
                Aim_Angle[5] = -angle_outer;  // 反向
                break;

            case 2:  /* 右后腿：电机6（内侧）, 7（外侧）*/
                Aim_Angle[6] =  -angle_inner;
                Aim_Angle[7] =  -angle_outer;
                break;

            case 3:  /* 右前腿：电机0（外侧）, 1（内侧）- 顺序相反 */
                Aim_Angle[0] = angle_outer;  // 外侧电机，反向
                Aim_Angle[1] = angle_inner;  // 内侧电机，反向
                break;

            default:
                break;
        }
    }

    Is_Aim_Angle_Get = 1; //数据填充完毕
}
