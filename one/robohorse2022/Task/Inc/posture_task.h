#ifndef POSTURE_TASK_H
#define POSTURE_TASK_H

#include "gait_param.h"

/**
 * @brief 步态控制主任务
 * @param argument FreeRTOS任务参数（未使用）
 * @note 周期：10ms，优先级：高
 */
void Posture_Task(void *argument);

/**
 * @brief 切换机器人当前状态
 * @param target 目标状态枚举
 * @note 仅在flag_Standup=1时生效，自动处理模式切换过渡
 */
void Change_NowState(Robohorse_State target);

/* 目标足端坐标（单位：cm），用于调试或外部监控 */
extern float aim_x[4];
extern float aim_y[4];

/* 当前时间戳（单位：s），用于轨迹生成 */
extern float now_time;

#endif /*POSTURE_TASK_H*/
