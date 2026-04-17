#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include <main.h>
#include <cmsis_os.h>

//温度保护阈值
#define TEMP_WARNING_THRESHOLD 100 //高温警告阈值
#define TEMP_RECOVERY_THRESHOLD 80 //温度恢复正常阈值，低于此值解除警告

//检测任务配置
#define DETECT_TASK_INTERVAL_MS 100 //检测周期100ms（10Hz）
#define LED_BLINK_INTERVAL_MS 500 //LED闪烁周期500ms

//高温警告标志
extern uint8_t High_Temp_Warning;

//任务函数
void Detect_Task(void *argument);
void Detect_Task_Init(void);

#endif /*DETECT_TASK_H*/
