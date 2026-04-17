#ifndef __STANDUP_H__
#define __STANDUP_H__

#include <stdint.h>

extern float Standup_LegR_Offset;//右腿偏移量（浮点型）
extern float Standup_LegL_Offset;//左腿偏移量（浮点型）
extern uint8_t flag_Standup;     //站立状态标志（8位无符号整数）

void Standup_Task(void *argument);

#endif
