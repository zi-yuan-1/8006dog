#ifndef REMOCTRL_TASK_H
#define REMOCTRL_TASK_H

#include "gait_param.h"
#include "serial.h"

//步态参数限制
#define STEPLENTH_MAX                      (12.0f) //步长最大值
#define RC_CPLTCTRL_FREQ_MAX               (11.0f) //遥控器控制步频最大值
#define RC_TROT_FREQ_MAX                    (7.0f) //遥控器控制步频最大值
#define RC_WALK_FREQ_MAX                   (5.0f) //遥控器控制步频最大值
#define RC_CPLTCTRL_STEPLENTH_MAX          (11.0f)//遥控器控制步幅最大值
#define RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX (5.0f) //遥控器微调步幅最大值
#define RC_CPLTCTRL_FLIGHTPERCENT_MAX      (0.2f) //遥控器控制摆动相占比最大值

//通道参数
#define RC_CH_MID           (1024U)
#define RC_CH_RANGE         (660.0f)
#define RC_CH_MIN           (364U)   // 手册定义最小值
#define RC_CH_MAX           (1684U)  // 手册定义最大值


extern float CH0_Offset;
extern float CH1_Offset;
extern float CH2_Offset;
extern float CH3_Offset;

void RemoCtrl_Task(void *argument);

/* 内部函数声明（供调试或外部调用） */
void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state);

#endif /*REMOCTRL_TASK_H*/
