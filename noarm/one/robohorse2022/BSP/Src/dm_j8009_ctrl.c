#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"
#include "dm8009.h"

#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"

// J8009电机反馈数据（8个电机）
J8009_Measure_t J8009[8];

// 速度环PID数组
PID_t PID_Speed[8];

//关节电机位置限幅
float pos_limit_min[8] = {MOTOR_0_POS_MIN, MOTOR_1_POS_MIN, MOTOR_2_POS_MIN, MOTOR_3_POS_MIN,MOTOR_4_POS_MIN,MOTOR_5_POS_MIN, MOTOR_6_POS_MIN, MOTOR_7_POS_MIN};
float pos_limit_max[8] = {MOTOR_0_POS_MAX, MOTOR_1_POS_MAX, MOTOR_2_POS_MAX, MOTOR_3_POS_MAX,MOTOR_4_POS_MAX,MOTOR_5_POS_MAX, MOTOR_6_POS_MAX, MOTOR_7_POS_MAX};

extern uint8_t DM_J8009_ENABLE_FLAG;
extern uint8_t DM_J8009_ctrl_FLAG;

/**
  * @brief          达妙电机位置环控制
  * @param[in]      can: CAN句柄
  * @param[in]      id: 电机id
  * @param[in]      set_pos: 目标位置
  * @param[in]      kp: 控制参数KP
  * @param[in]     	kd: 控制参数KD
  * @retval         none
  */
void dm_j8009_pos_ctrl(CAN_HandleTypeDef can,uint8_t id, float set_pos, float kp, float kd)
{
//	if(id > 7)			//关节电机ID为1~4
//		return;
	//关节先限幅   防止超调卡住损坏电机
	if(set_pos > pos_limit_max[id]){				//先是最大值限幅
		set_pos = pos_limit_max[id];
	}
	if(set_pos < pos_limit_min[id]){				//再是最小值限幅
		set_pos = pos_limit_min[id];
	}
	
	CTRL_DM_J8009_MIT(can, id, set_pos, 0, kp, kd, 0);
}


void dm_PD_set(PID_t *pid, LegGain state_pid)
{
    pid->p = state_pid.kp_pos;
    pid->d = state_pid.kd_pos;
}

/**
 * @brief 达妙电机双环控制（新增）
 * @param SET_Angle 目标角度数组
 * @param PID_POS 位置环PID数组（pos_out为速度目标，p为KP，d为KD）
 * @param PID_SPD 速度环PID数组（pos_out为力矩输出）
 * @retval none
 * @note 使用MIT模式：T = kp*(pos - pos_actual) + kd*(vel - vel_actual) + torque
 *       双环实现：vel = 位置环输出，torque = 速度环输出
 */
void dm_j8009_all_dual_ctrl(float SET_Angle[8], PID_t PID_POS[8], PID_t PID_SPD[8])
{
    for (uint8_t id = 0; id <= 0x07; id++)
    {
        // 位置限幅
        float set_pos = SET_Angle[id];
        if (set_pos > pos_limit_max[id]) set_pos = pos_limit_max[id];
        if (set_pos < pos_limit_min[id]) set_pos = pos_limit_min[id];
        
        // 双环参数：vel=位置环输出（速度目标），torque=速度环输出（力矩）
        float vel_target = PID_POS[id].pos_out;      // 位置环输出作为速度目标
        float torque = PID_SPD[id].pos_out;           // 速度环输出作为力矩
        float kp = PID_POS[id].p;                     // 位置环KP
        float kd = PID_POS[id].d;                     // 位置环KD（或速度环D）
        
        if (id == 0x00 || id == 0x01 || id == 0x06 || id == 0x07) {
            while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
            CTRL_DM_J8009_MIT(hcan1, id, set_pos, vel_target, kp, kd, torque);
            vTaskDelay(1);
        }
        else if (id == 0x02 || id == 0x03 || id == 0x04 || id == 0x05) {
            while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));
            CTRL_DM_J8009_MIT(hcan2, id, set_pos, vel_target, kp, kd, torque);
            vTaskDelay(1);
        }
        DM_J8009_ctrl_FLAG = 1;
    }
}

/**
 * @brief 达妙电机控制（保留原有单环位置控制）
 * @retval none
 */
void dm_j8009_all_ctrl(float SET_Angle[8], PID_t PID_SET[8])
{
		
	for(uint8_t id=0; id<=0x07; id++)
	{
		if(id == 0x00 || id == 0x01 || id == 0x06 || id==0x07){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			dm_j8009_pos_ctrl(hcan1,id,SET_Angle[id], PID_SET[id].p, PID_SET[id].d);
			vTaskDelay(1);    //延迟1ms
		}
		else if(id == 0x02 || id == 0x03 || id == 0x04 || id == 0x05){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			dm_j8009_pos_ctrl(hcan2,id,SET_Angle[id], PID_SET[id].p, PID_SET[id].d);
			vTaskDelay(1);    //延迟1ms
		}
		DM_J8009_ctrl_FLAG = 1;  
	}
}

/**
  * @brief          达妙电机使能
  * @retval         none
  */
void dm_j8009_all_enable(void){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x00);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x01);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x06);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x07);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x02);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x03);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x04);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x05);
			vTaskDelay(1);    //延迟1ms
			DM_J8009_ENABLE_FLAG = 1;
	
}

/**
  * @brief          达妙电机失能
  * @retval         none
  */
void dm_j8009_all_disable(void){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x00);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x01);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x06);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x07);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x02);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x03);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x04);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x05);
			vTaskDelay(1);    //延迟1ms
			DM_J8009_ENABLE_FLAG = 0;			
}
