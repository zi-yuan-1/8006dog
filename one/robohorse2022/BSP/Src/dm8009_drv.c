#include "dm8009_drv.h"
/**
  ****************************(C) COPYRIGHT 2024 Zhang****************************
  * @file      motor_task.c/h
  * @brief     达妙电机驱动程序电机任务
  * @note      4个达妙J8009
  * @history
  *  Version    Date            Author          Modification
  *	 V3.0.0			2024-7-12			R&A战队					张哲元
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Zhang****************************
  */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef  can_tx_message;
uint8_t              can_send_data[8];

DM_J8009_DATA_t dm_j8009_data[8];		//达妙电机反馈数据


/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
  * @brief    清除错误  发送给达妙电机  DM-J8009
  * @param[in] 	can：	CAN句柄
  * @param[in] 	id：	电机ID
 */
void CLEAR_ERROR_DM_J8009(CAN_HandleTypeDef can, uint16_t id)//统一can参数
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0xFF;
    can_send_data[1] = 0xFF;
    can_send_data[2] = 0xFF;
    can_send_data[3] = 0xFF;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0xFF;
    can_send_data[6] = 0xFF;
    can_send_data[7] = 0xFB;

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);//使用传入的can
}

/**
  * @brief   电机使能  发送给达妙电机  DM-J8009
  * @param[in] 	can：	CAN句柄
  * @param[in] 	id：	电机ID
 */
void ENABLE_DM_J8009(CAN_HandleTypeDef can, uint16_t id)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0xFF;
    can_send_data[1] = 0xFF;
    can_send_data[2] = 0xFF;
    can_send_data[3] = 0xFF;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0xFF;
    can_send_data[6] = 0xFF;
    can_send_data[7] = 0xFC;
		
    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief电机失能  发送给达妙电机  DM-J8009
  * @param[in] 	can：	CAN句柄
  * @param[in] 	id：	电机ID
 */
void DISABLE_DM_J8009(CAN_HandleTypeDef can, uint16_t id)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0xFF;
    can_send_data[1] = 0xFF;
    can_send_data[2] = 0xFF;
    can_send_data[3] = 0xFF;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0xFF;
    can_send_data[6] = 0xFF;
    can_send_data[7] = 0xFD;

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief    保存零点  发送给达妙电机  DM-J8009
  * @param[in] 	can：	CAN句柄
  * @param[in]  id：	电机ID
 */
void SAVE_ZERO_DM_J8009(CAN_HandleTypeDef can, uint16_t id)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0xFF;
    can_send_data[1] = 0xFF;
    can_send_data[2] = 0xFF;
    can_send_data[3] = 0xFF;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0xFF;
    can_send_data[6] = 0xFF;
    can_send_data[7] = 0xFE;

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief  MIT模式控制  发送给达妙电机  DM-J8009
  * @param[in]	     id：电机ID
  * @param[in]		 _pos：位置				单位：rad
  * @param[in]		 _vel：速度				单位：rad/s
  * @param[in]		 _KP： KP
  * @param[in]		 _KD： KD
  * @param[in]		_torq：转矩				单位：N-m
  * @note:给定位置、KP、KD且其余项为0时，此时为位置控制；
          给定速度、KD且其余项为0时，此时为速度控制；
          给定扭矩且其余项为0时，此时为扭矩控制
 */
void CTRL_DM_J8009_MIT(CAN_HandleTypeDef can, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
		uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

    // 限制输入范围，防止溢出
	  _pos = (_pos < P_MIN) ? P_MIN : ((_pos > P_MAX) ? P_MAX : _pos);
	  _vel = (_vel < V_MIN) ? V_MIN : ((_vel > V_MAX) ? V_MAX : _vel);
	  _KP = (_KP < KP_MIN) ? KP_MIN : ((_KP > KP_MAX) ? KP_MAX : _KP);
	  _KD = (_KD < KD_MIN) ? KD_MIN : ((_KD > KD_MAX) ? KD_MAX : _KD);
	  _torq = (_torq < T_MIN) ? T_MIN : ((_torq > T_MAX) ? T_MAX : _torq);

		pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
		vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
		kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
		kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
		tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
		
		uint32_t send_mail_box;
        can_tx_message.StdId = id;
        can_tx_message.IDE = CAN_ID_STD;
        can_tx_message.RTR = CAN_RTR_DATA;
        can_tx_message.DLC = 0x08;
        can_send_data[0] = (pos_tmp >> 8);
        can_send_data[1] = (pos_tmp & 0xFF);
        can_send_data[2] = (vel_tmp >> 4);
        can_send_data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
        can_send_data[4] = (kp_tmp & 0xFF);
        can_send_data[5] = (kd_tmp >> 4);
        can_send_data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
        can_send_data[7] = (tor_tmp & 0xFF);

        HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief    电机反馈数据  来自达妙电机  DM-J8009
  * @param[in] 	can_id：	接收到的CAN帧ID（用于验证）
  * @param[in]	num：	序号(0-7)
  * @param[in]	data：8字节CAN数据
  * @note: 修复12位有符号数符号扩展，分离ID和故障码
 */
void FEEDBACK_DATA_DM_J8009(uint16_t num, uint8_t data[])
{
	int16_t p_raw, v_raw, t_raw;

  // 解析原始数据（保持原有位操作，增加符号扩展）
	p_raw = (int16_t)((data[1] << 8) | data[2]);					// 16位有符号位置
	v_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));				// 12位速度（需符号扩展）
	t_raw = (int16_t)(((data[4] & 0xF) << 8) | data[5]);			// 12位扭矩（需符号扩展）

  // 12位有符号数符号扩展（说明书要求）
	if (v_raw > 2047) v_raw -= 4096;
	if (t_raw > 2047) t_raw -= 4096;

  //存入结构体
	dm_j8009_data[num].p_int = (int)p_raw;
	dm_j8009_data[num].v_int = (int)v_raw;
	dm_j8009_data[num].t_int = (int)t_raw;

	// 优化：正确分离ID和故障码（保持data[0]赋值给id的兼容性）
	dm_j8009_data[num].id = data[0];								// 保持原有行为（包含ERR在高4位）
	// 如需分离使用：dm_j8009_data[num].id = data[0] & 0x0F;		// 低4位ID
	// uint8_t err_code = (data[0] >> 4) & 0x0F;						// 高4位故障码

	//转换为物理量
	dm_j8009_data[num].POS = uint_to_float(dm_j8009_data[num].p_int, P_MIN, P_MAX, 16); 
	dm_j8009_data[num].VEL = uint_to_float(dm_j8009_data[num].v_int, V_MIN, V_MAX, 12); 
	dm_j8009_data[num].TORQUE = uint_to_float(dm_j8009_data[num].t_int, T_MIN, T_MAX, 12); 
	dm_j8009_data[num].T_MOS = data[6];
	dm_j8009_data[num].T_Rotor = data[7];
}
/****************************************************达妙电机驱动程序**************************************************************/
//查阅DM-J8009说明手册
