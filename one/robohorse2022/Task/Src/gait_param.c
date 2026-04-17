/*步态参数设置*/
#include "gait_param.h"

//当前生效PID参数
LegGain Gait_Gains = {15.0f, 0.3f, 20.0f, 0.05f};

//各状态PID参数表（明确大小END+1）
LegGain state_gait_gains[END+1] = {
    //{kp_pos, kd_pos， kp_spd, kd_spd}，
    {15.0f, 0.3f, 20.0f, 0.05f},  // NORMAL
    {18.0f, 0.5f, 16.0f, 0.08f},  // CLIMBING
    {12.0f, 0.2f, 22.0f, 0.03f},  // TURN_LEFT
    {12.0f, 0.2f, 22.0f, 0.03f},  // TURN_RIGHT
    {14.0f, 0.4f, 18.0f, 0.06f},  // WALK
    {10.0f, 0.15f, 25.0f, 0.02f},  // TROT
    {20.0f, 0.8f, 12.0f, 0.1f},  // BRIDGE
    {22.0f, 1.0f, 10.0f, 0.15f},  // STOP
    {15.0f, 0.3f, 20.0f, 0.05f},  // RELEASE
    {16.0f, 0.4f, 18.0f, 0.06f},  // JUMP_GROUND
    {16.0f, 0.4f, 18.0f, 0.06f},  // JUMP_UPHILL
    {18.0f, 0.5f, 15.0f, 0.08f},  // START
    {15.0f, 0.3f, 20.0f, 0.05f},  // END
};

//当前生效轨迹参数
Cycloid_Generator_Param_t gait_params = {18.0f, 0.0f, 2.5f, 0.5f, 0.25f, 2.5f};
// typedef struct
//    {
//        float body_height;    //身体到地面的距离 (cm)
//        float step_length;    //一步的距离 (cm)
//        float up_amp;         //上部振幅y (cm)
//        float down_amp;       //下部振幅 (cm)
//        float flight_percent; //摆动相百分比(%)
//        float freq;           //一步的频率 (Hz)
//    } Cycloid_Generator_Param_t;

Cycloid_Generator_Param_t state_gait_params[END + 1] = {
	  // NORMAL
    {18.0f, 0.0f, 0.00f, 0.00f, 0.0f, 0.0f},    
                                               
		// CLIMBING 爬坡
    {17.0f, 6.0f, 4.0f, 1.0f, 0.25f, 2.0f}, 
		//{18.0f, 10.00f, 3.3f, 0.0f, 0.25f, 1.0f},
		
		 // TURN_LEFT 
    {18.0f, 5.0f, 2.0f, 1.5f, 0.25f, 2.5f},
		//{22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 1.0f}, 
		
		// TURN_RIGHT
    {18.0f, 5.0f, 2.0f, 1.5f, 0.25f, 2.5f},       
		
		// WALK
		{18.5f, 6.5f, 4.5f, 0.8f, 0.40f, 2.5f}, 
   // {20.0f, 9.00f, 2.0f, 0.1f, 0.45f, 2.0f},   
    //目前最好值  12
		
		// TROT（低趴）
    {16.5f, 5.5f, 2.0f, 0.8f, 0.35f, 2.8f},   
		//{18.0f, 10.00f, 4.3f, 1.0f, 0.25f, 3.6f},TROT快
		
		
		// BRIDGE闪光黑子
    {19.0f, 8.00f, 2.5f, 0.5f, 0.3f, 1.0f}, 
                                            
		// STOP    
    {18.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},   
		
    //RELEASE
    {NAN, NAN, NAN, NAN, NAN, NAN},

    // JUMP_GROUND
    {15.0f, 0.0f, 6.0f, 0.0f, 0.0f, 4.0f},    
    
    // JUMP_UPHILL
    {15.0f, 0.0f, 6.0f, 0.0f, 0.0f, 4.0f},    
    
    // START
    {16.0f, 2.0f, 1.5f, 0.3f, 0.15f, 1.5f},
    
    // END
    {NAN, NAN, NAN, NAN, NAN, NAN},
};

//四腿独立参数（运行时）
DetachedParam detached_params;

//四腿独立参数预设表
DetachedParam state_detached_params[4] = {
    //TROT（索引0）
    {
    {18.0f, 5.5f, 2.0f, 0.8f, 0.35f, 2.8f}, //   {21.0, 12.0, 3.00, 2.00, 0.25, 4.0}
    {18.0f, 5.5f, 2.0f, 0.8f, 0.35f, 2.8f},
    {18.0f, 5.5f, 2.0f, 0.8f, 0.35f, 2.8f},
    {18.0f, 5.5f, 2.0f, 0.8f, 0.35f, 2.8f}
    },

    // CLIMBING（索引1）
    {
    {16.5f, 5.5f, 3.5f, 0.8f, 0.25f, 2.0f}, //小步子可以迈上去
    {16.5f, 5.5f, 3.5f, 0.8f, 0.25f, 2.0f},
    {17.5f, 7.0f, 4.5f, 1.0f, 0.25f, 2.0f},
    {17.5f, 7.0f, 4.5f, 1.0f, 0.25f, 2.0f}
    },

    // TURN_LEFT（索引2）
    {
        {17.5f, 3.5f, 2.0f, 1.0f, 0.25f, 2.5f},
        {18.5f, 6.5f, 2.0f, 1.5f, 0.25f, 2.5f},
        {17.5f, 3.5f, 2.0f, 1.0f, 0.25f, 2.5f},
        {18.5f, 6.5f, 2.0f, 1.5f, 0.25f, 2.5f}
    },

    // TURN_RIGHT（索引3）
    {
        {18.5f, 6.5f, 2.0f, 1.5f, 0.25f, 2.5f},
        {17.5f, 3.5f, 2.0f, 1.0f, 0.25f, 2.5f},
        {18.5f, 6.5f, 2.0f, 1.5f, 0.25f, 2.5f},
        {17.5f, 3.5f, 2.0f, 1.0f, 0.25f, 2.5f},
    },

};

//遥控调参默认值
DetachedParam RcDetachedParam = {
    {18.0f, 0.0f, 2.0f, 0.5f, 0.20f, 2.0f},
    {18.0f, 0.0f, 2.0f, 0.5f, 0.20f, 2.0f},
    {18.0f, 0.0f, 2.0f, 0.5f, 0.20f, 2.0f},
    {18.0f, 0.0f, 2.0f, 0.5f, 0.20f, 2.0f}};

//系统状态
Robohorse_State NowState = STOP; //当前运行状态
Robohorse_Direction NowDir = POSITIVE;

Stanford_Type_Lite_Data_t Gait_Data[4]; // 步态数据
Cycloid_Generator_t Cycloid_Gen[4]; // 摆线生成器
float Aim_Angle[8]; //目标角度，单位为弧度rad
volatile uint8_t Is_Aim_Angle_Get = 0; //是否已经获得到目标角度
float Last_Time = 0.0f; //上次计算时间
uint8_t Mode_Change_Flag = 1; //模式切换标志

// 1. 计算body_height
//float min_height = 12.0f;
//float max_height = 22.0f;
//float norm = (pRCData->rc.ch[4] - 400) / 1200.0f; // 0~1
//if (norm < 0) norm = 0;
//if (norm > 1) norm = 1;
//float body_height = min_height + norm * (max_height - min_height);

//// 2. 赋值给所有腿
//for (int i = 0; i < 4; i++) {
//    pCycGenerato[i].param.body_height = body_height;
//}
