#ifndef GAIT_PARAM_H
#define GAIT_PARAM_H

#include <math.h>
#include "stanford_type.h"
#include "cycloid_generator.h"

#define pi (3.14159265f) // 圆周率

#define ReductionAndAngleRatio (515.6620162069706f) // 9/1/2/pi*360 编码器值与弧度之比（没看懂，理解为减速比/1弧度）

//单位都为cm(不确定)
#define L1 (39.6f)          //前后腿的距离
#define init_h 18.0f        //初始高度
#define FORWARD_hight 15.0f //前进高度
#define BACK_hight 15.0f    //后退高度

#define shortK_hight 18.0f  //短腿模式高度（屈膝）
#define longK_hight   14.0f //长腿模式高度（伸展）
// 机器马运动模式
typedef enum
{
    NORMAL = 0,  //常规/待机状态

    CLIMBING,    //爬坡模式
    TURN_LEFT,   //左转
    TURN_RIGHT,  //右转
    WALK,        //行走（慢速）
    TROT,        //小跑（快速）
    BRIDGE,      //桥接/过渡模式

    STOP,        //停止
    RELEASE,     //释放/放松（断电保护姿态）
    JUMP_GROUND, //平地跳跃
    JUMP_UPHILL, //上坡跳跃
    START,       //启动
    END,         //结束
} Robohorse_State;

typedef enum
{
    POSITIVE,
    NEGATIVE
} Robohorse_Direction;

// 腿部PID增益结构体
typedef struct
{
    float kp_pos; //位置环
    float kd_pos;

    float kp_spd; //速度环
    float kd_spd;
} LegGain;

// 腿部摆动轨迹生成器参数
typedef struct
{
    Cycloid_Generator_Param_t detached_params_0;
    Cycloid_Generator_Param_t detached_params_1;
    Cycloid_Generator_Param_t detached_params_2;
    Cycloid_Generator_Param_t detached_params_3;
} DetachedParam;

extern LegGain Gait_Gains;                                  //步态增益（PID）
extern LegGain state_gait_gains[END+1];                     //运行状态PID参数
extern Cycloid_Generator_Param_t gait_params;               //当前步态参数
extern Cycloid_Generator_Param_t state_gait_params[END+1];  //各个状态的步态参数
extern DetachedParam detached_params;                       //摆动轨迹生成器参数
extern DetachedParam state_detached_params[4];              //摆动轨迹生成器（数据分离）参数
extern DetachedParam RcDetachedParam;
extern Robohorse_State NowState;                            //当前运行状态
extern Robohorse_Direction NowDir;
extern Stanford_Type_Lite_Data_t Gait_Data[4];              //步态数据
extern Cycloid_Generator_t Cycloid_Gen[4];                  //轨迹生成器结构体
extern float Aim_Angle[8];                                  //目标角度
extern volatile uint8_t Is_Aim_Angle_Get;                   //是否获得目标角度
extern float Last_Time;                                     //上一次运算时间
extern uint8_t Mode_Change_Flag;                            //模式切换标志

#endif
