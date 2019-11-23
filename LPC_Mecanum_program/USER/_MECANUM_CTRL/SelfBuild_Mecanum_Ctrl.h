#ifndef _SELFBUILD_MECANUM_CTRL_H_
#define _SELFBUILD_MECANUM_CTRL_H_

#include "common.h"

#define a_PARAMETER_cm 18
#define b_PARAMETER_cm 18

typedef enum
{
	Left_Front =0,
	Left_Back    ,
	Right_Front  ,
	Right_Back   ,
	
	Wheel_Sum,
}WheelNum_Typedef;

typedef struct
{
      int8 x;
      int8 y;
}INT8_XY;

typedef struct
{
      double x;
      double y;
}DOUBLE_XY;

extern struct MECANUM_Motor_Data_Typedef
{
	double SPEED_Set_cm_s[Wheel_Sum+1];
	double SPEED_Get_cm_s[Wheel_Sum+1];
	double SPEED_Get_cm_s_Last[Wheel_Sum+1];
	double PWM_Set[Wheel_Sum+1];
	double PWM_Mid;
	
	double Speed_All;
	DOUBLE_XY Speed;			//机械坐标
	
	DOUBLE_XY Speed_Real;		//相对地图坐标
	
	DOUBLE_XY Distance_Real;	//相对地图平移距离
	
	INT8_XY Car_Coord_Now,
					Car_Coord_Set;
	
	double Speed_GyroZ_Out;
	double Speed_GyroZ_Set;
	
	double Distance_Sum_cm[Wheel_Sum+1];
	double Distance_Aid[Wheel_Sum+1];
}MECANUM_Motor_Data;

extern uint8 Encoder_Pulse_TIMER[Wheel_Sum+1];
extern uint8 Encoder_Dir_Pin[Wheel_Sum+1];
extern uint8 Encoder_Dir_Set[Wheel_Sum+1];
extern uint8 Motor_PWM[2][Wheel_Sum+1];

void Wheel_Analysis(void);
void Wheel_Speed_Get_cm_s(void);
void Wheel_Speed_Real_Get(void);
void Wheel_Speed_PID(void);
void Motor_PWM_Set(int16 PWM_MAX);

#endif
