#include "SelfBuild_Mecanum_Ctrl.h"
#include "math.h"
#include "headfile.h"

struct MECANUM_Motor_Data_Typedef MECANUM_Motor_Data;

float Speed_Ratio=(3.7*2*3.1415926)/(1024.0*18.0*0.01);	//(半径'cm'*半径'cm'*pai)/(编码器线数*减速比*速度采样周期)

uint8 Encoder_Pulse_TIMER[Wheel_Sum+1]={TIMER3_COUNT0_A4,TIMER1_COUNT0_A16,TIMER0_COUNT0_A1,TIMER4_COUNT0_A15};//编码器计数时钟
uint8 Encoder_Dir_Pin[Wheel_Sum+1]		={	      B29			 ,        B8			,        B14			,        B21     };//编码器方向口
uint8 Encoder_Dir_Set[Wheel_Sum+1]		={				0        ,         0      ,        1        ,         1      };
#ifdef old_PCB
	uint8 Motor_PWM[2][Wheel_Sum+1]				={
																				 {TIMER2_PWMCH1_B4    ,TIMER2_PWMCH0_B5  ,TIMER0_PWMCH1_A3    ,TIMER2_PWMCH2_B7}		//正转
																				,{SCT0_OUT8_A29    ,SCT0_OUT6_A27    ,SCT0_OUT9_A30    ,SCT0_OUT5_A26   }		//倒转
																				 };						
#endif
																			 
#ifdef new_PCB
	uint8 Motor_PWM[2][Wheel_Sum+1]				={
																				 {TIMER2_PWMCH1_B4    ,TIMER2_PWMCH0_B5  ,TIMER0_PWMCH1_A3    ,TIMER2_PWMCH2_B7}		//正转
																				,{SCT0_OUT3_A22    ,SCT0_OUT6_A27    ,SCT0_OUT4_A23    ,SCT0_OUT7_A28   }		//倒转
																				 };						
#endif																			 
																			 
/**************************************************************************
函数功能：根据期望的XY轴速度和旋转角速度得出每个轮子的速度,X型安装，最后将此函数周期调用
入口参数：无
返回  值：无
**************************************************************************/
void Wheel_Analysis(void)
{
		MECANUM_Motor_Data.Speed_GyroZ_Out=RANGE(MECANUM_Motor_Data.Speed_GyroZ_Out,100,-100);	//限幅
	
		MECANUM_Motor_Data.Speed.x=MECANUM_Motor_Data.Speed_Real.x*cos(MPU_Data.Yaw_MapZero*0.0174533)	//将地图坐标转换为机械坐标
															+MECANUM_Motor_Data.Speed_Real.y*sin(MPU_Data.Yaw_MapZero*0.0174533-3.1415926);
	
		MECANUM_Motor_Data.Speed.y=MECANUM_Motor_Data.Speed_Real.x*sin(MPU_Data.Yaw_MapZero*0.0174533)
															+MECANUM_Motor_Data.Speed_Real.y*cos(MPU_Data.Yaw_MapZero*0.0174533);
	
		MECANUM_Motor_Data.SPEED_Set_cm_s[Right_Front]  = -(int16)MECANUM_Motor_Data.Speed.x + (int16)MECANUM_Motor_Data.Speed.y-(int16)MECANUM_Motor_Data.Speed_GyroZ_Out*(a_PARAMETER_cm+b_PARAMETER_cm);
		MECANUM_Motor_Data.SPEED_Set_cm_s[Left_Front]   = +(int16)MECANUM_Motor_Data.Speed.x + (int16)MECANUM_Motor_Data.Speed.y+(int16)MECANUM_Motor_Data.Speed_GyroZ_Out*(a_PARAMETER_cm+b_PARAMETER_cm);
		MECANUM_Motor_Data.SPEED_Set_cm_s[Left_Back]    = -(int16)MECANUM_Motor_Data.Speed.x + (int16)MECANUM_Motor_Data.Speed.y+(int16)MECANUM_Motor_Data.Speed_GyroZ_Out*(a_PARAMETER_cm+b_PARAMETER_cm);
		MECANUM_Motor_Data.SPEED_Set_cm_s[Right_Back]   = +(int16)MECANUM_Motor_Data.Speed.x + (int16)MECANUM_Motor_Data.Speed.y-(int16)MECANUM_Motor_Data.Speed_GyroZ_Out*(a_PARAMETER_cm+b_PARAMETER_cm);
	
		MECANUM_Motor_Data.Distance_Real.x+=MECANUM_Motor_Data.Speed_Real.x*0.00020668372;	//Ratio=(pi*d)/19*60*100
		MECANUM_Motor_Data.Distance_Real.y+=MECANUM_Motor_Data.Speed_Real.y*0.00020668372;
}

/**************************************************************************
函数功能：获取四个轮子的速度，转化为单位cm/s，此处待编写，最后将此函数周期调用,放在Wheel_Analysis之后
入口参数：无
返回  值：无
**************************************************************************/
void Wheel_Speed_Get_cm_s(void)	
{
		for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)
		{
			MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num]=ctimer_count_read(Encoder_Pulse_TIMER[(uint8)num]);
			if(gpio_get(Encoder_Dir_Pin[(uint8)num]) == Encoder_Dir_Set[(uint8)num]) 
				MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num] = - MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num]; 
			ctimer_count_clean(Encoder_Pulse_TIMER[(uint8)num]);
			
			MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num] = (int16)(MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num]*0.8+MECANUM_Motor_Data.SPEED_Get_cm_s_Last[(uint8)num]*0.2);
			MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num]*=Speed_Ratio;//转换为mm/s
			MECANUM_Motor_Data.Distance_Sum_cm[(uint8)num]+=MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num]*0.01;
			
			MECANUM_Motor_Data.SPEED_Get_cm_s_Last[(uint8)num]=MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num];
		}
}

void Wheel_Speed_Real_Get(void)
{
//	MECANUM_Motor_Data.Speed_Real.x=(-MECANUM_Motor_Data.SPEED_Get_cm_s[Right_Front]
//																	+MECANUM_Motor_Data.SPEED_Get_cm_s[Left_Front]
//																	-MECANUM_Motor_Data.SPEED_Get_cm_s[Left_Back]
//																	+MECANUM_Motor_Data.SPEED_Get_cm_s[Right_Back])/4.0;
//	MECANUM_Motor_Data.Speed_Real.y=(MECANUM_Motor_Data.SPEED_Get_cm_s[Right_Front]
//																	+MECANUM_Motor_Data.SPEED_Get_cm_s[Left_Front]
//																	+MECANUM_Motor_Data.SPEED_Get_cm_s[Left_Back]
//																	+MECANUM_Motor_Data.SPEED_Get_cm_s[Right_Back])/4.0;
//	MECANUM_Motor_Data.X_Dir=MECANUM_Motor_Data.Speed_Real.x<0?-1:(MECANUM_Motor_Data.Speed_Real.x==0?0:1);	//保存运动方向
//	MECANUM_Motor_Data.Y_Dir=MECANUM_Motor_Data.Speed_Real.y<0?-1:(MECANUM_Motor_Data.Speed_Real.y==0?0:1);
}

/**************************************************************************
函数功能：速度闭环，转化为单位cm/s，最后将此函数周期调用,放在Wheel_Speed_Get_cm_s()之后
入口参数：无
返回  值：无
**************************************************************************/
void Wheel_Speed_PID(void)
{
	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)
	{
		MECANUM_Motor_Data.PWM_Set[(uint8)num]+= PID_Calcu	(MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num],  
																								 MECANUM_Motor_Data.SPEED_Get_cm_s[(uint8)num],
																								 &PID_Speed,
																								 Incre);
	}
}
/**************************************************************************
函数功能：PWM设定，转化为单位cm/s，PWM设定值部分待编写，最后将此函数周期调用,放在Wheel_Speed_PID()之后
入口参数：PWM_MAX---PWM限幅上下限
返回  值：无
**************************************************************************/
void Motor_PWM_Set(int16 PWM_MAX)	//PWM输出
{
	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)
	{
		MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]=(int16)(MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]*0.125);
		MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]=RANGE(MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num],1000,-1000);		//PWM限幅
		MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]+=MECANUM_Motor_Data.PWM_Mid
																									+((MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]<0)?-50:(MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]>0?50:0));
		sct_pwm_duty(Motor_PWM[1][(uint8)num],(uint16)MECANUM_Motor_Data.SPEED_Set_cm_s[(uint8)num]);
	}
}

void Car_Turn_Right_90(Car_Dir_Typedef* Car_Dir_Mode)		//全车右转90度，并保存车身方向
{
	if(*Car_Dir_Mode == Left)
		*Car_Dir_Mode = Front;
	else
		*Car_Dir_Mode += 1;
	
	MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
	MPU_Data.Yaw_HeadZero_Aid=90;
}

void Car_Turn_Left_90(Car_Dir_Typedef* Car_Dir_Mode)		//全车左转90度，并保存车身方向
{
	if(*Car_Dir_Mode == Front)
		*Car_Dir_Mode = Left;
	else
		*Car_Dir_Mode -= 1;
	
	MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
	MPU_Data.Yaw_HeadZero_Aid=-90;
}