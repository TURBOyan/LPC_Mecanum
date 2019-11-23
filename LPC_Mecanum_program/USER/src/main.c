#include "headfile.h"
int main(void)
{
	get_clk();
 	Init_ALL();	//启动全车初始化
/************************初始参数设置*****************************************/
	PID_Dis[1].Param_Kp=PID_Dis[0].Param_Kp;
	PID_Dis[1].Param_Kd=PID_Dis[0].Param_Kd;
	MECANUM_Motor_Data.Speed_All=1500;			//全车速度
	MPU_Data.Yaw_CloseLoop_Flag=1;
//	MECANUM_Motor_Data.Distance_X_Real_Set=300;
//	MECANUM_Motor_Data.Distance_Y_Real_Set=250;
	
	MECANUM_Motor_Data.Car_Coord_Now.x=0;
	MECANUM_Motor_Data.Car_Coord_Now.y=0;
	MECANUM_Motor_Data.Car_Coord_Set.x=0;
	MECANUM_Motor_Data.Car_Coord_Set.y=0;
/*****************************************************************/
	
	EnableInterrupts;//全车开始运行，以10ms控制周期进行控制
	
	while(1)
	{
	}
}