#include "headfile.h"
int main(void)
{
	get_clk();
 	Init_ALL();	//启动全车初始化
/************************初始参数设置*****************************************/
	PID_Dis[1].Param_Kp=PID_Dis[0].Param_Kp;
	PID_Dis[1].Param_Kd=PID_Dis[0].Param_Kd;
	MECANUM_Motor_Data.Speed_All=1500;			//全车最大速度
	MPU_Data.Yaw_CloseLoop_Flag=1;					//默认开启偏航角闭环
	
	MECANUM_Motor_Data.Car_Coord_Now.x=0;		//设置初始坐标
	MECANUM_Motor_Data.Car_Coord_Now.y=0;
	MECANUM_Motor_Data.Car_Coord_Set.x=0;		//设置期望坐标
	MECANUM_Motor_Data.Car_Coord_Set.y=0;
/*****************************************************************/
	
	EnableInterrupts;//全车开始运行，以10ms控制周期进行控制
	
	while(1)
	{	
		//所有的控制代码在isr.c的PIN_INT1_DriverIRQHandler()外部中断服务函数内，中断由MPU9250的INT引脚控制，默认10ms周期
	}
}