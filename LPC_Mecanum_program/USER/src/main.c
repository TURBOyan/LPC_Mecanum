#include "headfile.h"

int main(void)
{
	get_clk();
 	Init_ALL();	//启动全车初始化
	
/*****************************************************************/
	MECANUM_Motor_Data.Speed_All=2000;	//初始参数设置
	MECANUM_Motor_Data.Speed_X=0;
	MECANUM_Motor_Data.Speed_Y=0;
	MECANUM_Motor_Data.Speed_GyroZ_Set=0;
	MECANUM_Motor_Data.PWM_Mid=3750;
	MPU_Data.Yaw_CloseLoop_Flag=1;
/*****************************************************************/
	
	EnableInterrupts;//全车开始运行，以10ms控制周期进行控制
	
	while(1)
	{
	}
}