#include "headfile.h"


uint8 First_axis[3]={1,5,3};

int main(void)
{
	get_clk();
 	Init_ALL();	//启动全车初始化
	
/************************初始参数设置*****************************************/
	MECANUM_Motor_Data.Speed_All=1500;			//全车最大速度
	MPU_Data.Yaw_CloseLoop_Flag=1;					//默认开启偏航角闭环
	
	MECANUM_Motor_Data.Car_Begin.x=0;			//设置出发坐标
	MECANUM_Motor_Data.Car_Begin.y=-1;
	
	MECANUM_Motor_Data.Chess_Coord_Now.x=3;		//设置棋子初始坐标
	MECANUM_Motor_Data.Chess_Coord_Now.y=0;

	MECANUM_Motor_Data.Car_Dir_Mode=Front;	//设置初始车体朝向
	
	//以上为用户设置部分,以下不允许修改
	
	MECANUM_Motor_Data.Chess_Coord_Set.x=MECANUM_Motor_Data.Chess_Coord_Now.x;		//设置棋子期望坐标
	MECANUM_Motor_Data.Chess_Coord_Set.y=MECANUM_Motor_Data.Chess_Coord_Now.y;
	MECANUM_Motor_Data.Car_Coord_Now.x =MECANUM_Motor_Data.Car_Begin.x;		//设置初始坐标
	MECANUM_Motor_Data.Car_Coord_Now.y =MECANUM_Motor_Data.Car_Begin.y;
	MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//设置期望坐标
	MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
/*****************************************************************/
	
	EnableInterrupts;//全车开始运行，以10ms控制周期进行控制
	
	while(1)
	{	
		//所有的控制代码在isr.c的PIN_INT1_DriverIRQHandler()外部中断服务函数内，中断由MPU9250的INT引脚控制，默认10ms周期
	}
}