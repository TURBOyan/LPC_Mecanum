#include "headfile.h"

int main(void)
{
	get_clk();
 	Init_ALL();

	while(1)
	{

	}
}

//		qian=0;
//		hou=0;
//		Wheel_Speed_Real_Get();	//将速度转换为XY轴量并保存运动方向
//		Read_GrayData(0);			//光电管数据读取，不打印
//		for(uint8 x=0;x<6;x++)
//		{
//			if( Base_Data.Gray_Data_Last[0][x]==1
//				&&Base_Data.Gray_Data[0][x]==0)
//			{
//				Base_Data.Gray_Data_Rise[0][x]=(MECANUM_Motor_Data.Y_Dir==1?1:0);
//			}
//			qian+=Base_Data.Gray_Data_Rise[0][x];
//			OLED_P6x8Int(x*10 , 0, Base_Data.Gray_Data_Rise[0][x], 1);
//		}
//		
//		for(uint8 x=0;x<6;x++)
//		{
//			if( Base_Data.Gray_Data_Last[5][x]==0
//				&&Base_Data.Gray_Data[5][x]==1)
//			{
//				Base_Data.Gray_Data_Rise[5][x]=MECANUM_Motor_Data.Y_Dir==1?0:1;
//			}
//			hou+=Base_Data.Gray_Data_Rise[5][x];
//			OLED_P6x8Int(x*10 , 5, Base_Data.Gray_Data_Rise[5][x], 1);
//		}
//		if(Base_Data.Gray_Data[0][2] && Base_Data.Gray_Data[0][3] && Base_Data.Gray_Data[5][2] && Base_Data.Gray_Data[5][3])
//		{
//			qian=0;
//			hou=0;
//		}
//		
//		if(MECANUM_Motor_Data.Speed_X!=0 && qian>=2 && hou<=2)MECANUM_Motor_Data.Speed_Y=-15;
//		else if(MECANUM_Motor_Data.Speed_X!=0 && qian<=2 && hou>=2)MECANUM_Motor_Data.Speed_Y=15;
//		else MECANUM_Motor_Data.Speed_Y=0;
//		
//		OLED_P6x8Int(70 , 0, qian, 2);
//		OLED_P6x8Int(70 , 5, hou, 2);