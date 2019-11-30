#include "Selfbuild_GrayCtrl.h"

#ifdef old_PCN
PIN_enum Gray[6][6]={		//光电管位置（第一行为车头）
{B31, A0, B11, B15 , A2,B30},
{A6,0xff,0xff,0xff,0xff,B16},
{A19,0xff,0xff,0xff,0xff,A5},
{A10 ,0xff,0xff,0xff,0xff,A18},	
{B20,0xff,0xff,0xff,0xff,A17},
{B19,B18 , B6 ,A21 , A20 ,B28},
};
#endif

#ifdef new_PCB
PIN_enum Gray_Front[6][6]={		//光电管位置（第一行为正前方）		//车头朝向为前时
{B31, A0, B11, A2 ,B17 ,B15},
{A6,0xff,0xff,0xff,0xff,B16},
{A19,0xff,0xff,0xff,0xff,A5},
{A10 ,0xff,0xff,0xff,0xff,A18},	
{B20,0xff,0xff,0xff,0xff,A17},
{B19,B18 , B6 ,A21 , A20 ,B28},
};

PIN_enum Gray_Left[6][6]={		//光电管位置（第一行为正前方）	//车头朝向为左时
{B15, B16, A5, A18 ,A17 ,B28},
{B17,0xff,0xff,0xff,0xff,A20},
{A2,0xff,0xff,0xff,0xff,A21},
{B11 ,0xff,0xff,0xff,0xff,B6},	
{A0,0xff,0xff,0xff,0xff,B18},
{B31,A6 , A19 ,A10 , B20 ,B19},
};

PIN_enum Gray_Right[6][6]={		//光电管位置（第一行为正前方）	//车头朝向为右时
{B19, B20, A10, A19 ,A6 ,B31},
{B18,0xff,0xff,0xff,0xff,A0},
{B6,0xff,0xff,0xff,0xff,B11},
{A21 ,0xff,0xff,0xff,0xff,A2},	
{A20,0xff,0xff,0xff,0xff,B17},
{B28,A17 , A18 ,A5 , B16 ,B15},
};
#endif

void Read_GrayData(uint8 x,uint8 y,uint8 showflag)	//读取光电管值并设定是否显示
{
	if(showflag)
	{
		if(x>92)x=92;
		if(y>2)y=2;
	}
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			if(MECANUM_Motor_Data.Car_Dir_Mode == Front)	
			{
				if((uint8)(Gray_Front[row][col])!=0xff)
				{
					Base_Data.Gray_Data[row][col]=gpio_get(Gray_Front[row][col]);
					if(showflag)OLED_P6x8Int(col*6+x, row+y, Base_Data.Gray_Data[row][col], 1);
				}
			}
			
			if(MECANUM_Motor_Data.Car_Dir_Mode == Left)	
			{
				if((uint8)(Gray_Left[row][col])!=0xff)
				{
					Base_Data.Gray_Data[row][col]=gpio_get(Gray_Left[row][col]);
					if(showflag)OLED_P6x8Int(col*6+x, row+y, Base_Data.Gray_Data[row][col], 1);
				}
			}
			
			if(MECANUM_Motor_Data.Car_Dir_Mode == Right)
			{
				if((uint8)(Gray_Right[row][col])!=0xff)
				{
					Base_Data.Gray_Data[row][col]=gpio_get(Gray_Right[row][col]);
					if(showflag)OLED_P6x8Int(col*6+x, row+y, Base_Data.Gray_Data[row][col], 1);
				}
			}
		}
	}
}

#define Gray_Aid_Num 5

uint8 Gray_Calibration_X(int16 X_Dir,int8 Return_Flag)
{
	static uint16 Continue_flag;
	uint8 Gray_xleft[6]	 	 ,Gray_xRight[6],
				Gray_xleft_sum=0 ,Gray_xRight_sum=0;
	
	int8 X_Dir_judge;	//保存运动方向
	for(uint8 num=0;num<6;num++)
	
	{
		Gray_xleft[num]=Base_Data.Gray_Data[num][0];	//保存左侧光电管值
		Gray_xleft_sum+=Gray_xleft[num];
		
		Gray_xRight[num]=Base_Data.Gray_Data[num][5];	//保存右侧光电管值
		Gray_xRight_sum+=Gray_xRight[num];
	}
	
	if(Gray_xleft_sum  >= Gray_Aid_Num
	 &&Gray_xRight_sum >= Gray_Aid_Num
		)
	{
		Continue_flag = 0;
		MECANUM_Motor_Data.Speed_Real.x=0;
		return 1;
	}
	X_Dir_judge=X_Dir>0?1:(X_Dir<0?-1:0);
	
	if(Return_Flag == 1)X_Dir_judge=-X_Dir_judge;
	
	if(Continue_flag == 0)
		MECANUM_Motor_Data.Speed_Real.x=500*X_Dir_judge;
	
	if(Continue_flag == 0
		&&(Gray_xleft_sum  >= Gray_Aid_Num ||Gray_xRight_sum  >= Gray_Aid_Num))
	{
		Continue_flag = 1;
		MECANUM_Motor_Data.Speed_Real.x=200*X_Dir_judge;
	}
	
	if( Continue_flag == 1
	 &&	Gray_xleft_sum  >= Gray_Aid_Num
	 && Gray_xRight_sum >= Gray_Aid_Num
	)
	{
		Continue_flag = 0;
		MECANUM_Motor_Data.Speed_Real.x=0;
		return 1;
	}

	return 0;
	
}


uint8 Gray_Calibration_Y(int16 Y_Dir,int8 Return_Flag)
{
	static uint16 Continue_flag=0;
	uint8 Gray_yUp[6]	 	 	 ,Gray_yDown[6],
				Gray_yUp_sum=0	 ,Gray_yDown_sum=0;
	
	int8 Y_Dir_judge;	//保存运动方向
	
	for(uint8 num=0;num<6;num++)
	{
		Gray_yUp[num]=Base_Data.Gray_Data[0][num];	//保存上侧光电管值
		Gray_yUp_sum+=Gray_yUp[num];
		
		Gray_yDown[num]=Base_Data.Gray_Data[5][num];	//保存下侧光电管值
		Gray_yDown_sum+=Gray_yDown[num];
	}
	
	if(Gray_yUp_sum    >= Gray_Aid_Num
	 &&Gray_yDown_sum  >= Gray_Aid_Num)
	{
		Continue_flag = 0;
		MECANUM_Motor_Data.Speed_Real.y=0;
		return 1;
	}

	Y_Dir_judge=Y_Dir>0?1:(Y_Dir<0?-1:0);
	
	if(Return_Flag ==1)Y_Dir_judge=-Y_Dir_judge;
	
	if(Continue_flag == 0)
		MECANUM_Motor_Data.Speed_Real.y=500*Y_Dir_judge;
	
	if(Continue_flag == 0
		&&(Gray_yUp_sum  >= Gray_Aid_Num ||Gray_yDown_sum  >= Gray_Aid_Num))
	{
		Continue_flag = 1;
		MECANUM_Motor_Data.Speed_Real.y=200*Y_Dir_judge;
	}
	
	if( Continue_flag == 1
	 &&	Gray_yUp_sum  >= Gray_Aid_Num
	 && Gray_yDown_sum >= Gray_Aid_Num
	)
	{
		Continue_flag = 0;
		MECANUM_Motor_Data.Speed_Real.y=0;
		return 1;
	}
	return 0;
}