#include "Selfbuild_GrayCtrl.h"


PIN_enum Gray[6][6]={		//光电管位置（第一行为车头）
{B31, A0, B11, B15 , A2,B30},
{A6,0xff,0xff,0xff,0xff,B16},
{A19,0xff,0xff,0xff,0xff,A5},
{A10 ,0xff,0xff,0xff,0xff,A18},	
{B20,0xff,0xff,0xff,0xff,A17},
{B19,B18 , B6 ,A21 , A20 ,B28},
};

//PIN_enum Gray[6][6]={		//光电管位置（第一行为车头）
//{B31 , A0 , B11,0xff,0xff,0xff},
//{A6  ,0xff,0xff,0xff,0xff,0xff},
//{A19 ,0xff,0xff,0xff,0xff,0xff},
//{0xff,0xff,0xff,0xff,0xff, A18},	
//{0xff,0xff,0xff,0xff,0xff, A17},
//{0xff,0xff,0xff,A21 ,A20 , B28},
//};

void Gray_RotationClockwise90(PIN_enum a[6][6])		//将6x6数组顺时针旋转90度
{
	uint8 offect,first,last;
	PIN_enum top;
	for(uint8 layer=0;layer< 6/2;layer++)
	{
		first = layer;
		last = 6-1 - layer;
		for(uint8 i = first;i<last;i++)
		{
			offect = i - first;
			top = a[first][i];
			a[first][i] = a[last-offect][first];
			a[last-offect][first] = a[last][last - offect];	
			a[last][last - offect] = a[i][last]; 
			a[i][last] = top;
		}
	}
}

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
			if((uint8)(Gray[row][col])!=0xff)
			{
				Base_Data.Gray_Data[row][col]=gpio_get(Gray[row][col]);
				if(showflag)OLED_P6x8Int(col*6+x, row+y, Base_Data.Gray_Data[row][col], 1);
			}
		}
	}
}

void Save_GrayData(uint8 data_new[6][6],uint8 data_save[6][6])	//保存光电管值
{
	
	
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			data_save[row][col]=data_new[row][col];
		}
	}
}

void Judge_GrayData(void)	//跳变点检测
{
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			if(Base_Data.Gray_Data[row][col]<Base_Data.Gray_Data_Last[row][col])//如果上次光电管判断到白线而这次没有则下降沿置一
				Base_Data.Gray_Data_Fall[row][col]=1;
			else if(Base_Data.Gray_Data[row][col]>Base_Data.Gray_Data_Last[row][col])//如果这次光电管判断到白线而上次没有则上升沿置一
				Base_Data.Gray_Data_Rise[row][col]=1;
			else
			{
				Base_Data.Gray_Data_Fall[row][col]=0;
				Base_Data.Gray_Data_Rise[row][col]=0;
			}
		}
	}
}

uint8 Gray_Calibration(void)
{
	static uint16 Continue_flag;
	uint8 Gray_upleft[5]	 ,Gray_upleft_sum,
				Gray_dowmright[5],Gray_dowmright_sum;
	
	Gray_upleft[0]=Base_Data.Gray_Data[2][0];		//保存左上角光电管值
	Gray_upleft[1]=Base_Data.Gray_Data[1][0];
	Gray_upleft[2]=Base_Data.Gray_Data[0][0];
	Gray_upleft[3]=Base_Data.Gray_Data[0][1];
	Gray_upleft[4]=Base_Data.Gray_Data[0][2];
	
	Gray_dowmright[0]=Base_Data.Gray_Data[3][5];	//保存右下角光电管值
	Gray_dowmright[1]=Base_Data.Gray_Data[4][5];
	Gray_dowmright[2]=Base_Data.Gray_Data[5][5];
	Gray_dowmright[3]=Base_Data.Gray_Data[5][4];
	Gray_dowmright[4]=Base_Data.Gray_Data[5][3];
	
	for(uint8 num=0;num<6;num++)				//分别保存左上角和右下角光电管状态总值
	{
		Gray_upleft_sum    +=Gray_upleft[num];
		Gray_dowmright_sum +=Gray_dowmright[num];
	}
	
	if(Gray_upleft_sum == 5 && Gray_dowmright_sum == 5)
	{
		Continue_flag=0;
		return 1;
	}
//		MECANUM_Motor_Data.Speed_Real.x=0;
//		MECANUM_Motor_Data.Speed_Real.y=0;
	if(	Continue_flag==0
		&&Gray_upleft_sum>=Gray_dowmright_sum)
	{
		Continue_flag=11;

	}
}