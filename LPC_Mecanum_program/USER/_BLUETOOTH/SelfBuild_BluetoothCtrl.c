#include "SelfBuild_BluetoothCtrl.h"

struct Computer_communi_Typedef Computer_communi;

void Android_Ctrl(uint8 Data)
{
		static uint8 mode;
		if(mode)
		{
			switch(Data)
			{
				case 'A': MECANUM_Motor_Data.Speed_Real.x=0;		//前进
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All; break;
				
				case 'H': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All/1.4;		//左前
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All/1.4; break;
				
				case 'G': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All;	//左转
									MECANUM_Motor_Data.Speed_Real.y=0;   break;
				
				case 'F': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All/1.4;	//左后
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
				
				case 'E': MECANUM_Motor_Data.Speed_Real.x=0;		//后退
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All;break;

				case 'D': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All/1.4;	//右后
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
				
				case 'C': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All;		//右转
									MECANUM_Motor_Data.Speed_Real.y=0;   break;

				case 'B': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All/1.4;	//右前
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All/1.4;   break;
									
				case 'Z': MECANUM_Motor_Data.Speed_Real.x=0;		//停止
									MECANUM_Motor_Data.Speed_Real.y=0;
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;					break;	
									
				case 'X': Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);break;		//右转90度

				case 'Y': Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);break;//左转90度
									
				case 'p':	Elema_Unabsorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;break;//放棋子
				case 'o': Elema_Absorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;	break;//吸棋子
				case 'n': Elema_Unabsorb(Elema_Left);	break;//放左障碍
				case 'm': Elema_Unabsorb(Elema_Right);	break;//放右障碍
				
				case 'K': mode=0;
									MPU_Data.Yaw_CloseLoop_Flag=0; //关闭偏航角闭环
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;
									break;
				default : break;
			}
		}
		else
		{
			switch(Data)
			{
				case 'A': MECANUM_Motor_Data.Speed_Real.x=0;		//前进
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All; break;
				
				case 'E': MECANUM_Motor_Data.Speed_Real.x=0;		//后退
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All;break;

				case 'G': MECANUM_Motor_Data.Speed_GyroZ_Set=-100;	//顺时针
									break;
			
				case 'C': MECANUM_Motor_Data.Speed_GyroZ_Set=100;			//逆时针
									break;
				
				case 'Z': MECANUM_Motor_Data.Speed_Real.x=0;		//停止
									MECANUM_Motor_Data.Speed_Real.y=0;
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;			
									break;
									
				case 'X': MECANUM_Motor_Data.Speed_All=2000; break;		//高速

				case 'Y': MECANUM_Motor_Data.Speed_All=500;break;//低速
				
				case 'J': mode=1;
									MPU_Data.Yaw_CloseLoop_Flag=1; //开启偏航角闭环
									MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
									MPU_Data.Yaw_HeadZero_Aid=0;
									break;
				default : break;
			}
		}
}

void Uart_Receive_Ctrl(uint16 Timeout_10ms_num)
{
	static uint8 Data_Point_Save;
	if(Computer_communi.Data_FinishReceive_Flag == 0)		//如果还未接收完
	{
		if(Computer_communi.Data_Point == Data_Point_Save)	//如果没有新的一位数据接收进来，这次传入的指针值和上次的一样
		{
			Computer_communi.Timer++;	//记一次10ms
			if(Computer_communi.Timer == Timeout_10ms_num)	//如果未接收数据超过设定的时间，则判断为接收完成
			{
				Computer_communi.Timer=0;	//清零10ms计数器
				Computer_communi.Data_Point=0;
				Data_Point_Save=0;
				Computer_communi.Data_FinishReceive_Flag=1;		//置位接收完成标志位
			}
		}
		else
		{
			 Data_Point_Save=Computer_communi.Data_Point;		//如果有新的数据进来，则保存数据指针
		}
	}
}

extern uint8 First_axis[3];
void Chess_Before_Ctrl(void)
{
	static uint8 continue_flag=0,Point=0;
	
	if(continue_flag == 0
	&& MECANUM_Motor_Data.Car_RunPlayChess_Flag == 0)				//第一步，走到（First_axis[Point]，-1）
	{
		continue_flag = 1;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[Point];		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}
	
	if(continue_flag == 1
	&&MECANUM_Motor_Data.Car_Arrive_Flag)		//第二步，当走到（First_axis[Point]，-1）时，发送00+First_axis[Point]
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, First_axis[0]+48);		//将整型数据转换为ASCALL码再发送
	}
	
	if(continue_flag == 2)
	{
		if(Point==2)
		{
			continue_flag = 3;
		}
		else
		{
			if(Computer_communi.Data_FinishReceive_Flag		//如果接收到OK00?则走到下一个点
			&& Computer_communi.Data[0]=='O'
			&& Computer_communi.Data[1]=='K'
			&& Computer_communi.Data[2]=='0'
			&& Computer_communi.Data[3]=='0'
			&& Computer_communi.Data[4]==(First_axis[0]+48)
			)
			{
				continue_flag = 0;		
				Point++;
			}
		}
	}
	
	if(continue_flag == 3			//如果接收到下棋指令，则开始运行
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] == 'M' || Computer_communi.Data[0] == 'H' || Computer_communi.Data[0] == 'N'))
	{
		Point =0;
		continue_flag = 0;
		MECANUM_Motor_Data.Car_RunPlayChess_Flag=1;
	}
}

void Chess_Move_M(void)			//棋子移动控制
{
		static uint8 continue_flag=0;
		if(continue_flag == 0)		//设置移动到上次棋子位置
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Now.x;		//设置期望坐标
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Now.y;
		}
		
		if(continue_flag == 1		//等到移动完成，则开始吸棋子
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Servo_Down;		//降下中间电磁铁
			Elema_Absorb(Elema_Mid);	//电磁铁开启
		}
		
		if(continue_flag == 2
		&& Elema_Mid_Sensor_Read)			//如果磁铁已经吸起，则准备移动至下一个目标
		{
			continue_flag = 3;
			Servo_Up;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//设置期望坐标
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 3					//到达预定地点过后，放下棋子，并保存当前棋子位置
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			Elema_Unabsorb(Elema_Mid);		//关闭电磁铁，释放棋子
			MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;
			MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 4)
		{
			 continue_flag =0;
			 MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//完成一次移动棋子操作，准备接受下次指令
			 MECANUM_Motor_Data.Car_Coord_Set.x =-1;		//设置期望坐标
			 MECANUM_Motor_Data.Car_Coord_Set.y =-1;
		}
}

void Chess_Ctrl(void)		//下棋总控
{
	if(Computer_communi.Data[0] == 'M')
	{
		MECANUM_Motor_Data.Chess_Coord_Set.x=Computer_communi.Data[1]-48;		//保存设置的棋子坐标
		MECANUM_Motor_Data.Chess_Coord_Set.y=Computer_communi.Data[2]-48;
		Chess_Move_M();
	}
	
	if(Computer_communi.Data[0] == 'H')
	{
		
	}
}