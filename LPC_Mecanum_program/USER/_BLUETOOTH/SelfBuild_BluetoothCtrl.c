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
				case 'n': Elema_Unabsorb(Elema_Front);	break;//放左障碍
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
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//第二步，当走到（First_axis[Point]，-1）时，发送00+First_axis[Point]
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, First_axis[Point]+48);		//将整型数据转换为ASCALL码再发送
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
			&& Computer_communi.Data[4]==(First_axis[Point]+48)
			)
			{
				Computer_communi.Data_FinishReceive_Flag=0;
				continue_flag = 0;		
				Point++;
			}
			
			if(Computer_communi.Data_FinishReceive_Flag		//如果接收到错误数据，则重新接收
			&& Computer_communi.Data[0]!='O')
			{
				Computer_communi.Data_FinishReceive_Flag=0;
				Computer_communi.Timer=0;	//清零10ms计数器
				Computer_communi.Data_Point=0;		//清零指针
			}
		}
	}
	
	if(continue_flag == 3			//如果接收到下棋指令，则开始运行
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] == 'M' || Computer_communi.Data[0] == 'H' || Computer_communi.Data[0] == 'V'))
	{
		Point =0;
		continue_flag = 0;
		Computer_communi.Data_FinishReceive_Flag =0;
		MECANUM_Motor_Data.Car_RunPlayChess_Flag=1;
	}
	
	if(continue_flag == 3			//如果接收到错误数据，则重新接收
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] != 'M' && Computer_communi.Data[0] != 'H' && Computer_communi.Data[0] != 'V'))
	{
		Computer_communi.Data_FinishReceive_Flag=0;
		Computer_communi.Timer=0;	//清零10ms计数器
		Computer_communi.Data_Point=0;		//清零指针
	}
}

uint8 Chess_Move_M(void)			//棋子移动控制
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
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
		
		if(continue_flag == 2)		//吸棋子计时
		{
			Timer++;
		}
		
		if(continue_flag == 2
		&&Timer>=100
		)			//如果磁铁已经吸起，则准备移动至下一个目标
		{
			continue_flag = 21;
			Timer =0;
			Servo_Up;
		}
		if(continue_flag == 21)
		{
			Timer++;
		}
		
		if(continue_flag == 21
		&&Timer>=100)
		{
			continue_flag = 3;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//设置期望坐标
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 3					//到达预定地点过后，放下棋子，并保存当前棋子位置
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			Elema_Unabsorb(Elema_Mid);		//关闭电磁铁，释放棋子
		}
		
		if(continue_flag == 4)
		{
			Timer++;
		}
		
		if(continue_flag == 4
		&&Timer >= 200)
		{
			 continue_flag =5;
			 Timer =0;
			 MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;		//保存此时棋子的坐标
			 MECANUM_Motor_Data.Chess_Coord_Now.y=MECANUM_Motor_Data.Chess_Coord_Set.y;
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//指令车辆回到出发点
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 5
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//到达原点以后
		{
			 continue_flag =6;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		}
		
		if(continue_flag == 6)	//缓冲一会儿
		{
			Timer++;
		}
		
		if(continue_flag == 6
		&&Timer>=50)
		{
			 continue_flag =7;
			 MPU_Data.Yaw_CloseLoop_Flag = 0;//关闭角度闭环
			 Elema_Absorb(Elema_Right);//重新吸起障碍
			 Elema_Absorb(Elema_Front);//重新吸起障碍
		}
		
		
		if(continue_flag == 7
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))//等待上按键或者下按键按下执行下个回合
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//完成一次移动棋子操作，准备接受下次指令
			MPU_Data.Yaw_CloseLoop_Flag =1;//重新开启角度闭环
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//重新校准地图坐标
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;
		}
		return 0;
}

void WallCross_Push(void)	//横向障碍放置
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
		if(continue_flag == 0)		//设置移动到目标障碍位置
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//设置障碍期望坐标
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 1		//等到移动完成
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Elema_Unabsorb(Elema_Front);	//首先放下车前方障碍
		}
		
		if(continue_flag == 2)
		{
			Timer++;
		}
		if(continue_flag == 2
		&&Timer>=100			//如果障碍已经放下，则准备移动至下一个目标
		)			
		{
			continue_flag = 3;
			Timer =0;
			Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);			//左转90度

		}
		
		if(continue_flag == 3
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//当旋转完成
		)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x+1;		//向右方移动一格
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 4					//到达预定地点过后，放下障碍，并保存当前障碍位置
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 5;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		}
		
		if(continue_flag == 5)
		{
			Timer++;
		}
		
		if(continue_flag == 5
		&&Timer >= 50)
		{
			 continue_flag =6;
			 Timer =0;
			 Elema_Unabsorb(Elema_Right);		//关闭电磁铁，释放棋子
		}
		
		if(continue_flag == 6)
		{
			Timer++;
		}
		
		if(continue_flag == 6
		&& Timer>=50
		)
		{
			 continue_flag =7;
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//设置回到出发点
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 7
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//到达原点以后，关闭角度闭环，等待上按键按下执行下个回合
		{
			 continue_flag =8;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			 Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);		//右转90度
		}
		
		if(continue_flag == 8
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//当旋转完成
		)
		{
			 continue_flag =9;
		}
		
		if(continue_flag == 9)
		{
			Timer++;
		}
		
		if(continue_flag == 9
		&& Timer >= 100)			//1秒后关闭角度闭环
		{
			continue_flag =10;
			Timer=0;
			MPU_Data.Yaw_CloseLoop_Flag = 0;//关闭角度闭环
		  Elema_Absorb(Elema_Right);//重新吸起障碍
		  Elema_Absorb(Elema_Front);//重新吸起障碍
		}

		if(continue_flag == 10
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//完成一次移动棋子操作，准备接受下次指令
			
			MPU_Data.Yaw_CloseLoop_Flag =1;		//重新开启角度闭环
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//重新校准地图坐标
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;

		}
	
}

void WallVert_Push(void)		//竖向障碍放置
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
		if(continue_flag == 0)		//设置移动到目标障碍位置
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//设置障碍期望坐标
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 1		//等到移动完成
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Elema_Unabsorb(Elema_Right);	//首先放下车右方障碍
		}
		
		if(continue_flag == 2)
		{
			Timer++;
		}
		if(continue_flag == 2
		&&Timer>=100			//如果障碍已经放下，则准备移动至下一个目标
		)			
		{
			continue_flag = 3;
			Timer =0;
			Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);			//左转90度

		}
		
		if(continue_flag == 3
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//当旋转完成
		)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//向右方移动一格
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y+1;
		}
		
		if(continue_flag == 4					//到达预定地点过后，放下棋子，并保存当前棋子位置
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 5;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		}
		
		if(continue_flag == 5)
		{
			Timer++;
		}
		
		if(continue_flag == 5
		&&Timer >= 100)
		{
			 continue_flag =6;
			 Timer =0;
			 Elema_Unabsorb(Elema_Front);		//关闭电磁铁，释放棋子
		}
		
		if(continue_flag == 6)
		{
			Timer++;
		}
		
		if(continue_flag == 6
		&& Timer>=50
		)
		{
			 continue_flag =7;
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//设置回到出发点
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 7
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//到达原点以后，关闭角度闭环，等待上按键按下执行下个回合
		{
			 continue_flag =8;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			 Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);		//左转90度
		}
		
		if(continue_flag == 8
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//当旋转完成
		)
		{
			 continue_flag =9;
		}
		
		if(continue_flag == 9)
		{
			Timer++;
		}
		
		if(continue_flag == 9
		&& Timer >= 100)			//1秒后关闭角度闭环
		{
			continue_flag =10;
			Timer=0;
			MPU_Data.Yaw_CloseLoop_Flag = 0;//关闭角度闭环
		  Elema_Absorb(Elema_Right);//重新吸起障碍
		  Elema_Absorb(Elema_Front);//重新吸起障碍
		}

		if(continue_flag == 10
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//完成一次移动棋子操作，准备接受下次指令
			
			MPU_Data.Yaw_CloseLoop_Flag =1;		//重新开启角度闭环
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//重新校准地图坐标
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;

		}
	
}

void Chess_Ctrl_Auto(void)		//下棋总控
{
	MECANUM_Motor_Data.Chess_Coord_Set.x=Computer_communi.Data[1]-48;		//保存设置的棋子坐标
	MECANUM_Motor_Data.Chess_Coord_Set.y=Computer_communi.Data[2]-48;
	if(Computer_communi.Data[0] == 'M')
	{
		Chess_Move_M();			//棋子移动执行
	}
	
	if(Computer_communi.Data[0] == 'H')
	{
    WallCross_Push();		//横向障碍放置
	}
	
	if(Computer_communi.Data[0] == 'V')
	{
    WallVert_Push();		//竖向障碍放置
	}
}

void Chess_Ctrl_Manual(void)
{
	static uint16 continue_flag=0,count=0,Timer=0;
	if(continue_flag == 0)	//按键设置棋子坐标
	{
		count++;
		if(count >=20)
		{
			count=0;
			if(Query_ButtSwitData(Button_Data,Button_Up_Data))MECANUM_Motor_Data.Chess_Coord_Set.y++;
			if(Query_ButtSwitData(Button_Data,Button_Down_Data))MECANUM_Motor_Data.Chess_Coord_Set.y--;
			if(Query_ButtSwitData(Button_Data,Button_Right_Data))MECANUM_Motor_Data.Chess_Coord_Set.x++;
			if(Query_ButtSwitData(Button_Data,Button_Left_Data))MECANUM_Motor_Data.Chess_Coord_Set.x--;
			if(Query_ButtSwitData(Button_Data,Button_Mid_Data))
			{
				continue_flag = 1;
			}
		}
	}
	
	if(continue_flag == 1)				//第一步
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[0];		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
		
		MPU_Data.Yaw_CloseLoop_Flag =1;//重新开启角度闭环
		MPU_Data.Yaw_Save=MPU_Data.Yaw;	//重新校准地图坐标
		MPU_Data.Yaw_HeadZero_Aid=0;
		MPU_Data.Yaw_MapZero_Save=0;
	}
	
	if(continue_flag == 2
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//第二步
	{
		continue_flag = 31;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 31)			//延时
	{
		Timer++;
	}
	
	if(continue_flag == 31
	&& Timer>=100)
	{
		continue_flag = 3;
		Timer=0;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[1];		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}

	
	if(continue_flag == 3
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//第三步
	{
		continue_flag = 41;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 41)
	{
		Timer++;
	}
	
	if(continue_flag == 41
	&& Timer>=100)
	{
		continue_flag = 4;
		Timer=0;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[2];		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}
	
	if(continue_flag == 4
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//第四步
	{
		continue_flag = 51;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 51)
	{
		Timer++;
	}
	
	if(continue_flag == 51
	&& Timer>=200)
	{
		continue_flag = 6;
		Timer=0;
		MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Now.x;		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Now.y;
	}
	
	if(continue_flag == 6		//等到移动完成，则开始吸棋子
	&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
	{
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		continue_flag = 7;
		Servo_Down;		//降下中间电磁铁
		Elema_Absorb(Elema_Mid);	//电磁铁开启
	}
	
	if(continue_flag == 7)		//吸棋子计时
	{
		Timer++;
	}
	
	if(continue_flag == 7
	&&Timer>=100
	)			//如果磁铁已经吸起，则准备移动至下一个目标
	{
		continue_flag = 71;
		Timer =0;
		Servo_Up;
	}
	if(continue_flag == 71)
	{
		Timer++;
	}
	
	if(continue_flag == 71
	&&Timer>=100)
	{
		continue_flag = 8;
		MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//设置期望坐标
		MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
	}
	
	if(continue_flag == 8					//到达预定地点过后，放下棋子，并保存当前棋子位置
	&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
	{
		continue_flag = 9;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		Elema_Unabsorb(Elema_Mid);		//关闭电磁铁，释放棋子
	}
	
	if(continue_flag == 9)
	{
		Timer++;
	}
	
	if(continue_flag == 9
	&&Timer >= 200)
	{
		 continue_flag =10;
		 Timer =0;
		 MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;		//保存此时棋子的坐标
		 MECANUM_Motor_Data.Chess_Coord_Now.y=MECANUM_Motor_Data.Chess_Coord_Set.y;
		 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//指令车辆回到出发点
		 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
	}
	
	if(continue_flag == 10
	&& MECANUM_Motor_Data.Car_Arrive_Flag)	//到达原点以后
	{
		 continue_flag =11;
		 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 11)	//缓冲一会儿
	{
		Timer++;
	}
	
	if(continue_flag == 11
	&&Timer>=50)
	{
		 continue_flag =0;
		 MPU_Data.Yaw_CloseLoop_Flag = 0;//关闭角度闭环
	}
}