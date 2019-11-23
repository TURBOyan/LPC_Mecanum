#include "SelfBuild_control.h"
#include <string.h>

struct Base_Data_Typedef Base_Data;	//基础数据

void Init_ALL(void)		//全车初始化
{
//	Beep_Init;	//蜂鸣器
//	Servo_Init;	//磁铁舵机
//	Elema_Init(Elema_Mid);	//电磁铁
//	Elema_Init(Elema_Left); 
//	Elema_Init(Elema_Right);
	gpio_init(Button_Up,GPI,0,PULLUP);
	gpio_init(Button_Down,GPI,0,PULLUP);
	gpio_init(Button_Left,GPI,0,PULLUP);
	gpio_init(Button_Right,GPI,0,PULLUP);
	gpio_init(Button_Mid,GPI,0,PULLUP);
	gpio_init(Switch_1,GPI,0,PULLUP);
	gpio_init(Switch_2,GPI,0,PULLUP);
	gpio_init(Switch_3,GPI,0,PULLUP);	
	gpio_init(Switch_4,GPI,0,PULLUP);	
	
	memset(&Base_Data,0,sizeof(Base_Data));		//将所有会用到的结构体初始化为0
	memset(&MECANUM_Motor_Data,0,sizeof(MECANUM_Motor_Data));
	memset(&MPU_Data,0,sizeof(MPU_Data));
	MECANUM_Motor_Data.PWM_Mid=3750;				//电机PWM中值
	
	for(uint8 row=0;row<6;row++)//光电管初始化，管脚定义在SelfBuild_control.h内
	{
		for(uint8 col=0;col<6;col++)
		{
			if((uint8)(Gray[row][col])!=0xff)
			{
				gpio_init(Gray[row][col],GPI,0,PULLUP);	
			}
		}
	}
	

	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)		//电机相关初始化
	{
		ctimer_pwm_init(Motor_PWM[0][(uint8)num],250,MECANUM_Motor_Data.PWM_Mid);  //正转PWM
	}
	
	OLED_Init();
	while(MPU_Init_ForUser());	//初始化MPU9250，直到初始化完成

//  /*----------------------------菜单调参----------------------------------*/
	DisableInterrupts;                          //关闭所有中断，防止菜单调节过程中出现中断
	Menu_Init();                                  //初始化菜单
	while(!Menu_Work()) systick_delay_ms(200);    //菜单每200ms工作一次，并根据是否按下“关闭菜单”选项后（函数返回0）结束死循环
	
//	Elema_Absorb(Elema_Left);
//	Elema_Absorb(Elema_Right);
	
	View_MPUddata();
	
	pint_init(PINT_CH1, B0, FALLING);		//MPU9250的INT引脚连接在A3上，设置为下降沿触发
	set_irq_priority(PIN_INT1_IRQn,1);//设置优先级 越低优先级越高
	
	enable_irq(PIN_INT1_IRQn);		//开启引脚中断
	uart_rx_irq(USART_0,2); 	//蓝牙遥控中断开启
	
	DisableInterrupts;  
}

void DataSend(uint8 AllowFlag)		//置1位允许发送
{
	if(AllowFlag)
	{
		int16 data[4];
		data[0]=(int16)MECANUM_Motor_Data.Speed_X/100;
		data[1]=(int16)MECANUM_Motor_Data.Speed_Y/100;
		data[2]=(int16)MECANUM_Motor_Data.Speed_GyroZ_Out/7;
		data[3]=(int16)MPU_Data.Yaw;
		printf("{A%d:%d:%d:%d}$",data[0],data[1],data[2],data[3]);			//参数上传
	}
}


void Read_ButtSwitData(void)  //读取按键和拨码开关值
{
		Base_Data.Switch_Data=((~gpio_get(Switch_1)<<3)&0x08)  |	//拨码开关数据保存在Base_Data.Switch_Data结构体内
													((~gpio_get(Switch_2)<<2)&0x04)  |
													((~gpio_get(Switch_3)<<1)&0x02)  |
													(~gpio_get(Switch_4)&0x01);
		Base_Data.Button_Data=((~gpio_get(Button_Up)<<4)&0x10)    |//按键数据保存在Base_Data.Switch_Data结构体内
													((~gpio_get(Button_Down)<<3)&0x08)  |
													((~gpio_get(Button_Left)<<2)&0x04)  |
													((~gpio_get(Button_Right)<<1)&0x02) |
													(~gpio_get(Button_Mid)&0x01);
}


uint8 View_MPUddata(void)
{
  LED_Fill(0x00);
  LED_P6x8Str(0, 0, "Pitch=");
  LED_P6x8Str(0, 1, "Roll=");
  LED_P6x8Str(0, 2, "Yaw=");
	LED_P6x8Str(0, 3, "Yaw_MapZero=");
  while(1)
  {
		  Read_ButtSwitData();
			if(Query_ButtSwitData(Button_Data,Button_Up_Data))break;
		  Refresh_MPUTeam(DMP_MPL);
			if(Query_ButtSwitData(Button_Data,Button_Right_Data))MPU_Data.Yaw_Save=MPU_Data.Yaw;
			MPU_Data.Yaw_MapZero=Mpu_Normalization(MPU_Data.Yaw,MPU_Data.Yaw_Save);		
		  OLED_P6x8Flo(60, 0, MPU_Data.Pitch, -3);
		  OLED_P6x8Flo(60, 1, MPU_Data.Roll, -3);
		  OLED_P6x8Flo(60, 2, MPU_Data.Yaw, -3);
		  OLED_P6x8Flo(60, 3, MPU_Data.Yaw_MapZero, -3); 
  }
	for(uint8 i=0;i<=20;i++)
	{
		Refresh_MPUTeam(DMP_MPL);
		systick_delay_ms(10);
	}
	LED_Fill(0x00);
  return 0;
}

void MPU_Yaw_Closeloop(void)
{
		MPU_Data.Yaw_MapZero=Mpu_Normalization(MPU_Data.Yaw,MPU_Data.Yaw_Save);
		
		if(MPU_Data.Yaw_CloseLoop_Flag)		//如果角度闭环标志位被置位，则执行角度闭环
		{
			MPU_Data.Yaw_HeadZero=Mpu_Normalization(MPU_Data.Yaw_MapZero,MPU_Data.Yaw_MapZero_Save);	//偏航角重新将车头设为零点
			MECANUM_Motor_Data.Speed_GyroZ_Out=PID_Calcu(MPU_Data.Yaw_HeadZero_Aid,MPU_Data.Yaw_HeadZero,&PID_Dir,Local);	//角度闭环
		}
		else
		{
			MECANUM_Motor_Data.Speed_GyroZ_Out=MECANUM_Motor_Data.Speed_GyroZ_Set;
		}
		
//		OLED_P6x8Flo(60, 2, MPU_Data.Yaw_Aid, -3);
//		OLED_P6x8Flo(60, 3, MPU_Data.Yaw_Real, -3);
//		OLED_P6x8Flo(60, 4, MPU_Data.Yaw_Save, -3);
//		OLED_P6x8Int(0, 5, MECANUM_Motor_Data.Speed_GyroZ_Out, -5);
}

uint8 Distance_Coarse(int8* X_Now,int8* Y_Now,int8 X_Set,int8 Y_Set)
{
	static double Distance_SetX,Distance_SetY;
	static uint8 Continue_Flag=0;
	
	if(Continue_Flag==0										//当设定的坐标和此时的坐标发生变化时,开始粗调
		&&((*X_Now!=X_Set) || (*Y_Now!=Y_Set)))
	{
		Continue_Flag=1;
		MECANUM_Motor_Data.Distance_X_Real = 0;
		MECANUM_Motor_Data.Distance_Y_Real = 0;
		Distance_SetX = (X_Set-*X_Now)*50;
		Distance_SetY = (Y_Set-*Y_Now)*50;
	}
	
	if(	 Continue_Flag == 1					//当接近目标坐标时，停止粗调，返回1
		&&(MECANUM_Motor_Data.Distance_X_Real >Distance_SetX-1)
		&&(MECANUM_Motor_Data.Distance_X_Real <Distance_SetX+1)
		&&(MECANUM_Motor_Data.Distance_Y_Real >Distance_SetY-1)
		&&(MECANUM_Motor_Data.Distance_Y_Real <Distance_SetY+1)
		)
	{
		Continue_Flag =0;
		MECANUM_Motor_Data.Distance_X_Real = 0;
		MECANUM_Motor_Data.Distance_Y_Real = 0;
		MECANUM_Motor_Data.Speed_X_Real=0;
		MECANUM_Motor_Data.Speed_Y_Real=0;
		*X_Now=X_Set;
		*Y_Now=Y_Set;
		return 1;
	}
	
	if(Continue_Flag == 1)	//当允许粗调时，粗调一次，并返回0
	{
		MECANUM_Motor_Data.Speed_X_Real=10*PID_Calcu	(Distance_SetX,MECANUM_Motor_Data.Distance_X_Real,&PID_Dis[0],Local);
		MECANUM_Motor_Data.Speed_Y_Real=10*PID_Calcu	(Distance_SetY,MECANUM_Motor_Data.Distance_Y_Real,&PID_Dis[1],Local);

	//分别对相对地图的X、Y速度分量设置缓慢启动，缓慢停止，和限速
		if(fabs(MECANUM_Motor_Data.Distance_X_Real)<=10)		//地图X方向
		{
			MECANUM_Motor_Data.Speed_X_Real=RANGE(MECANUM_Motor_Data.Speed_X_Real,500,-500);
		}
		else if((Distance_SetX-MECANUM_Motor_Data.Distance_X_Real)>10)
		{
			MECANUM_Motor_Data.Speed_X_Real=RANGE(MECANUM_Motor_Data.Speed_X_Real,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
		}
		else
		{
			MECANUM_Motor_Data.Speed_X_Real=RANGE(MECANUM_Motor_Data.Speed_X_Real,500,-500);
		}
		
		if(fabs(MECANUM_Motor_Data.Distance_Y_Real)<=10)	//地图Y方向
		{
			MECANUM_Motor_Data.Speed_Y_Real=RANGE(MECANUM_Motor_Data.Speed_Y_Real,500,-500);
		}
		else if((Distance_SetY-MECANUM_Motor_Data.Distance_Y_Real)>10)
		{
			MECANUM_Motor_Data.Speed_Y_Real=RANGE(MECANUM_Motor_Data.Speed_Y_Real,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
		}
		else
		{
			MECANUM_Motor_Data.Speed_Y_Real=RANGE(MECANUM_Motor_Data.Speed_Y_Real,500,-500);
		}
			
	return 0;
	}
}