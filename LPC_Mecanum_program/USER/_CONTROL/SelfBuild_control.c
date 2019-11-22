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
	EnableInterrupts;
	
//	Elema_Absorb(Elema_Left);
//	Elema_Absorb(Elema_Right);
	
	View_MPUddata();
	
	pint_init(PINT_CH1, B0, FALLING);		//MPU9250的INT引脚连接在A3上，设置为下降沿触发
	set_irq_priority(PIN_INT1_IRQn,1);//设置优先级 越低优先级越高
	
	enable_irq(PIN_INT1_IRQn);		//开启引脚中断
	uart_rx_irq(USART_0,2); 	//蓝牙遥控中断开启
	
	DisableInterrupts;  
}

void DataSend(uint8 AllowFlag)
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
		if(Query_ButtSwitData(Button_Data,Button_Up_Data))		//如果按下上键，则重新校准地图坐标
		{
			MPU_Data.Yaw_Save=MPU_Data.Yaw;
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;
		}
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
