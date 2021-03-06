#include "SelfBuild_control.h"
#include <string.h>

#define Speed_1 2000

struct Base_Data_Typedef Base_Data;	//基础数据

void Init_ALL(void)		//全车初始化
{
	Beep_Init;	//蜂鸣器
	Servo_Init;	//磁铁舵机
	Elema_Init(Elema_Mid);	//电磁铁
	Elema_Init(Elema_Front); 
	Elema_Init(Elema_Right);
	Elema_Mid_Sensor_Init;
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
			if((uint8)(Gray_Front[row][col])!=0xff)
			{
				gpio_init(Gray_Front[row][col],GPI,0,PULLUP);	
			}
		}
	}
	

	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)		//电机相关初始化
	{
		sct_pwm_init(Motor_PWM[1][(uint8)num],250,MECANUM_Motor_Data.PWM_Mid);  //正转PWM
	}
	
	OLED_Init();
	while(MPU_Init_ForUser());	//初始化MPU9250，直到初始化完成

//  /*----------------------------菜单调参----------------------------------*/
	DisableInterrupts;                          //关闭所有中断，防止菜单调节过程中出现中断
	Menu_Init();                                  //初始化菜单
	while(!Menu_Work()) systick_delay_ms(200);    //菜单每200ms工作一次，并根据是否按下“关闭菜单”选项后（函数返回0）结束死循环
	PID_Dis[1].Param_Kp=PID_Dis[0].Param_Kp;
	PID_Dis[1].Param_Kd=PID_Dis[0].Param_Kd;

	Elema_Absorb(Elema_Right);//吸起障碍
	Elema_Absorb(Elema_Front);
	
	LED_P6x8Str(20,  2, "Wait MPU6050......");
  while(MPU_Init_ForUser()){if(gpio_get(Button_Up) == 0){while(gpio_get(Button_Up) == 0){};break;}} 	//初始化MPL
  LED_CLS(); 
	View_MPUddata();
	
	pint_init(PINT_CH1, B14, FALLING);		//MPU9250的INT引脚连接在B14上，设置为下降沿触发
	set_irq_priority(PIN_INT1_IRQn,1);//设置优先级 越低优先级越高
	enable_irq(PIN_INT1_IRQn);		//开启引脚中断
	
	uart_rx_irq(USART_0,1); 	//蓝牙遥控中断开启
	set_irq_priority(FLEXCOMM0_IRQn,0);
	
	DisableInterrupts;  
}

void DataSend(uint8 AllowFlag)		//置1位允许发送
{
	if(AllowFlag)
	{
		int16 data[4];
		data[0]=(int16)MECANUM_Motor_Data.Speed.x/100;
		data[1]=(int16)MECANUM_Motor_Data.Speed.y/100;
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
}

uint8 Distance_Coarse(int8* X_Now,int8* Y_Now,int8 X_Set,int8 Y_Set)
{
		static double Distance_SetX,Distance_SetY;
		static int8 Continue_Flag_x=0,Continue_Flag_y=0,Return_Flag_x=0,Return_Flag_y=0;
		
		if(	Continue_Flag_x ==0
			&&(*X_Now!=X_Set)
			)
		{
			Continue_Flag_x=1;
			MECANUM_Motor_Data.Distance_Real.x = 0;
			Distance_SetX = (X_Set-*X_Now)*50;
		}
		
		if(Continue_Flag_x ==0
		&& X_Set==*X_Now
		&& Y_Set!=*Y_Now)
		{
			Continue_Flag_x=1;
			Return_Flag_x=1;
			MECANUM_Motor_Data.Distance_Real.x = 0;
			Distance_SetX=-15;
		}
		if(Continue_Flag_x == 1)	//当允许粗调时，开始粗调-x
		{
		//分别对相对地图的X速度分量设置缓慢启动，缓慢停止，和限速
			MECANUM_Motor_Data.Speed_Real.x=10*PID_Calcu	(Distance_SetX,MECANUM_Motor_Data.Distance_Real.x,&PID_Dis[0],Local);
			if(fabs(MECANUM_Motor_Data.Distance_Real.x)<=5)		//地图X方向//启动时
			{
				MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,Speed_1,-Speed_1);
			}
			else if(fabs(Distance_SetX-MECANUM_Motor_Data.Distance_Real.x)>=51)
			{
				MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
			}
			else//将要接近目标坐标时
			{
				MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,Speed_1,-Speed_1);
			}
		}
		
		if(	 Continue_Flag_x == 1					//当接近目标坐标时，且回调标志位未被置位，停止粗调，开始细调
		&& Return_Flag_x == 0
		&&(MECANUM_Motor_Data.Distance_Real.x >=Distance_SetX-30)
		&&(MECANUM_Motor_Data.Distance_Real.x <=Distance_SetX+30)
		)
		{
			Continue_Flag_x =2;
		}
		
			if(Continue_Flag_x == 2
		 &&Gray_Calibration_X((int16)Distance_SetX,Return_Flag_x)		//光电管细调
			)
		{
			Continue_Flag_x=3;
			Return_Flag_x=0;
			MECANUM_Motor_Data.Distance_Real.x = 0;
			MECANUM_Motor_Data.Speed_Real.x=0;
			*X_Now=X_Set;		//保存位置
		}
		
		/******************************************************/
		/******************************************************/
		if(	Continue_Flag_y ==0
			&&(*Y_Now!=Y_Set)
			)
		{
			Continue_Flag_y=1;
			MECANUM_Motor_Data.Distance_Real.y = 0;
			Distance_SetY = (Y_Set-*Y_Now)*50;
		}
		
		if(Continue_Flag_y ==0
		&& X_Set!=*X_Now
		&& Y_Set==*Y_Now)
		{
			Continue_Flag_y=1;
			Return_Flag_y=1;
			MECANUM_Motor_Data.Distance_Real.y = 0;
			Distance_SetY=-15;
		}
		if(Continue_Flag_y == 1)	//当允许粗调时，开始粗调-x
		{
		//分别对相对地图的X速度分量设置缓慢启动，缓慢停止，和限速
			MECANUM_Motor_Data.Speed_Real.y=10*PID_Calcu	(Distance_SetY,MECANUM_Motor_Data.Distance_Real.y,&PID_Dis[1],Local);
			if(fabs(MECANUM_Motor_Data.Distance_Real.y)<=5)		//地图X方向//启动时
			{
				MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,Speed_1,-Speed_1);
			}
			else if(fabs(Distance_SetY-MECANUM_Motor_Data.Distance_Real.y)>=51)
			{
				MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
			}
			else//将要接近目标坐标时
			{
				MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,Speed_1,-Speed_1);
			}
		}
		
		if(	 Continue_Flag_y == 1					//当接近目标坐标时，且回调标志位未被置位，停止粗调，开始细调
		&& Return_Flag_y == 0
		&&(MECANUM_Motor_Data.Distance_Real.y >=Distance_SetY-30)
		&&(MECANUM_Motor_Data.Distance_Real.y <=Distance_SetY+30)
		)
		{
			Continue_Flag_y =2;
		}
		
		
		if(Continue_Flag_y == 2
		 &&Gray_Calibration_Y((int16)Distance_SetY,Return_Flag_y)		//光电管细调
			)
		{
			Continue_Flag_y=3;
			Return_Flag_y=0;
			MECANUM_Motor_Data.Distance_Real.y = 0;
			MECANUM_Motor_Data.Speed_Real.y=0;
			*Y_Now=Y_Set;		//保存位置
		}
		/******************************************************/
		/******************************************************/
		/******************************************************/
		/******************************************************/
		if(	 Continue_Flag_x == 1					//当接近目标坐标时，且回调标志位置位，停止粗调，开始细调,
		&& Return_Flag_x == 1
		&&(MECANUM_Motor_Data.Distance_Real.y >=Distance_SetY-20)
		&&(MECANUM_Motor_Data.Distance_Real.y <=Distance_SetY+20)
		)
		{
			Continue_Flag_x =2;
		}
		
		if(	 Continue_Flag_y == 1					//当接近目标坐标时，且回调标志位置位，停止粗调，开始细调,
		&& Return_Flag_y == 1
		&&(MECANUM_Motor_Data.Distance_Real.x >=Distance_SetX-20)
		&&(MECANUM_Motor_Data.Distance_Real.x <=Distance_SetX+20)
		)
		{
			Continue_Flag_y =2;
		}
		
		
		
		
		if(Continue_Flag_x==3
		 &&Continue_Flag_y==3)
		{
			Continue_Flag_x=0;
			Continue_Flag_y=0;
	 		MECANUM_Motor_Data.Car_Arrive_Flag = 1;			//坐标移动完成
		}

}