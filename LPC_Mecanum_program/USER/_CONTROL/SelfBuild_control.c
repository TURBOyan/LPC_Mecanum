#include "SelfBuild_control.h"
#include <string.h>

struct Base_Data_Typedef Base_Data;	//��������

PIN_enum Gray[6][6]={		//����λ�ã���һ��Ϊ��ͷ��
{B31, A0, B11, B15 , A2,B30},
{A6,0xff,0xff,0xff,0xff,B16},
{A19,0xff,0xff,0xff,0xff,A5},
{A10 ,0xff,0xff,0xff,0xff,A18},	
{B20,0xff,0xff,0xff,0xff,A17},
{B19,B18 , B6 ,A21 , A20 ,B28},
};

void Init_ALL(void)
{
	Beep_Init;	//������
	Servo_Init;	//�������
	Elema_Init(Elema_Mid);	//�����
	Elema_Init(Elema_Left); 
	Elema_Init(Elema_Right);
	gpio_init(Button_Up,GPI,0,PULLUP);
	gpio_init(Button_Down,GPI,0,PULLUP);
	gpio_init(Button_Left,GPI,0,PULLUP);
	gpio_init(Button_Right,GPI,0,PULLUP);
	gpio_init(Button_Mid,GPI,0,PULLUP);
	gpio_init(Switch_1,GPI,0,PULLUP);
	gpio_init(Switch_2,GPI,0,PULLUP);
	gpio_init(Switch_3,GPI,0,PULLUP);	
	gpio_init(Switch_4,GPI,0,PULLUP);	
	
	for(uint8 row=0;row<6;row++)//���ܳ�ʼ�����ܽŶ�����SelfBuild_control.h��
	{
		for(uint8 col=0;col<6;col++)
		{
			if((uint8)(Gray[row][col])!=0xff)
			{
				gpio_init(Gray[row][col],GPI,0,PULLUP);	
			}
		}
	}
	
	
	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)		//�����س�ʼ��
	{
		gpio_init(Encoder_Dir_Pin[(uint8)num],GPI,1,PULLUP);		//�����������
		ctimer_count_init(Encoder_Pulse_TIMER[(uint8)num]);		//������������
		sct_pwm_init(Motor_PWM[0][(uint8)num],5000,0);  //��תPWM
		sct_pwm_init(Motor_PWM[1][(uint8)num],5000,0);  //��תPWM
	}
	
	OLED_Init();
	while(MPU_Init_ForUser());

//  /*----------------------------�˵�����----------------------------------*/
	DisableInterrupts;                          //�ر������жϣ���ֹ�˵����ڹ����г����ж�
	Menu_Init();                                  //��ʼ���˵�
	while(!Menu_Work()) systick_delay_ms(200);    //�˵�ÿ200ms����һ�Σ��������Ƿ��¡��رղ˵���ѡ��󣨺�������0��������ѭ��
	EnableInterrupts;
	
	PID_Dir.Param_Kd*=10;
	
	Elema_Absorb(Elema_Left);
	Elema_Absorb(Elema_Right);
	MECANUM_Motor_Data.Speed_All=0;
	MECANUM_Motor_Data.Speed_X=0;
	MECANUM_Motor_Data.Speed_Y=0;
	MECANUM_Motor_Data.Speed_GyroZ_Set=0;

	View_MPUddata();
	
	pit_init_ms(10);
	enable_irq(RIT_IRQn);
	
	uart_rx_irq(USART_0,1); 
}

void Read_ButtSwitData(void)
{
		Base_Data.Switch_Data=((~gpio_get(Switch_1)<<3)&0x08)  |	//���뿪�����ݱ�����Base_Data.Switch_Data�ṹ����
													((~gpio_get(Switch_2)<<2)&0x04)  |
													((~gpio_get(Switch_3)<<1)&0x02)  |
													(~gpio_get(Switch_4)&0x01);
		Base_Data.Button_Data=((~gpio_get(Button_Up)<<4)&0x10)    |//�������ݱ�����Base_Data.Switch_Data�ṹ����
													((~gpio_get(Button_Down)<<3)&0x08)  |
													((~gpio_get(Button_Left)<<2)&0x04)  |
													((~gpio_get(Button_Right)<<1)&0x02) |
													(~gpio_get(Button_Mid)&0x01);
}

void Read_GrayData(uint8 show)
{
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			if((uint8)(Gray[row][col])!=0xff)
			{
				Base_Data.Gray_Data[row][col]=gpio_get(Gray[row][col]);
				if(show)OLED_P6x8Int(col*10 , row, Base_Data.Gray_Data[row][col], 1);
			}
		}
	}
}

void Save_GrayData(uint8 data_new[6][6],uint8 data_save[6][6])
{
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			data_save[row][col]=data_new[row][col];
		}
	}
}

void Judge_GrayData(void)
{
	for(uint8 row=0;row<6;row++)
	{
		for(uint8 col=0;col<6;col++)
		{
			if(Base_Data.Gray_Data[row][col]<Base_Data.Gray_Data_Last[row][col])//����ϴι����жϵ����߶����û�����½�����һ
				Base_Data.Gray_Data_Fall[row][col]=1;
			else if(Base_Data.Gray_Data[row][col]>Base_Data.Gray_Data_Last[row][col])//�����ι����жϵ����߶��ϴ�û������������һ
				Base_Data.Gray_Data_Rise[row][col]=1;
			else
			{
				Base_Data.Gray_Data_Fall[row][col]=0;
				Base_Data.Gray_Data_Rise[row][col]=0;
			}
		}
	}
}


uint8 View_MPUddata(void)
{
  LED_Fill(0x00);
  LED_P6x8Str(0, 0, "Pitch=");
  LED_P6x8Str(0, 1, "Roll=");
  LED_P6x8Str(0, 2, "Yaw=");
	LED_P6x8Str(0, 3, "Yaw_Real=");
  while(1)
  {
		  Read_ButtSwitData();
			if(Query_ButtSwitData(Button_Data,Button_Up_Data))break;
		  Refresh_MPUTeam(DMP_MPL);
			if(Query_ButtSwitData(Button_Data,Button_Right_Data))MPU_Data.Yaw_Save=MPU_Data.Yaw;
			MPU_Data.Yaw_Real=Mpu_Normalization(MPU_Data.Yaw,MPU_Data.Yaw_Save);		
		  OLED_P6x8Flo(60, 0, MPU_Data.Pitch, -3);
		  OLED_P6x8Flo(60, 1, MPU_Data.Roll, -3);
		  OLED_P6x8Flo(60, 2, MPU_Data.Yaw, -3);
		  OLED_P6x8Flo(60, 3, MPU_Data.Yaw_Real, -3); 
  }
	for(uint8 i=0;i<=20;i++)
	{
	Refresh_MPUTeam(DMP_MPL);
	systick_delay_ms(10);
	}
	LED_Fill(0x00);
  return 0;
}