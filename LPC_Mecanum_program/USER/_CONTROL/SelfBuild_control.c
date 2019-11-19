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
//	Beep_Init;	//������
//	Servo_Init;	//�������
//	Elema_Init(Elema_Mid);	//�����
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
	
	MECANUM_Motor_Data.PWM_Mid=3750;
	for(WheelNum_Typedef num=0;num<Wheel_Sum;num++)		//�����س�ʼ��
	{
		ctimer_pwm_init(Motor_PWM[0][(uint8)num],250,MECANUM_Motor_Data.PWM_Mid);  //��תPWM
	}
	
	OLED_Init();
	while(MPU_Init_ForUser());	//��ʼ��MPU9250��ֱ����ʼ�����

//  /*----------------------------�˵�����----------------------------------*/
	DisableInterrupts;                          //�ر������жϣ���ֹ�˵����ڹ����г����ж�
	Menu_Init();                                  //��ʼ���˵�
	while(!Menu_Work()) systick_delay_ms(200);    //�˵�ÿ200ms����һ�Σ��������Ƿ��¡��رղ˵���ѡ��󣨺�������0��������ѭ��
	EnableInterrupts;
	
//	Elema_Absorb(Elema_Left);
//	Elema_Absorb(Elema_Right);
	MECANUM_Motor_Data.Speed_All=2000;
	MECANUM_Motor_Data.Speed_X=0;
	MECANUM_Motor_Data.Speed_Y=0;
	MECANUM_Motor_Data.Speed_GyroZ_Set=0;
	MPU_Data.Yaw_CloseLoop_Flag=1;

	View_MPUddata();
	
	pint_init(PINT_CH1, B0, FALLING);		//MPU9250��INT����������A3�ϣ�����Ϊ�½��ش���
	set_irq_priority(PIN_INT1_IRQn,1);//�������ȼ� Խ�����ȼ�Խ��
	
	enable_irq(PIN_INT1_IRQn);		//���������ж�
	uart_rx_irq(USART_0,2); 	//����ң���жϿ���
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

void Read_GrayData(uint8 x,uint8 y,uint8 showflag)
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

void calibration(void)
{
	static uint16 flag;
	uint8 Gray_upleft;
	uint8 Gray_dowmright;
	if(		Base_Data.Gray_Data[2][0]==1
			&&Base_Data.Gray_Data[1][0]==1
			&&Base_Data.Gray_Data[0][0]==1
			&&Base_Data.Gray_Data[0][1]==1
			&&Base_Data.Gray_Data[0][2]==1
	
			&&Base_Data.Gray_Data[3][5]==1
			&&Base_Data.Gray_Data[4][5]==1
			&&Base_Data.Gray_Data[5][5]==1
			&&Base_Data.Gray_Data[5][4]==1
			&&Base_Data.Gray_Data[5][3]==1
		)
	{
				return;
	}
	Gray_upleft  =  Base_Data.Gray_Data[2][0]
								 +Base_Data.Gray_Data[1][0]
								 +Base_Data.Gray_Data[0][0]
								 +Base_Data.Gray_Data[0][1]
								 +Base_Data.Gray_Data[0][2];
	Gray_dowmright=	Base_Data.Gray_Data[3][5]
								 +Base_Data.Gray_Data[4][5]
								 +Base_Data.Gray_Data[5][5]
								 +Base_Data.Gray_Data[5][4]
								 +Base_Data.Gray_Data[5][3];
	if(flag==0
		&&Gray_upleft>Gray_dowmright)
	{
		
	}
	
		
}