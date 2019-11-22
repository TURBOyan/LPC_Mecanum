#include "SelfBuild_control.h"
#include <string.h>

struct Base_Data_Typedef Base_Data;	//��������
	



void Init_ALL(void)		//ȫ����ʼ��
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
	
	View_MPUddata();
	
	pint_init(PINT_CH1, B0, FALLING);		//MPU9250��INT����������A3�ϣ�����Ϊ�½��ش���
	set_irq_priority(PIN_INT1_IRQn,1);//�������ȼ� Խ�����ȼ�Խ��
	
	enable_irq(PIN_INT1_IRQn);		//���������ж�
	uart_rx_irq(USART_0,2); 	//����ң���жϿ���
	
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
		printf("{A%d:%d:%d:%d}$",data[0],data[1],data[2],data[3]);			//�����ϴ�
	}
}


void Read_ButtSwitData(void)  //��ȡ�����Ͳ��뿪��ֵ
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
		if(Query_ButtSwitData(Button_Data,Button_Up_Data))		//��������ϼ���������У׼��ͼ����
		{
			MPU_Data.Yaw_Save=MPU_Data.Yaw;
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;
		}
		MPU_Data.Yaw_MapZero=Mpu_Normalization(MPU_Data.Yaw,MPU_Data.Yaw_Save);
		
		if(MPU_Data.Yaw_CloseLoop_Flag)		//����Ƕȱջ���־λ����λ����ִ�нǶȱջ�
		{
			MPU_Data.Yaw_HeadZero=Mpu_Normalization(MPU_Data.Yaw_MapZero,MPU_Data.Yaw_MapZero_Save);	//ƫ�������½���ͷ��Ϊ���
			MECANUM_Motor_Data.Speed_GyroZ_Out=PID_Calcu(MPU_Data.Yaw_HeadZero_Aid,MPU_Data.Yaw_HeadZero,&PID_Dir,Local);	//�Ƕȱջ�
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
