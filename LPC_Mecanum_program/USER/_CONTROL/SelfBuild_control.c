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
	
	memset(&Base_Data,0,sizeof(Base_Data));		//�����л��õ��Ľṹ���ʼ��Ϊ0
	memset(&MECANUM_Motor_Data,0,sizeof(MECANUM_Motor_Data));
	memset(&MPU_Data,0,sizeof(MPU_Data));
	MECANUM_Motor_Data.PWM_Mid=3750;				//���PWM��ֵ
	
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
		sct_pwm_init(Motor_PWM[1][(uint8)num],250,MECANUM_Motor_Data.PWM_Mid);  //��תPWM
	}
	
	OLED_Init();
	while(MPU_Init_ForUser());	//��ʼ��MPU9250��ֱ����ʼ�����

//  /*----------------------------�˵�����----------------------------------*/
	DisableInterrupts;                          //�ر������жϣ���ֹ�˵����ڹ����г����ж�
	Menu_Init();                                  //��ʼ���˵�
	while(!Menu_Work()) systick_delay_ms(200);    //�˵�ÿ200ms����һ�Σ��������Ƿ��¡��رղ˵���ѡ��󣨺�������0��������ѭ��
	
//	Elema_Absorb(Elema_Left);
//	Elema_Absorb(Elema_Right);
	
	View_MPUddata();
	
	pint_init(PINT_CH1, B14, FALLING);		//MPU9250��INT����������A3�ϣ�����Ϊ�½��ش���
	set_irq_priority(PIN_INT1_IRQn,1);//�������ȼ� Խ�����ȼ�Խ��
	
	enable_irq(PIN_INT1_IRQn);		//���������ж�
	uart_rx_irq(USART_0,2); 	//����ң���жϿ���
	
	DisableInterrupts;  
}

void DataSend(uint8 AllowFlag)		//��1λ������
{
	if(AllowFlag)
	{
		int16 data[4];
		data[0]=(int16)MECANUM_Motor_Data.Speed.x/100;
		data[1]=(int16)MECANUM_Motor_Data.Speed.y/100;
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

uint8 Distance_Coarse(int8* X_Now,int8* Y_Now,int8 X_Set,int8 Y_Set)
{
	static double Distance_SetX,Distance_SetY;
	static int8 Continue_Flag_x=0,Continue_Flag_y=0,Return_Flag=0;
	
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
		Return_Flag=1;
		if(*X_Now >= 6)
		{
			Distance_SetX=-15;
		}
		else
		{
			Distance_SetX=15;
		}
	}
	if(Continue_Flag_x == 1)	//������ֵ�ʱ����ʼ�ֵ�-x
	{
	//�ֱ����Ե�ͼ��X�ٶȷ������û�������������ֹͣ��������
		MECANUM_Motor_Data.Speed_Real.x=10*PID_Calcu	(Distance_SetX,MECANUM_Motor_Data.Distance_Real.x,&PID_Dis[0],Local);
		if(fabs(MECANUM_Motor_Data.Distance_Real.x)<=5)		//��ͼX����//����ʱ
		{
			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,900,-900);
		}
		else if(fabs(Distance_SetX-MECANUM_Motor_Data.Distance_Real.x)>=50)
		{
			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
		}
		else//��Ҫ�ӽ�Ŀ������ʱ
		{
			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,500,-500);
		}
	}
	
	if(	 Continue_Flag_x == 1					//���ӽ�Ŀ������ʱ��ֹͣ�ֵ�����ʼϸ��
	&&(MECANUM_Motor_Data.Distance_Real.x >=Distance_SetX-30)
	&&(MECANUM_Motor_Data.Distance_Real.x <=Distance_SetX+30)
	)
	{
		Continue_Flag_x =2;
	}
	
		if(Continue_Flag_x == 2
	 &&Gray_Calibration_X((int16)Distance_SetX,Return_Flag)		//����ϸ��
		)
	{
		Continue_Flag_x=3;
		Return_Flag=0;
		MECANUM_Motor_Data.Distance_Real.x = 0;
		MECANUM_Motor_Data.Speed_Real.x=0;
		*X_Now=X_Set;		//����λ��
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
		Return_Flag=1;
		if(*Y_Now >= 6)
		{
			Distance_SetY=-15;
		}
		else
		{
			Distance_SetY=15;
		}
	}
	if(Continue_Flag_y == 1)	//������ֵ�ʱ����ʼ�ֵ�-x
	{
	//�ֱ����Ե�ͼ��X�ٶȷ������û�������������ֹͣ��������
		MECANUM_Motor_Data.Speed_Real.y=10*PID_Calcu	(Distance_SetY,MECANUM_Motor_Data.Distance_Real.y,&PID_Dis[1],Local);
		if(fabs(MECANUM_Motor_Data.Distance_Real.y)<=5)		//��ͼX����//����ʱ
		{
			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,900,-900);
		}
		else if(fabs(Distance_SetY-MECANUM_Motor_Data.Distance_Real.y)>=50)
		{
			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
		}
		else//��Ҫ�ӽ�Ŀ������ʱ
		{
			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,500,-500);
		}
	}
	
	if(	 Continue_Flag_y == 1					//���ӽ�Ŀ������ʱ��ֹͣ�ֵ�����ʼϸ��
	&&(MECANUM_Motor_Data.Distance_Real.y >=Distance_SetY-30)
	&&(MECANUM_Motor_Data.Distance_Real.y <=Distance_SetY+30)
	)
	{
		Continue_Flag_y =2;
	}
	
	if(Continue_Flag_y == 2
	 &&Gray_Calibration_Y((int16)Distance_SetY,Return_Flag)		//����ϸ��
		)
	{
		Continue_Flag_y=3;
		Return_Flag=0;
		MECANUM_Motor_Data.Distance_Real.y = 0;
		MECANUM_Motor_Data.Speed_Real.y=0;
		*Y_Now=Y_Set;		//����λ��
	}
	
	if(Continue_Flag_x==3
	 &&Continue_Flag_y==3)
	{
		Continue_Flag_x=0;
		Continue_Flag_y=0;
		MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
		MPU_Data.Yaw_HeadZero_Aid=0;
		MPU_Data.Yaw_MapZero_Save=0;
	}
	
//	
//	
//	
//	if(Continue_Flag==0										//���趨������ʹ�ʱ�����귢���仯ʱ,��ʼ�ֵ�
//		&&((*X_Now!=X_Set) || (*Y_Now!=Y_Set)))
//	{
//		Continue_Flag=1;
//		MECANUM_Motor_Data.Distance_Real.x = 0;
//		MECANUM_Motor_Data.Distance_Real.y = 0;
//		Distance_SetX = (X_Set-*X_Now)*50;
//		Distance_SetY = (Y_Set-*Y_Now)*50;
//		if(X_Set==*X_Now)
//		{
//			Return_Flag=1;
//			if(*X_Now == 6)
//			{
//				Distance_SetX=-15;
//			}
//			else if(*X_Now == 0)
//			{
//				Distance_SetX=15;
//			}
//			else 
//			{
//				Distance_SetX=15;
//			}
//		}
//		
//		if(Y_Set==*Y_Now)
//		{
//			Return_Flag=-1;
//			if(*Y_Now == 6)
//			{
//				Distance_SetY=-15;
//			}
//			else if(*Y_Now == 0)
//			{
//				Distance_SetY=15;
//			}
//			else 
//			{
//				Distance_SetY=15;
//			}
//		}
//	}
//	
//	if(Continue_Flag == 1)	//������ֵ�ʱ����ʼ�ֵ�
//	{
//	//�ֱ����Ե�ͼ��X��Y�ٶȷ������û�������������ֹͣ��������
//		if(fabs(MECANUM_Motor_Data.Distance_Real.x)<=5)		//��ͼX����//����ʱ
//		{
//			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,900,-900);
//		}
//		else if(fabs(Distance_SetX-MECANUM_Motor_Data.Distance_Real.x)>=50)
//		{
//			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
//		}
//		else//��Ҫ�ӽ�Ŀ������ʱ
//		{
//			MECANUM_Motor_Data.Speed_Real.x=RANGE(MECANUM_Motor_Data.Speed_Real.x,500,-500);
//		}
//		
//		if(fabs(MECANUM_Motor_Data.Distance_Real.y)<=5)	//��ͼY����//����ʱ
//		{
//			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,900,-900);
//		}
//		else if(fabs(Distance_SetY-MECANUM_Motor_Data.Distance_Real.y)>=50)
//		{
//			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,MECANUM_Motor_Data.Speed_All,-MECANUM_Motor_Data.Speed_All);
//		}
//		else//��Ҫ�ӽ�Ŀ������ʱ
//		{
//			MECANUM_Motor_Data.Speed_Real.y=RANGE(MECANUM_Motor_Data.Speed_Real.y,500,-500);
//		}
//	}
//	
//	if(	 Continue_Flag == 1					//���ӽ�Ŀ������ʱ��ֹͣ�ֵ�����ʼϸ��
//		&&(MECANUM_Motor_Data.Distance_Real.x >Distance_SetX-30)
//		&&(MECANUM_Motor_Data.Distance_Real.x <Distance_SetX+30)
//		&&(MECANUM_Motor_Data.Distance_Real.y >Distance_SetY-30)
//		&&(MECANUM_Motor_Data.Distance_Real.y <Distance_SetY+30)
//		)
//	{
//		Continue_Flag =2;
//	}
//	
//	if(Continue_Flag == 2
//	 &&Gray_Calibration((int16)Distance_SetX,(int16)Distance_SetY,Return_Flag)		//����ϸ��
//		)
//	{
//		Continue_Flag=0;
//		Return_Flag=0;
//		MECANUM_Motor_Data.Distance_Real.x = 0;
//		MECANUM_Motor_Data.Distance_Real.y = 0;
//		MECANUM_Motor_Data.Speed_Real.x=0;
//		MECANUM_Motor_Data.Speed_Real.y=0;
//		*X_Now=X_Set;		//����λ��
//		*Y_Now=Y_Set;
//		
//		MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
//		MPU_Data.Yaw_HeadZero_Aid=0;
//		MPU_Data.Yaw_MapZero_Save=0;
//	}

///**/		OLED_P6x8Int(70, 0, Continue_Flag, 2);
///**/		OLED_P6x8Int(70, 1, Gray_Flag_x, 2);
///**/		OLED_P6x8Int(70, 2, Gray_Flag_y, 2);
//	
//	if(Continue_Flag == 2
//	// &&Gray_Calibration()		//����ϸ��
//		)
//	{
//		Continue_Flag=0;
//		MECANUM_Motor_Data.Distance_Real.x = 0;
//		MECANUM_Motor_Data.Distance_Real.y = 0;
//		MECANUM_Motor_Data.Speed_Real.x=0;
//		MECANUM_Motor_Data.Speed_Real.y=0;
//		*X_Now=X_Set;		//����λ��
//		*Y_Now=Y_Set;

//		MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
//		MPU_Data.Yaw_HeadZero_Aid=0;
//		MPU_Data.Yaw_MapZero_Save=0;
//	}

}