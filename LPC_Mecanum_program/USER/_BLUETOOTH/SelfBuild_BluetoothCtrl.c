#include "SelfBuild_BluetoothCtrl.h"

struct Computer_communi_Typedef Computer_communi;

void Android_Ctrl(uint8 Data)
{
		static uint8 mode;
		if(mode)
		{
			switch(Data)
			{
				case 'A': MECANUM_Motor_Data.Speed_Real.x=0;		//ǰ��
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All; break;
				
				case 'H': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All/1.4;		//��ǰ
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All/1.4; break;
				
				case 'G': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All;	//��ת
									MECANUM_Motor_Data.Speed_Real.y=0;   break;
				
				case 'F': MECANUM_Motor_Data.Speed_Real.x=-MECANUM_Motor_Data.Speed_All/1.4;	//���
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
				
				case 'E': MECANUM_Motor_Data.Speed_Real.x=0;		//����
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All;break;

				case 'D': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All/1.4;	//�Һ�
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
				
				case 'C': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All;		//��ת
									MECANUM_Motor_Data.Speed_Real.y=0;   break;

				case 'B': MECANUM_Motor_Data.Speed_Real.x=MECANUM_Motor_Data.Speed_All/1.4;	//��ǰ
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All/1.4;   break;
									
				case 'Z': MECANUM_Motor_Data.Speed_Real.x=0;		//ֹͣ
									MECANUM_Motor_Data.Speed_Real.y=0;
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;					break;	
									
				case 'X': Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);break;		//��ת90��

				case 'Y': Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);break;//��ת90��
									
				case 'p':	Elema_Unabsorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;break;//������
				case 'o': Elema_Absorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;	break;//������
				case 'n': Elema_Unabsorb(Elema_Left);	break;//�����ϰ�
				case 'm': Elema_Unabsorb(Elema_Right);	break;//�����ϰ�
				
				case 'K': mode=0;
									MPU_Data.Yaw_CloseLoop_Flag=0; //�ر�ƫ���Ǳջ�
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;
									break;
				default : break;
			}
		}
		else
		{
			switch(Data)
			{
				case 'A': MECANUM_Motor_Data.Speed_Real.x=0;		//ǰ��
									MECANUM_Motor_Data.Speed_Real.y=MECANUM_Motor_Data.Speed_All; break;
				
				case 'E': MECANUM_Motor_Data.Speed_Real.x=0;		//����
									MECANUM_Motor_Data.Speed_Real.y=-MECANUM_Motor_Data.Speed_All;break;

				case 'G': MECANUM_Motor_Data.Speed_GyroZ_Set=-100;	//˳ʱ��
									break;
			
				case 'C': MECANUM_Motor_Data.Speed_GyroZ_Set=100;			//��ʱ��
									break;
				
				case 'Z': MECANUM_Motor_Data.Speed_Real.x=0;		//ֹͣ
									MECANUM_Motor_Data.Speed_Real.y=0;
									MECANUM_Motor_Data.Speed_GyroZ_Set=0;			
									break;
									
				case 'X': MECANUM_Motor_Data.Speed_All=2000; break;		//����

				case 'Y': MECANUM_Motor_Data.Speed_All=500;break;//����
				
				case 'J': mode=1;
									MPU_Data.Yaw_CloseLoop_Flag=1; //����ƫ���Ǳջ�
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
	if(Computer_communi.Data_FinishReceive_Flag == 0)		//�����δ������
	{
		if(Computer_communi.Data_Point == Data_Point_Save)	//���û���µ�һλ���ݽ��ս�������δ����ָ��ֵ���ϴε�һ��
		{
			Computer_communi.Timer++;	//��һ��10ms
			if(Computer_communi.Timer == Timeout_10ms_num)	//���δ�������ݳ����趨��ʱ�䣬���ж�Ϊ�������
			{
				Computer_communi.Timer=0;	//����10ms������
				Computer_communi.Data_Point=0;
				Data_Point_Save=0;
				Computer_communi.Data_FinishReceive_Flag=1;		//��λ������ɱ�־λ
			}
		}
		else
		{
			 Data_Point_Save=Computer_communi.Data_Point;		//������µ����ݽ������򱣴�����ָ��
		}
	}
}

extern uint8 First_axis[3];
void Chess_Before_Ctrl(void)
{
	static uint8 continue_flag=0,Point=0;
	
	if(continue_flag == 0
	&& MECANUM_Motor_Data.Car_RunPlayChess_Flag == 0)				//��һ�����ߵ���First_axis[Point]��-1��
	{
		continue_flag = 1;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[Point];		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}
	
	if(continue_flag == 1
	&&MECANUM_Motor_Data.Car_Arrive_Flag)		//�ڶ��������ߵ���First_axis[Point]��-1��ʱ������00+First_axis[Point]
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, First_axis[0]+48);		//����������ת��ΪASCALL���ٷ���
	}
	
	if(continue_flag == 2)
	{
		if(Point==2)
		{
			continue_flag = 3;
		}
		else
		{
			if(Computer_communi.Data_FinishReceive_Flag		//������յ�OK00?���ߵ���һ����
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
	
	if(continue_flag == 3			//������յ�����ָ���ʼ����
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] == 'M' || Computer_communi.Data[0] == 'H' || Computer_communi.Data[0] == 'N'))
	{
		Point =0;
		continue_flag = 0;
		MECANUM_Motor_Data.Car_RunPlayChess_Flag=1;
	}
}

void Chess_Move_M(void)			//�����ƶ�����
{
		static uint8 continue_flag=0;
		if(continue_flag == 0)		//�����ƶ����ϴ�����λ��
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Now.x;		//������������
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Now.y;
		}
		
		if(continue_flag == 1		//�ȵ��ƶ���ɣ���ʼ������
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Servo_Down;		//�����м�����
			Elema_Absorb(Elema_Mid);	//���������
		}
		
		if(continue_flag == 2
		&& Elema_Mid_Sensor_Read)			//��������Ѿ�������׼���ƶ�����һ��Ŀ��
		{
			continue_flag = 3;
			Servo_Up;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//������������
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 3					//����Ԥ���ص���󣬷������ӣ������浱ǰ����λ��
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			Elema_Unabsorb(Elema_Mid);		//�رյ�������ͷ�����
			MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;
			MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 4)
		{
			 continue_flag =0;
			 MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//���һ���ƶ����Ӳ�����׼�������´�ָ��
			 MECANUM_Motor_Data.Car_Coord_Set.x =-1;		//������������
			 MECANUM_Motor_Data.Car_Coord_Set.y =-1;
		}
}

void Chess_Ctrl(void)		//�����ܿ�
{
	if(Computer_communi.Data[0] == 'M')
	{
		MECANUM_Motor_Data.Chess_Coord_Set.x=Computer_communi.Data[1]-48;		//�������õ���������
		MECANUM_Motor_Data.Chess_Coord_Set.y=Computer_communi.Data[2]-48;
		Chess_Move_M();
	}
	
	if(Computer_communi.Data[0] == 'H')
	{
		
	}
}