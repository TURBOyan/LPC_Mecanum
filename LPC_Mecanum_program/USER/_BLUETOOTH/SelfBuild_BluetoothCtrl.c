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
				case 'n': Elema_Unabsorb(Elema_Front);	break;//�����ϰ�
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
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//�ڶ��������ߵ���First_axis[Point]��-1��ʱ������00+First_axis[Point]
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, '0');
		uart_putchar(USART_0, First_axis[Point]+48);		//����������ת��ΪASCALL���ٷ���
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
			&& Computer_communi.Data[4]==(First_axis[Point]+48)
			)
			{
				Computer_communi.Data_FinishReceive_Flag=0;
				continue_flag = 0;		
				Point++;
			}
			
			if(Computer_communi.Data_FinishReceive_Flag		//������յ��������ݣ������½���
			&& Computer_communi.Data[0]!='O')
			{
				Computer_communi.Data_FinishReceive_Flag=0;
				Computer_communi.Timer=0;	//����10ms������
				Computer_communi.Data_Point=0;		//����ָ��
			}
		}
	}
	
	if(continue_flag == 3			//������յ�����ָ���ʼ����
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] == 'M' || Computer_communi.Data[0] == 'H' || Computer_communi.Data[0] == 'V'))
	{
		Point =0;
		continue_flag = 0;
		Computer_communi.Data_FinishReceive_Flag =0;
		MECANUM_Motor_Data.Car_RunPlayChess_Flag=1;
	}
	
	if(continue_flag == 3			//������յ��������ݣ������½���
	&&Computer_communi.Data_FinishReceive_Flag
	&&(Computer_communi.Data[0] != 'M' && Computer_communi.Data[0] != 'H' && Computer_communi.Data[0] != 'V'))
	{
		Computer_communi.Data_FinishReceive_Flag=0;
		Computer_communi.Timer=0;	//����10ms������
		Computer_communi.Data_Point=0;		//����ָ��
	}
}

uint8 Chess_Move_M(void)			//�����ƶ�����
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
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
		
		if(continue_flag == 2)		//�����Ӽ�ʱ
		{
			Timer++;
		}
		
		if(continue_flag == 2
		&&Timer>=100
		)			//��������Ѿ�������׼���ƶ�����һ��Ŀ��
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
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//������������
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 3					//����Ԥ���ص���󣬷������ӣ������浱ǰ����λ��
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			Elema_Unabsorb(Elema_Mid);		//�رյ�������ͷ�����
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
			 MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;		//�����ʱ���ӵ�����
			 MECANUM_Motor_Data.Chess_Coord_Now.y=MECANUM_Motor_Data.Chess_Coord_Set.y;
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//ָ����ص�������
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 5
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//����ԭ���Ժ�
		{
			 continue_flag =6;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		}
		
		if(continue_flag == 6)	//����һ���
		{
			Timer++;
		}
		
		if(continue_flag == 6
		&&Timer>=50)
		{
			 continue_flag =7;
			 MPU_Data.Yaw_CloseLoop_Flag = 0;//�رսǶȱջ�
			 Elema_Absorb(Elema_Right);//���������ϰ�
			 Elema_Absorb(Elema_Front);//���������ϰ�
		}
		
		
		if(continue_flag == 7
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))//�ȴ��ϰ��������°�������ִ���¸��غ�
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//���һ���ƶ����Ӳ�����׼�������´�ָ��
			MPU_Data.Yaw_CloseLoop_Flag =1;//���¿����Ƕȱջ�
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;
		}
		return 0;
}

void WallCross_Push(void)	//�����ϰ�����
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
		if(continue_flag == 0)		//�����ƶ���Ŀ���ϰ�λ��
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//�����ϰ���������
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 1		//�ȵ��ƶ����
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Elema_Unabsorb(Elema_Front);	//���ȷ��³�ǰ���ϰ�
		}
		
		if(continue_flag == 2)
		{
			Timer++;
		}
		if(continue_flag == 2
		&&Timer>=100			//����ϰ��Ѿ����£���׼���ƶ�����һ��Ŀ��
		)			
		{
			continue_flag = 3;
			Timer =0;
			Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);			//��ת90��

		}
		
		if(continue_flag == 3
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//����ת���
		)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x+1;		//���ҷ��ƶ�һ��
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 4					//����Ԥ���ص���󣬷����ϰ��������浱ǰ�ϰ�λ��
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
			 Elema_Unabsorb(Elema_Right);		//�رյ�������ͷ�����
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
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//���ûص�������
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 7
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//����ԭ���Ժ󣬹رսǶȱջ����ȴ��ϰ�������ִ���¸��غ�
		{
			 continue_flag =8;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			 Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);		//��ת90��
		}
		
		if(continue_flag == 8
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//����ת���
		)
		{
			 continue_flag =9;
		}
		
		if(continue_flag == 9)
		{
			Timer++;
		}
		
		if(continue_flag == 9
		&& Timer >= 100)			//1���رսǶȱջ�
		{
			continue_flag =10;
			Timer=0;
			MPU_Data.Yaw_CloseLoop_Flag = 0;//�رսǶȱջ�
		  Elema_Absorb(Elema_Right);//���������ϰ�
		  Elema_Absorb(Elema_Front);//���������ϰ�
		}

		if(continue_flag == 10
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//���һ���ƶ����Ӳ�����׼�������´�ָ��
			
			MPU_Data.Yaw_CloseLoop_Flag =1;		//���¿����Ƕȱջ�
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;

		}
	
}

void WallVert_Push(void)		//�����ϰ�����
{
		static uint8 continue_flag=0;
		static uint16 Timer=0;
		if(continue_flag == 0)		//�����ƶ���Ŀ���ϰ�λ��
		{
			continue_flag = 1;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//�����ϰ���������
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
		}
		
		if(continue_flag == 1		//�ȵ��ƶ����
		&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
		{
			MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			continue_flag = 2;
			Elema_Unabsorb(Elema_Right);	//���ȷ��³��ҷ��ϰ�
		}
		
		if(continue_flag == 2)
		{
			Timer++;
		}
		if(continue_flag == 2
		&&Timer>=100			//����ϰ��Ѿ����£���׼���ƶ�����һ��Ŀ��
		)			
		{
			continue_flag = 3;
			Timer =0;
			Car_Turn_Right_90(&MECANUM_Motor_Data.Car_Dir_Mode);			//��ת90��

		}
		
		if(continue_flag == 3
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//����ת���
		)
		{
			continue_flag = 4;
			MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//���ҷ��ƶ�һ��
			MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y+1;
		}
		
		if(continue_flag == 4					//����Ԥ���ص���󣬷������ӣ������浱ǰ����λ��
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
			 Elema_Unabsorb(Elema_Front);		//�رյ�������ͷ�����
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
			 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//���ûص�������
			 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
		}
		
		if(continue_flag == 7
		&& MECANUM_Motor_Data.Car_Arrive_Flag)	//����ԭ���Ժ󣬹رսǶȱջ����ȴ��ϰ�������ִ���¸��غ�
		{
			 continue_flag =8;
			 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
			 Car_Turn_Left_90(&MECANUM_Motor_Data.Car_Dir_Mode);		//��ת90��
		}
		
		if(continue_flag == 8
		&& MPU_Data.Yaw_HeadZero >= (MPU_Data.Yaw_HeadZero_Aid-2) 
		&& MPU_Data.Yaw_HeadZero <= (MPU_Data.Yaw_HeadZero_Aid+2)//����ת���
		)
		{
			 continue_flag =9;
		}
		
		if(continue_flag == 9)
		{
			Timer++;
		}
		
		if(continue_flag == 9
		&& Timer >= 100)			//1���رսǶȱջ�
		{
			continue_flag =10;
			Timer=0;
			MPU_Data.Yaw_CloseLoop_Flag = 0;//�رսǶȱջ�
		  Elema_Absorb(Elema_Right);//���������ϰ�
		  Elema_Absorb(Elema_Front);//���������ϰ�
		}

		if(continue_flag == 10
		&&Query_ButtSwitData(Button_Data,Button_Up_Data))
		{
			continue_flag =0;
			MECANUM_Motor_Data.Car_RunPlayChess_Flag = 0;		//���һ���ƶ����Ӳ�����׼�������´�ָ��
			
			MPU_Data.Yaw_CloseLoop_Flag =1;		//���¿����Ƕȱջ�
			MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
			MPU_Data.Yaw_HeadZero_Aid=0;
			MPU_Data.Yaw_MapZero_Save=0;

		}
	
}

void Chess_Ctrl_Auto(void)		//�����ܿ�
{
	MECANUM_Motor_Data.Chess_Coord_Set.x=Computer_communi.Data[1]-48;		//�������õ���������
	MECANUM_Motor_Data.Chess_Coord_Set.y=Computer_communi.Data[2]-48;
	if(Computer_communi.Data[0] == 'M')
	{
		Chess_Move_M();			//�����ƶ�ִ��
	}
	
	if(Computer_communi.Data[0] == 'H')
	{
    WallCross_Push();		//�����ϰ�����
	}
	
	if(Computer_communi.Data[0] == 'V')
	{
    WallVert_Push();		//�����ϰ�����
	}
}

void Chess_Ctrl_Manual(void)
{
	static uint16 continue_flag=0,count=0,Timer=0;
	if(continue_flag == 0)	//����������������
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
	
	if(continue_flag == 1)				//��һ��
	{
		continue_flag = 2;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[0];		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
		
		MPU_Data.Yaw_CloseLoop_Flag =1;//���¿����Ƕȱջ�
		MPU_Data.Yaw_Save=MPU_Data.Yaw;	//����У׼��ͼ����
		MPU_Data.Yaw_HeadZero_Aid=0;
		MPU_Data.Yaw_MapZero_Save=0;
	}
	
	if(continue_flag == 2
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//�ڶ���
	{
		continue_flag = 31;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 31)			//��ʱ
	{
		Timer++;
	}
	
	if(continue_flag == 31
	&& Timer>=100)
	{
		continue_flag = 3;
		Timer=0;
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[1];		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}

	
	if(continue_flag == 3
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//������
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
		MECANUM_Motor_Data.Car_Coord_Set.x =First_axis[2];		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	}
	
	if(continue_flag == 4
	&&(MECANUM_Motor_Data.Car_Arrive_Flag))		//���Ĳ�
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
		MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Now.x;		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Now.y;
	}
	
	if(continue_flag == 6		//�ȵ��ƶ���ɣ���ʼ������
	&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
	{
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		continue_flag = 7;
		Servo_Down;		//�����м�����
		Elema_Absorb(Elema_Mid);	//���������
	}
	
	if(continue_flag == 7)		//�����Ӽ�ʱ
	{
		Timer++;
	}
	
	if(continue_flag == 7
	&&Timer>=100
	)			//��������Ѿ�������׼���ƶ�����һ��Ŀ��
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
		MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Chess_Coord_Set.x;		//������������
		MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Chess_Coord_Set.y;
	}
	
	if(continue_flag == 8					//����Ԥ���ص���󣬷������ӣ������浱ǰ����λ��
	&& MECANUM_Motor_Data.Car_Arrive_Flag == 1)
	{
		continue_flag = 9;
		MECANUM_Motor_Data.Car_Arrive_Flag = 0;
		Elema_Unabsorb(Elema_Mid);		//�رյ�������ͷ�����
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
		 MECANUM_Motor_Data.Chess_Coord_Now.x=MECANUM_Motor_Data.Chess_Coord_Set.x;		//�����ʱ���ӵ�����
		 MECANUM_Motor_Data.Chess_Coord_Now.y=MECANUM_Motor_Data.Chess_Coord_Set.y;
		 MECANUM_Motor_Data.Car_Coord_Set.x =MECANUM_Motor_Data.Car_Begin.x;		//ָ����ص�������
		 MECANUM_Motor_Data.Car_Coord_Set.y =MECANUM_Motor_Data.Car_Begin.y;
	}
	
	if(continue_flag == 10
	&& MECANUM_Motor_Data.Car_Arrive_Flag)	//����ԭ���Ժ�
	{
		 continue_flag =11;
		 MECANUM_Motor_Data.Car_Arrive_Flag = 0;
	}
	
	if(continue_flag == 11)	//����һ���
	{
		Timer++;
	}
	
	if(continue_flag == 11
	&&Timer>=50)
	{
		 continue_flag =0;
		 MPU_Data.Yaw_CloseLoop_Flag = 0;//�رսǶȱջ�
	}
}