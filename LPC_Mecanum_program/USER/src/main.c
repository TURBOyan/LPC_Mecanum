#include "headfile.h"


uint8 First_axis[3]={1,5,3};

int main(void)
{
	get_clk();
 	Init_ALL();	//����ȫ����ʼ��
	
/************************��ʼ��������*****************************************/
	MECANUM_Motor_Data.Speed_All=1500;			//ȫ������ٶ�
	MPU_Data.Yaw_CloseLoop_Flag=1;					//Ĭ�Ͽ���ƫ���Ǳջ�
	
	MECANUM_Motor_Data.Car_Coord_Now.x =-1;		//���ó�ʼ����
	MECANUM_Motor_Data.Car_Coord_Now.y =-1;
	MECANUM_Motor_Data.Car_Coord_Set.x =-1;		//������������
	MECANUM_Motor_Data.Car_Coord_Set.y =-1;
	MECANUM_Motor_Data.Car_Dir_Mode=Front;	//���ó�ʼ���峯��
	
	MECANUM_Motor_Data.Chess_Coord_Now.x=3;		//�������ӳ�ʼ����
	MECANUM_Motor_Data.Chess_Coord_Now.y=0;
	MECANUM_Motor_Data.Chess_Coord_Set.x=3;		//����������������
	MECANUM_Motor_Data.Chess_Coord_Set.y=0;
/*****************************************************************/
	
	EnableInterrupts;//ȫ����ʼ���У���10ms�������ڽ��п���
	
	while(1)
	{	
		//���еĿ��ƴ�����isr.c��PIN_INT1_DriverIRQHandler()�ⲿ�жϷ������ڣ��ж���MPU9250��INT���ſ��ƣ�Ĭ��10ms����
	}
}