#include "headfile.h"
int main(void)
{
	get_clk();
 	Init_ALL();	//����ȫ����ʼ��
/************************��ʼ��������*****************************************/
	PID_Dis[1].Param_Kp=PID_Dis[0].Param_Kp;
	PID_Dis[1].Param_Kd=PID_Dis[0].Param_Kd;
	MECANUM_Motor_Data.Speed_All=1500;			//ȫ������ٶ�
	MPU_Data.Yaw_CloseLoop_Flag=1;					//Ĭ�Ͽ���ƫ���Ǳջ�
	
	MECANUM_Motor_Data.Car_Coord_Now.x=0;		//���ó�ʼ����
	MECANUM_Motor_Data.Car_Coord_Now.y=0;
	MECANUM_Motor_Data.Car_Coord_Set.x=0;		//������������
	MECANUM_Motor_Data.Car_Coord_Set.y=0;
/*****************************************************************/
	
	EnableInterrupts;//ȫ����ʼ���У���10ms�������ڽ��п���
	
	while(1)
	{	
		//���еĿ��ƴ�����isr.c��PIN_INT1_DriverIRQHandler()�ⲿ�жϷ������ڣ��ж���MPU9250��INT���ſ��ƣ�Ĭ��10ms����
	}
}