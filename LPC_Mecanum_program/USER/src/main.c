#include "headfile.h"

int main(void)
{
	get_clk();
 	Init_ALL();	//����ȫ����ʼ��
	
/*****************************************************************/
	MECANUM_Motor_Data.Speed_All=2000;	//��ʼ��������
	MECANUM_Motor_Data.Speed_X=0;
	MECANUM_Motor_Data.Speed_Y=0;
	MECANUM_Motor_Data.Speed_GyroZ_Set=0;
	MECANUM_Motor_Data.PWM_Mid=3750;
	MPU_Data.Yaw_CloseLoop_Flag=1;
/*****************************************************************/
	
	EnableInterrupts;//ȫ����ʼ���У���10ms�������ڽ��п���
	
	while(1)
	{
	}
}