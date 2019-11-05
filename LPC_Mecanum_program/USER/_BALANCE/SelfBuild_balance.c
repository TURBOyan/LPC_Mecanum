#include "SelfBuild_balance.h"

int32
		Speed_Min = 0,	// ������С�ٶ�
		Theory_Duty = 0,// ����ֱ��ռ�ձ�
		Vel_Set = 0,	// Ŀ��ת����ٶ�
		Direct_Parameter = 0,// ת��ϵ��
		Direct_Last = 0,
		Radius = 0;		// Ŀ��ת��뾶����
uint8 System_OK=0;

int Balance(void)
{
	if(FLAG_2MS)
	{
		FLAG_2MS=0;
		Refresh_MPUTeam(GYRO);
		Data_Filter();
		Theory_Duty -= PID_Calcu(  MPU6050.Tar_Ang_Vel.y  ,  (int32)(MPU6050.GYRO_Real.y*10)  ,  &PID_Ang_Velo  ,  Incre  );
		Theory_Duty  = RANGE(Theory_Duty,950,-950);
		
		if(System_OK)
		{
			Direct_Parameter  = PID_Calcu(  Radius*Speed_Min  ,  (int32)MPU6050.GYRO.z ,  &PID_Direct  ,  Local  );
			Direct_Parameter  = RANGE(Direct_Parameter,1200,-1200);	
//			OLED_P6x8Int(0, 5, Direct_Parameter, -5); 
		}
		Direct_Last  = (int32)(Direct_Last*0.3 + Direct_Parameter*0.7);
		
		Motor_Data.PWM_R_Set  = Theory_Duty - Direct_Last;
		Motor_Data.PWM_L_Set  = Theory_Duty + Direct_Last;
		
		if(Motor_Data.Crazy_StopFlag)			//��תɲ��
		{
		  if(Motor_Data.Speed_Ave_Now > 20)
		  {
			Motor_Data.PWM_L_Set = -300-Dead_Pwm;
			Motor_Data.PWM_R_Set = -300-Dead_Pwm;
		  }
		  else
		  {
			Motor_Data.PWM_L_Set = 0-Dead_Pwm;
			Motor_Data.PWM_R_Set = 0-Dead_Pwm;
			Error_Flag.Steer_crazy = 1;
		  }
		}

		Motor_PWM_Set();	//���ݼ����PWM�������ֵ
		
		Refresh_MPUTeam(DMP);
	}
	
	if(FLAG_10MS)
	{
		 FLAG_10MS=0;
		 Motor_Speed_Get();		//�õ������ٶ�
		 
		 MPU6050.Tar_Ang_Vel.y  = PID_Calcu(  MPU6050.Target_Angle.y  ,  MPU6050.Pitch*100  ,  &PID_Ang  ,  Local  );
		 
		 //MPU6050.Tar_Ang_Vel.y=0;	//���νǶȻ�ʹ����仰
		 
		 MPU6050.Tar_Ang_Vel.y=RANGE(MPU6050.Tar_Ang_Vel.y,1500,-1500);	 
	}
	if(FLAG_100MS)
	{
		FLAG_100MS=0;
		
		MPU6050.Target_Angle.y  = MPU6050.ZERO.y*100
		  			 +10*PID_Calcu(  Motor_Data.Speed_Ave_Set  ,  Motor_Data.Speed_Ave_Now  ,  &PID_Speed  ,  Local);
		
		//MPU6050.Target_Angle.y=MPU6050.ZERO.y*100;		//�����ٶȻ�ʹ����仰
		
		
		MPU6050.Target_Angle.y=RANGE((int32)MPU6050.Target_Angle.y,6000,-6000);
		
		Speed_Min=(int32)(Speed_Min*0.1+Motor_Data.Speed_Ave_Now*0.9);
		if(Speed_Min<40)
		{
			Speed_Min=40;
		}
	}
	return SUCCESS;
}
