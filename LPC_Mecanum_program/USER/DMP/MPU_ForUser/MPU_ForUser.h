#ifndef _MPU_FORUSER_H_
#define _MPU_FORUSER_H_

#include "mpu9250.h"
#include "MPU9250_Config.h"		//��ֲ����Ҫ���õ��ļ�

/*
 * ��������ṹ��
 */
typedef struct
{
      float x;
      float y;
      float z;
}FLOAT_XYZ;

typedef struct
{
      int32 x;
      int32 y;
      int32 z;
}INT32_XYZ;

typedef struct
{
      int16 x;
      int16 y;
      int16 z;
}INT16_XYZ;

extern struct MPU_Typedef
{
		int16 TEMP;
		
		INT16_XYZ
			GYRO,				// ������ԭʼ����
			GYRO_Last,	// �������ϴ�����
			ACC, 				// ���ٶȼ�ԭʼ����
			ACC_Last,		// ���ٶȼ��ϴ�����
			MAG,
			MAG_Last;
		FLOAT_XYZ 
			GYRO_Real,		// �������˲��������
			ACC_Real;		// ���ٶȼ��˲��������
		float
			Pitch,			//������
			Roll,			//������
			Yaw,			//ƫ����
			Yaw_Real,		
			Yaw_Aid,
			Yaw_Save,
			Pitch_Last,		//�ϴθ�����
			Roll_Last,		//�ϴη�����
			Yaw_Last;		//�ϴ�ƫ����
		
		uint8 
			Closeloop_flag;
}MPU_Data;

typedef enum
{
	ACC,
	GYRO,
	MAG,
	TEMP,
	DMP_MPL,
}MPU_DataTeam_TypeDef;

uint8 MPU_Init_ForUser(void);
uint8 Refresh_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam);	//����MPU����
uint8 Save_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam);		//����MPU�ϴ�����
float Mpu_Normalization(float Mpu6050_Value,float Mpu6050_Zero);  //Mpuֵ��Ϊ0-360
#endif
