#ifndef _MPU_FORUSER_H_
#define _MPU_FORUSER_H_

#include "mpu9250.h"
#include "MPU9250_Config.h"		//移植所需要配置的文件

/*
 * 定义坐标结构体
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
			GYRO,				// 陀螺仪原始数据
			GYRO_Last,	// 陀螺仪上次数据
			ACC, 				// 加速度计原始数据
			ACC_Last,		// 加速度计上次数据
			MAG,
			MAG_Last;
		FLOAT_XYZ 
			GYRO_Real,		// 陀螺仪滤波后的数据
			ACC_Real;		// 加速度计滤波后的数据
		float
			Pitch,			//俯仰角
			Roll,			//翻滚角
			Yaw,			//偏航角
			Yaw_Real,		
			Yaw_Aid,
			Yaw_Save,
			Pitch_Last,		//上次俯仰角
			Roll_Last,		//上次翻滚角
			Yaw_Last;		//上次偏航角
		
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
uint8 Refresh_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam);	//更新MPU数据
uint8 Save_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam);		//保存MPU上次数据
float Mpu_Normalization(float Mpu6050_Value,float Mpu6050_Zero);  //Mpu值归为0-360
#endif
