#ifndef _SELFBUILD_CAMERA_CALCU_H_
#define _SELFBUILD_CAMERA_CALCU_H_

#include "headfile.h"

typedef struct
{
	float Error_Out;		//�������ƫ��ֵ
	int32 Control_Out;		//ƫ��ֵ��PID������Ŀ�����
	float Error_Aid;
	
}Camera_Data_Typedef;
extern Camera_Data_Typedef Camera_Data;

void Camera_Calcu(void);

#endif