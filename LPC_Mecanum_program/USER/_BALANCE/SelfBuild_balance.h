#ifndef _SELFBUILD_BALANCE_H_
#define _SELFBUILD_BALANCE_H_

#include "headfile.h"

extern int32
						 Speed_Min,	// ������С�ٶ�
						 Theory_Duty,// ����ֱ��ռ�ձ�
						 Vel_Set,	// Ŀ��ת����ٶ�
						 Direct_Parameter,// ת��ϵ��
						 Direct_Last,
						 Radius;		// Ŀ��ת��뾶����
extern uint8 System_OK;


int Balance(void);

#endif