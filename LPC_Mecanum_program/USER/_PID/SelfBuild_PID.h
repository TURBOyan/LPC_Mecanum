#ifndef _SELFBUILD_PID_H
#define _SELFBUILD_PID_H

#include "common.h"

typedef struct 
{
  float Param_Kp;
	float Param_Ki;
	float Param_Kd;
	
  double AidData_Set;
	double Error_Sum;
	double Param_measure;
	
	double Error;
	double Error_last;
	double Error_last_last;
	
	double PID_Dynam_Out;
	double PID_Incre_Return;
	double PID_Local_Out;
}PID_Param_Set;

typedef enum
{
	Dynam,
	Incre,
	Local,
}PID_Mode_Typedef;

extern PID_Param_Set PID_Speed;	//�ٶȻ�PID����
extern PID_Param_Set PID_Dir;	  //����PID����
extern PID_Param_Set PID_Dis[2];	  //���뻷PID����
double PID_Calcu	(float Aid_Data,float Measure_Data,PID_Param_Set* P_I_D,PID_Mode_Typedef PID_Mode);

#endif
