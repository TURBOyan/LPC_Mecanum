#ifndef __SELFBUILD_ELECTRON_CALCU_H_
#define __SELFBUILD_ELECTRON_CALCU_H_

#include "headfile.h"

#define Circle_LEFT  -1
#define Circle_RIGHT 1

#define ADC_Max 3088	//�������ֵ

#define ADC_R_R_Min ADC_R_R_Min_8bit*(ADC_Max/193)	//������Сֵ
#define ADC_R_Min   ADC_R_Min_8bit*(ADC_Max/193)
#define ADC_M_Min   ADC_M_Min_8bit*(ADC_Max/193)
#define ADC_L_Min   ADC_L_Min_8bit*(ADC_Max/193)
#define ADC_L_L_Min ADC_L_L_Min_8bit*(ADC_Max/193)

extern float g_fHuandaoTurn;
extern float g_fHuandaoOut;                            ;

typedef enum
{
  L__ = 0,
  L_1,
  M_1,
  R_1,
  R__,
  ADC_NUM,
}ADC_POSITION;

typedef struct 	//��Ŵ�����������ؽṹ��
{
	uint8 AD_L_L;	//�������ֵ
	uint8 AD_L_L_Last;	//�������ֵ��һ��
	
	uint8 AD_L;	//�����ֵ
	uint8 AD_L_Last;	//�����ֵ��һ��
	
	uint8 AD_M;	//�м���ֵ
	uint8 AD_M_Last;	//�м���ֵ��һ��
	
	uint8 AD_R;	//�Ҳ���ֵ
	uint8 AD_R_Last;	//�Ҳ���ֵ��һ��
	
	uint8 AD_R_R;	//���Ҳ���ֵ
	uint8 AD_R_R_Last;	//���Ҳ���ֵ��һ��
	
	float Error_Out;		//�������ƫ��ֵ
	int32  Control_Out;		//ƫ��ֵ��PID������Ŀ�����
}AD_Data_Typedef;

extern AD_Data_Typedef AD_Data; //�����ر���

void ADC_Calcu(void);


#endif
