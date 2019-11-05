#ifndef _SELFBUILD_PICTURE_DEAL_H_
#define _SELFBUILD_PICTURE_DEAL_H_

#include "headfile.h"

//------�궨��-------
#define Row      120   //��
#define Col      188   //��

#define RowCut   120

#define Black 0X00
#define White 0XFF

#define MissLine 0
#define MidLine_Expect 94
#define MidPwmOut 1800

//--ȫ�ֱ���-------------------------------------
extern uint8 imageUse[Row][Col];
extern uint8 CameraBinary[Row][Col];//����ͼ���ֵ�����ݵ�����
extern uint8 Three_Line[Row][Col];
extern uint8 Left_Line[Row];                  //����
extern uint8 Right_Line[Row];                 //����
extern int Mid_Line[Row];                   //����

extern uint8 Left_Jump_Value;                 //�������
extern uint8 Right_Jump_Value;                //�������
extern uint8 Mid_Jump_Value;                  //����ͬʱ����                                          
extern int diatance;                     //���߼��
extern uint8 ValueNum;                        //������Ч��
extern uint8 Road_Type;                     //��������
extern uint8 Mut_Jump_Value;                  //��һ�η�ȫ����
extern int  Mid_Value;                     //����ֵ

void Save_image(void);
void Road_Get(void);
void Three_Line_Hander(void);
void Mid_Line_Hander(void);
void Throw_The_Left_Line(uint8 y);
void Throw_The_Right_Line(uint8 y);
void Road_Juage(void);
void Mid_Value_Read(void);
void Mid_read(void);
float Sqrt2(int a);

#endif