#ifndef _SELFBUILD_UI_H_
#define _SELFBUILD_UI_H_

#include "headfile.h"

//�ӻ�Ҫ�����£�
//1��OLED�����ж��Ƿ���±�����
//2������ǣ����η��ͱ������������������������ͣ�������ͷ��ͱ������
//3���жϽ��գ��������+����+���ݣ�
//
//4��ʵʱ����---�����
//
//ң����Ҫ�����£�
//1���ȴ����ո�������
//2����Ϊ���£�����ʾ���ӳɹ��������ձ������������������������ͣ�������š�������ʷ���ݱ�����flash�ڡ�
//3���ı����ֵʱѡ���Ƿ��ͣ���Ϊ�����򱣴���EEPROM�ڲ��������ݣ��������+����+���ݣ�
//
//4��ʵʱ����---����� 

//typedef struct
//{
//	const char* Title;                                                      //�ò˵�������
//	void* Sub_Value[SUBMENU_MAX];                                           //�ò˵�����������ı�����ֵ
//	int8 Sub_Type[SUBMENU_MAX];                                             //�ò˵�����������ı���������
//	const char* Sub_Title[SUBMENU_MAX];					//�ò˵�����������ı���������
//}Debug_Typedef;

extern struct Wireless_Flag_Typedef Wireless_Flag;
extern int16 Speed_Set;

struct Wireless_Flag_Typedef
{
uint8 Wirelesson_Flag,
			FLAG_START,
			FLAG_GO,
			FLAG_BACK,
			FLAG_Speedzero,
			FLAG_ALLSTOP,
			FLAG_AdcCal;
};

struct Wireless_ReceiveData_Typedef
{
	uint8 Cmd;
};

typedef enum
{
	Start=0x4c,		//��ʼλ
	Stop=0x55,		//��ֹλ
	Data=0x49,		//����λ
	Cmd=0x48,		//����λ
}Wireless_Cmd_Typedef;

void wireless_CommunInit(void);
void wireless_init(void);
void wireless_CommunSend(uint8 *data);
void wireless_Control(void);
void UART4_IRQHandler(void);
void Computer_Send(void);

#endif
