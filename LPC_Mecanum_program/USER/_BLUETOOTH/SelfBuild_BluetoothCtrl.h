#ifndef _SELFBUILD_BLUETOOTHCTRL_H_
#define _SELFBUILD_BLUETOOTHCTRL_H_

#include "headfile.h"

extern struct Computer_communi_Typedef
{
	uint8 Data_Point;
	uint8 Data_FinishReceive_Flag;
	uint8 Data[20];
	
	uint16 Timer;
}Computer_communi;

void Android_Ctrl(uint8 Data);
void Uart_Receive_Ctrl(uint16 Timeout_10ms_num);
void Chess_Before_Ctrl(void);
void Chess_Ctrl(void);

#endif
