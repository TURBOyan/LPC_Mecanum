#ifndef _SELFBUILD_GRAYCTRL_H_
#define _SELFBUILD_GRAYCTRL_H_

#include "headfile.h"

extern PIN_enum Gray[6][6];

void Read_GrayData(uint8 x,uint8 y,uint8 showflag);
void Save_GrayData(uint8 data_new[6][6],uint8 data_save[6][6]);
void Judge_GrayData(void);
uint8 Gray_Calibration(int16 X_Dir,int16 Y_Dir,int8 Return_Flag);

#endif
