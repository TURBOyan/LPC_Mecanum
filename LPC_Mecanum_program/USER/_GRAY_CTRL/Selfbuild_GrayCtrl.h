#ifndef _SELFBUILD_GRAYCTRL_H_
#define _SELFBUILD_GRAYCTRL_H_

#include "headfile.h"

extern PIN_enum Gray_Front[6][6];
extern PIN_enum Gray_Left[6][6];
extern PIN_enum Gray_Right[6][6];

void Read_GrayData(uint8 x,uint8 y,uint8 showflag);
uint8 Gray_Calibration_X(int16 X_Dir,int8 Return_Flag);
uint8 Gray_Calibration_Y(int16 Y_Dir,int8 Return_Flag);

#endif
