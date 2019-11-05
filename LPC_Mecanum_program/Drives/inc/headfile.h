/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		headfile
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/
#ifndef _headfile_h
#define _headfile_h

#include "isr.h"
#include "common.h"
#include "math.h"
#include "stdint.h"
#include "SEEKFREE_FUN.h"
					
#include "LPC546XX_flexcomm.h"   //LPC�ײ�
#include "LPC546XX_pll.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_uart.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_gint.h"
#include "LPC546XX_adc.h"
#include "LPC546XX_pit.h"
#include "LPC546XX_ctimer.h"
#include "LPC546XX_pint.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_dma.h"
#include "LPC546XX_spi.h"
#include "LPC546XX_iic.h"
#include "LPC546XX_eeprom.h"
#include "LPC546XX_mrt.h"

//#include "SEEKFREE_18TFT.h"		//��ɿ�
#include "SEEKFREE_FONT.h"
//#include "SEEKFREE_IIC.h"		
#include "SEEKFREE_MPU6050.h"		
//#include "SEEKFREE_OLED.h"
#include "SEEKFREE_NRF24L01.h"
#include "SEEKFREE_VIRSCO.h"
#include "SEEKFREE_7725.h"
#include "SEEKFREE_MT9V032.h"
#include "SEEKFREE_MMA8451.h"
#include "SEEKFREE_L3G4200D.h"
#include "SEEKFREE_ICM20602.h"
#include "VCAN_computer.h"
//#include "SEEKFREE_PRINTF.h"

#include "PIN_CFG.h"
#include "SelfBuild_PID.h"	//�Խ���
#include "Selfbuild_oled.h"
#include "SelfBuild_OLEDMenu.h"
#include "SelfBuild_Mecanum_Ctrl.h"
#include "SelfBuild_control.h"

#include "MPU_ForUser.h"


#endif
