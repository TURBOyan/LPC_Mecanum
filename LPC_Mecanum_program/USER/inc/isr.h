/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/
 
#ifndef _isr_h
#define _isr_h

#include "common.h"

#define SUCCESS 1
#define FAIL    0

extern uint8 FLAG_2MS,FLAG_5MS,FLAG_10MS,FLAG_50MS,FLAG_100MS,FLAG_200MS;

void RIT_DriverIRQHandler(void);																									

void PIN_INT7_DriverIRQHandler(void);
void DMA0_DriverIRQHandler(void);
void FLEXCOMM5_DriverIRQHandler(void);

#endif
