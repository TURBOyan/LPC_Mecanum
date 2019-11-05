/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		FLEXCOMM���๦�ܴ���ͨ�Žӿڣ�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#ifndef _LPC546XX_flexcomm_h
#define _LPC546XX_flexcomm_h

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    FLEXCOMM_0,
    FLEXCOMM_1,
    FLEXCOMM_2,
    FLEXCOMM_3,
    FLEXCOMM_4,
    FLEXCOMM_5,
    FLEXCOMM_6,
    FLEXCOMM_7,
    FLEXCOMM_8,
    FLEXCOMM_9, 
}FLEXCOMMN_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    USART = 1,
    SPI,
    IIC,
    IIS,
}FLEXCOMM_TYPE_enum;

void flexcomm_clk_enable(FLEXCOMMN_enum flexcommn, FLEXCOMM_TYPE_enum type);
uint32 flexcomm_get_clk(FLEXCOMMN_enum flexcommn);
void flexcomm_irq(FLEXCOMMN_enum flexcommn, uint8 status);

#endif
