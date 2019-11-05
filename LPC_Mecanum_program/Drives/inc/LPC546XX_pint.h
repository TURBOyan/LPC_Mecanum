/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		�����ж�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/



#ifndef _LPC546XX_pint_h
#define _LPC546XX_pint_h

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    PINT_CH0,
    PINT_CH1,
    PINT_CH2,
    PINT_CH3,
    PINT_CH4,
    PINT_CH5,
    PINT_CH6,
    PINT_CH7,
}PINTNUM_enum;

#define PINT_RISE_FLAG_CLEAR(pint)    (PINT->RISE = (uint32)(1)<<pint)
#define PINT_FALL_FLAG_CLEAR(pint)    (PINT->FALL = (uint32)(1)<<pint)
#define PINT_IST_FLAG_CLEAR(pint)     (PINT->IST  = (uint32)(1)<<pint)


void pint_init(PINTNUM_enum pint, PIN_enum pin, TRIGGER_enum trigger);
void pint_enable_irq(PINTNUM_enum pint);
void pint_disable_irq(PINTNUM_enum pint);
void pint_close(PINTNUM_enum pint);

#endif 
