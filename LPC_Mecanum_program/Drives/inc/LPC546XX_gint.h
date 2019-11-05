/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		GPIO0��GPIO1 ���ж�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#ifndef _LPC546XX_gint_h
#define _LPC546XX_gint_h


//��ö�ٶ��岻�����û��޸�
typedef enum    // ö�����ж�ģ����
{
    GROUP0,
    GROUP1,
}GINTN_enum;



#define GINT0_FLAG   (GINT0->CTRL & GINT_CTRL_INT_MASK)         //���ж�0��־λ
#define GINT1_FLAG   (GINT1->CTRL & GINT_CTRL_INT_MASK)         //���ж�1��־λ

#define CLEAR_GINT0_FLAG   (GINT0->CTRL |= GINT_CTRL_INT_MASK)  //������ж�0��־λ
#define CLEAR_GINT1_FLAG   (GINT1->CTRL |= GINT_CTRL_INT_MASK)  //������ж�1��־λ



void gint_init(GINTN_enum gint, PIN_enum pin, TRIGGER_enum trigger);
void gint_enable(GINTN_enum gint, PIN_enum pin);
void gint_disable(GINTN_enum gint, PIN_enum pin);


#endif
