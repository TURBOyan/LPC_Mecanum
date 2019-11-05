/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		���ڶ�ʱ��(RIT)
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/



#ifndef _LPC546XX_pit_h
#define _LPC546XX_pit_h




#define PIT_FLAG_CLEAR  (RIT->CTRL |= RIT_CTRL_RITINT_MASK)



//------------------------------------���´�������PIT�ж�------------------------------------
#define pit_init_ms(x)  pit_init(x * main_clk_mhz * 1000)   //��ʼ��PIT�������ö�ʱʱ��(��λΪ ����)
#define pit_init_us(x)  pit_init(x * main_clk_mhz)          //��ʼ��PIT�������ö�ʱʱ��(��λΪ ΢��)
#define pit_init_ns(x)  pit_init(x * main_clk_mhz / 1000)   //��ʼ��PIT�������ö�ʱʱ��(��λΪ ����)(180M��ʱ��Ƶ��ʱ��С��λΪ5.55����)


//------------------------------------���´�������PIT��ʱ------------------------------------
#define pit_delay_ms(x) pit_delay(x * main_clk_mhz * 1000)  //PIT��ʱ ����
#define pit_delay_us(x) pit_delay(x * main_clk_mhz)         //PIT��ʱ ΢��
#define pit_delay_ns(x) pit_delay(x * main_clk_mhz / 1000)  //PIT��ʱ ����(180M��ʱ��Ƶ��ʱ��С��λΪ5.55����)


//------------------------------------���´������ڻ�ȡPIT��ʱʱ��------------------------------------
#define pit_get_ms()    (pit_get() / 1000 / main_clk_mhz)   //��ȡPIT��ʱʱ��  ��λ����
#define pit_get_us()    (pit_get() / main_clk_mhz)          //��ȡPIT��ʱʱ��  ��λ΢��
#define pit_get_ns()    (pit_get() * 1000 /main_clk_mhz)    //��ȡPIT��ʱʱ��  ��λ����(180M��ʱ��Ƶ��ʱ��С��λΪ5.55����)



void pit_init(long long time);
void pit_delay(long long time);
void pit_start(void);
long long pit_get(void);
void pit_clean(void);
void pit_deinit(void);




#endif
