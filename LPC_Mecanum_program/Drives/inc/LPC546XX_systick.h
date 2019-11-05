/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		�δ�ʱ��
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#ifndef _LPC546XX_systick_h
#define _LPC546XX_systick_h


void systick_delay(uint32 time);
uint32 systick_getval(void);


//------------------------------------���º궨������SYSTICK��ʱ------------------------------------
void systick_delay_ms(uint32 ms);                                           //���뼶systick��ʱ���� ���ڵδ�ʱ�����ֻ��24λ������ú���ʵ�֣�������ʱʱ�䷶Χ�Ͽ�
#define systick_delay_us(x)         systick_delay(x*main_clk_mhz)           //����SYSTICK��ʱʱ��  ��λus   ��Χ0  -  16777215(0xffffff)/(main_clk_mhz*1000)ms   ��Ƶ180M �����ʱʱ���ԼΪ93ms
#define systick_delay_ns(x)         systick_delay(x*main_clk_mhz/1000)      //����SYSTICK��ʱʱ��  ��λns   ��Χ0  -  16777215(0xffffff)*1000/(main_clk_mhz)ns   ��Ƶ180M �����ʱʱ���ԼΪ93ms


//------------------------------------���º궨������SYSTICK��ʱ------------------------------------
#define systick_timing_ms(x)        systick_timing(x*main_clk_mhz*1000)     //����SYSTICK��ʱʱ��  ��λms   ��Χ0  -  16777215(0xffffff)/(main_clk_mhz*1000)ms   ��Ƶ180M ���ʱʱ���ԼΪ93ms
#define systick_timing_us(x)        systick_timing(x*main_clk_mhz)          //����SYSTICK��ʱʱ��  ��λus   ��Χ0  -  16777215(0xffffff)/(main_clk_mhz)us        ��Ƶ180M ���ʱʱ���ԼΪ93ms
#define systick_timing_ns(x)        systick_timing(x*main_clk_mhz/1000)     //����SYSTICK��ʱʱ��  ��λns   ��Χ0  -  16777215(0xffffff)*1000/(main_clk_mhz)ns   ��Ƶ180M ���ʱʱ���ԼΪ93ms


//------------------------------------���º궨�����ڻ�ȡ��ǰSYSTICKʱ��------------------------------------
#define systick_getval_ms()         (systick_getval()/1000)/main_clk_mhz    //��ȡSYSTICK��ǰ��ʱʱ��  ��λms
#define systick_getval_us()         (systick_getval())/main_clk_mhz         //��ȡSYSTICK��ǰ��ʱʱ��  ��λus
#define systick_getval_ns()         (systick_getval()*1000)/main_clk_mhz    //��ȡSYSTICK��ǰ��ʱʱ��  ��λns
    

#endif
