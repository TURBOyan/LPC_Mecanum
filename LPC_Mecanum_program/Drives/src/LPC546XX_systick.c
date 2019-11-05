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

#include "common.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"


//-------------------------------------------------------------------------------------------------------------------
//  @brief      systick��ʱ����
//  @param      time            ��Ҫ��ʱ��ʱ��
//  @return     void
//  Sample usage:               �����û����ã��û���ʹ��h�ļ��еĺ궨��
//-------------------------------------------------------------------------------------------------------------------
void systick_delay(uint32 time)
{
    if(time == 0)   return;
    ASSERT(SysTick_LOAD_RELOAD_Msk >= time);//����   ��ʱʱ�����С�ڻ����SysTick_LOAD_RELOAD_Msk
    SysTick->CTRL = 0x00;                   //�ȹ��� systick ,���־λ
    SysTick->LOAD = time;                   //������ʱʱ��
    SysTick->VAL = 0x00;                    //��ռ�����
    SysTick->CTRL = ( 0 | SysTick_CTRL_ENABLE_Msk     //ʹ�� systick
                  //| SysTick_CTRL_TICKINT_Msk        //ʹ���ж� (ע���˱�ʾ�ر��ж�)
                    | SysTick_CTRL_CLKSOURCE_Msk      //ʱ��Դѡ�� (core clk)
                );
    while( !(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));//�ȴ�ʱ�䵽
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���뼶systick��ʱ����
//  @param      ms              ��ʱ���ٺ���
//  @return     void
//  Sample usage:               systick_delay_ms(1000);   //��ʱ1000����
//-------------------------------------------------------------------------------------------------------------------
void systick_delay_ms(uint32 ms)
{
    //get_clk();//��ȡ�ں�ʱ�ӱ��ں�������
	while(ms--) systick_delay(main_clk_mhz*1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      systick��ʱ��
//  @param      time            ��ʱʱ��(0-0x00ffffff)
//  @return     void
//  Sample usage:               �����û����ã��û���ʹ��h�ļ��еĺ궨��
//-------------------------------------------------------------------------------------------------------------------
void systick_timing(uint32 time)
{
    ASSERT(SysTick_LOAD_RELOAD_Msk >= time);//����   ��ʱʱ�����С�ڻ����SysTick_LOAD_RELOAD_Msk
    SysTick->LOAD = time;                   //������ʱʱ��
    SysTick->VAL = 0x00;       	            //��ռ�����
    SysTick->CTRL = ( 0 
					| SysTick_CTRL_ENABLE_Msk       //ʹ�� systick
                    | SysTick_CTRL_TICKINT_Msk      //ʹ���ж�
                    | SysTick_CTRL_CLKSOURCE_Msk    //ʱ��Դѡ�� (core clk) 
                );
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��õ�ǰSystem tick timer��ֵ
//  @return     ���ص�ǰSystem tick timer��ֵ
//  Sample usage:               uint32 tim = systick_getval();   
//-------------------------------------------------------------------------------------------------------------------
uint32 systick_getval(void)
{
    return SysTick->VAL;
}
