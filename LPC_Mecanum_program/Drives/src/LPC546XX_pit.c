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


#include "common.h"
#include "LPC546XX_pit.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����жϳ�ʼ��
//  @param      time        �����ж�ʱ��
//  @return     void
//  Sample usage:           �����û����ã��û�ֻ��Ҫʹ��h�ļ��еĺ궨�弴��
//-------------------------------------------------------------------------------------------------------------------
void pit_init(long long time)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_RIT_MASK;     //��PITʱ��
    
    ASSERT(time <= RIT_COMPVAL_RICOMP_MASK);
    RIT->CTRL = 0;
    RIT->COMPVAL = time & 0xFFFFFFFF;
    RIT->COMPVAL_H = (time>>32) & 0xFFFF;
    RIT->CTRL = RIT_CTRL_RITENCLR_MASK | RIT_CTRL_RITEN_MASK;
    
    //enable_irq(RIT_IRQn);     								   //����RIT�ж�
    //set_irq_priority(RIT_IRQn,0);//�������ȼ� Խ�����ȼ�Խ��
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ����
//  @param      time        ��ʱʱ��
//  @return     void
//  Sample usage:           �����û����ã��û�ֻ��Ҫʹ��h�ļ��еĺ궨�弴��
//-------------------------------------------------------------------------------------------------------------------
void pit_delay(long long time)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_RIT_MASK;     //��PITʱ��
    
    ASSERT(time <= RIT_COMPVAL_RICOMP_MASK);
    RIT->CTRL = 0;
    RIT->COMPVAL = time & 0xFFFFFFFF;
    RIT->COMPVAL_H = (time>>32) & 0xFFFF;
    RIT->CTRL = RIT_CTRL_RITENCLR_MASK | RIT_CTRL_RITEN_MASK;
    disable_irq(RIT_IRQn);
    
    while(!(RIT->CTRL & RIT_CTRL_RITINT_MASK));
    RIT->CTRL = 0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ����
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void pit_start(void)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_RIT_MASK;     //��PITʱ��

    RIT->CTRL = 0;
    RIT->COMPVAL = 0xFFFFFFFF;
    RIT->COMPVAL_H = 0xFFFF;
    RIT->COUNTER_H = 0;
    RIT->COUNTER = 0;
    RIT->CTRL = RIT_CTRL_RITENCLR_MASK | RIT_CTRL_RITEN_MASK;
    disable_irq(RIT_IRQn);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ����
//  @return     long long   ��ǰ��ʱʱ��
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
long long pit_get(void)
{
    long long time;
    
    time = RIT->COUNTER_H;
    time = time<<32 | RIT->COUNTER;
    
    return (time);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ���
//  @return     void
//  Sample usage:           ����������رռ�ʱ��
//-------------------------------------------------------------------------------------------------------------------
void pit_clean(void)
{
    RIT->CTRL = 0;
    RIT->COUNTER_H = 0;
    RIT->COUNTER = 0;
   //RIT->CTRL = RIT_CTRL_RITENCLR_MASK | RIT_CTRL_RITEN_MASK;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����жϷ���ʼ��
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void pit_deinit(void)
{
    RIT->CTRL = 0;
    SYSCON->AHBCLKCTRLCLR[1] = SYSCON_AHBCLKCTRL_RIT_MASK;     //��PITʱ��
    disable_irq(RIT_IRQn);
}
