/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		MRT(�����ʶ�ʱ��)
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-11-21
 ********************************************************************************************************************/


#include "common.h"
#include "LPC546XX_mrt.h"


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MRT�����ж�ģʽ��ʼ��
//  @param      mrtchx      MRTͨ����
//  @param      time        �����ж�ʱ��
//  @return     void
//  Sample usage:           mrt_pit_init(MRT_CH0,1000);         //�����û����� ��ʹ��H�ļ��ڵĺ궨��
//-------------------------------------------------------------------------------------------------------------------
void mrt_pit_init(MRTNUM_enum mrtchx, uint32 time)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_MRT_MASK;      //��MRTʱ��
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_MRT_RST_MASK;  //���MRT��λʱ��
    
    ASSERT(time <= MRT_CHANNEL_TIMER_VALUE_MASK);//����
    
    MRT0->CHANNEL[mrtchx].CTRL = MRT_CHANNEL_CTRL_INTEN_MASK | MRT_CHANNEL_CTRL_MODE(0);//PITģʽ ���ж�
    MRT0->CHANNEL[mrtchx].INTVAL = time;        //���ü��ʱ�� ��ʱ��ֹͣ������ʱ�� ��������ʱ��
    enable_irq(MRT0_IRQn);     				    //����RIT�ж�
    //set_irq_priority(MRT0_IRQn,0);//�������ȼ� Խ�����ȼ�Խ��
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      MRT��ʱ����
//  @param      mrtchx      MRTͨ����
//  @param      time        �����ж�ʱ��
//  @return     void
//  Sample usage:           mrt_delay(MRT_CH0,1000);            //�����û����� ��ʹ��H�ļ��ڵĺ궨��
//-------------------------------------------------------------------------------------------------------------------
void mrt_delay(MRTNUM_enum mrtchx, uint32 time)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_MRT_MASK;      //��MRTʱ��
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_MRT_RST_MASK;  //���MRT��λʱ��
    
    ASSERT(time <= MRT_CHANNEL_TIMER_VALUE_MASK);//����

    MRT0->CHANNEL[mrtchx].CTRL = MRT_CHANNEL_CTRL_MODE(1);      //һ���ж�ģʽģʽ

    MRT0->CHANNEL[mrtchx].INTVAL = time | MRT_CHANNEL_INTVAL_LOAD_MASK;        //���ü��ʱ�� ��������ʱ�� ��������ʱ��
    while(!MRT_FLAG_READ(mrtchx));              //�ȴ�ʱ�䵽
    MRT_FLAG_CLR(mrtchx);                       //�����־λ
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      MRT��ʼ��ʱ
//  @param      mrtchx      MRTͨ����
//  @param      time        �����ж�ʱ��
//  @return     void
//  Sample usage:           mrt_start(MRT_CH0);            
//-------------------------------------------------------------------------------------------------------------------
void mrt_start(MRTNUM_enum mrtchx)
{
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_MRT_MASK;      //��MRTʱ��
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_MRT_RST_MASK;  //���MRT��λʱ��
    
    MRT0->CHANNEL[mrtchx].CTRL = MRT_CHANNEL_CTRL_MODE(1);      //һ���ж�ģʽģʽ
    MRT0->CHANNEL[mrtchx].INTVAL = MRT_CHANNEL_INTVAL_IVALUE_MASK | MRT_CHANNEL_INTVAL_LOAD_MASK;//���ü��ʱ�� ��������ʱ�� ��������ʱ��
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MRT��ȡ��ʱʱ��
//  @param      mrtchx      MRTͨ����
//  @return     void
//  Sample usage:           uint32 time = mrt_get(MRT_CH0);     //�����û����� ��ʹ��H�ļ��ڵĺ궨��
//-------------------------------------------------------------------------------------------------------------------
uint32 mrt_get(MRTNUM_enum mrtchx)
{
    return (MRT_CHANNEL_INTVAL_IVALUE_MASK - MRT0->CHANNEL[mrtchx].TIMER); //������ص���0�����ʱ�Ѿ�����
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MRT��ʱ���
//  @param      mrtchx      MRTͨ����
//  @return     void
//  Sample usage:           �����ʱ����������
//-------------------------------------------------------------------------------------------------------------------
void mrt_clean(MRTNUM_enum mrtchx)
{
    MRT0->CHANNEL[mrtchx].INTVAL = MRT_CHANNEL_INTVAL_IVALUE_MASK | MRT_CHANNEL_INTVAL_LOAD_MASK;//���ü��ʱ�� ��������ʱ�� ��������ʱ��
}





