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


#include "common.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_pint.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PINT�жϳ�ʼ��
//  @param      pint        PINTͨ��(PINT_CH0-PINT_CH7) ע��Ĭ�ϵ����������ͷ��ʹ��PINT_CH7�ж�
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      trigger     ������ʽ �鿴common.h�ļ�TRIGGER_enumö������
//  @return     void
//  Sample usage:           pint_init(PINT_CH0,A5,RISING);     //��ʼ��PINTͨ��0Ϊ�����ش���    PINTͨ��0ʹ��A5����
//-------------------------------------------------------------------------------------------------------------------
void pint_init(PINTNUM_enum pint, PIN_enum pin, TRIGGER_enum trigger)
{
    uint16 port;
    uint16 pinn;
    port = pin>>5;
    pinn = pin&0x1f;
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK; //�򿪶�·����ʱ��
    INPUTMUX->PINTSEL[pint] = INPUTMUX_PINTSEL_INTPIN((uint8)port*32 + pinn);
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK; //�رն�·����ʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_PINT_MASK;     //��PINTʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_PINT_RST_MASK; //���PINT��λʱ��
    
    gpio_init(pin,GPI,0,PULLUP);
    
    PINT_RISE_FLAG_CLEAR(pint);                                 //�����־λ
    PINT_FALL_FLAG_CLEAR(pint);                                 //�����־λ
    
    if(LOW == trigger || HIGH == trigger)
    {
        PINT->ISEL |= (uint32)(1)<<pint;                        //��ƽ����ģʽ
    }
    else
    {
        PINT->ISEL &= (~((1UL)<<pint) & PINT_ISEL_PMODE_MASK);  //���ش���ģʽ
    }
    
    if(RISING == trigger || BOTH == trigger)
    {
        PINT->SIENR |= (uint32)(1)<<pint;                       //�����ػ��ߵ�ƽ�����ж�
    }
    else
    {
        PINT->CIENR |= (uint32)(1)<<pint;                       //�ر������ػ��ߵ�ƽ�����ж�
    }
    
    if(FALLING == trigger || HIGH == trigger || BOTH == trigger)
    {
        PINT->SIENF |= (uint32)(1)<<pint;                       //�½��ػ��߸ߵ�ƽ�����ж�  
    }
    else
    {
        PINT->CIENF |= (uint32)(1)<<pint;                       //�͵�ƽ�����ж�
    }
    
    PINT_IST_FLAG_CLEAR (pint);                                 //�����־λ
    
    //enable_irq(PIN_INT0_IRQn);//���ж� PIN_INT0_IRQn - PIN_INT7_IRQn
    //set_irq_priority(PIN_INT0_IRQn,0);//�������ȼ� Խ�����ȼ�Խ��
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ʹ��PINT�жϿ���
//  @param      pint        PINTͨ��(PINT_CH0-PINT_CH7)
//  @return     void
//  Sample usage:           pint_enable_irq(PINT_CH0);      //����ͨ��0�жϿ���
//-------------------------------------------------------------------------------------------------------------------
void pint_enable_irq(PINTNUM_enum pint)
{
    if(PINT_CH4 > pint) enable_irq((IRQn_Type)(PIN_INT0_IRQn+(IRQn_Type)pint));
    else                enable_irq((IRQn_Type)(PIN_INT4_IRQn+(IRQn_Type)(pint-4)));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����PINT�жϿ���
//  @param      pint        PINTͨ��(PINT_CH0-PINT_CH7)
//  @return     void
//  Sample usage:           pint_enable_irq(PINT_CH0);      //����ͨ��0�жϿ���
//-------------------------------------------------------------------------------------------------------------------
void pint_disable_irq(PINTNUM_enum pint)
{
    if(PINT_CH4 > pint) disable_irq((IRQn_Type)(PIN_INT0_IRQn+(IRQn_Type)pint));
    else                disable_irq((IRQn_Type)(PIN_INT4_IRQn+(IRQn_Type)(pint-4)));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PINT�жϹر�
//  @param      pint        PINTͨ��(PINT_CH0-PINT_CH7)
//  @return     void
//  Sample usage:           pint_close(PINT_CH0);           //�ر�PINTͨ��0�жϹ���
//-------------------------------------------------------------------------------------------------------------------
void pint_close(PINTNUM_enum pint)
{
    PINT->ISEL &= (~((1UL)<<pint) & PINT_ISEL_PMODE_MASK);  //���ش���ģʽ
    PINT->CIENR |= (uint32)(1)<<pint;                       //�ر������ػ��ߵ�ƽ�����ж�
    PINT->CIENF |= (uint32)(1)<<pint;                       //�͵�ƽ�����ж�
    pint_disable_irq(pint);
}



