/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		���Ÿ�������
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
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ü����ù��ܳ�ʼ��
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      cfg         ���ò��� �鿴h�ļ�pin_configö������
//  @return     void
//  Sample usage:           iocon_init(A8,ALT0 | PULLUP | DIGITAL);       // A8 ��������Ϊͨ��IO������
//-------------------------------------------------------------------------------------------------------------------
void iocon_init(PIN_enum pin, uint32 cfg)
{
    uint16 port;
    uint16 pinn;
    
    if(A11 == pin || A12 == pin)    
    {
        ASSERT(0);
        while(1);   //������ʹ��A11 A12���ţ�����SWD�������⡣
    }
    port = pin>>5;
    pinn = pin&0x1f;
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK;        //��IOCONʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_IOCON_RST_MASK;    //�����λIOCONʱ��
        
        
    IOCON->PIO[port][pinn] = cfg;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ó�ʼ�������ı�ԭ�и��ù��ܣ��������þ��ᱻ��д��
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      cfg         ���ò��� �鿴h�ļ�pin_configö������
//  @return     void
//  Sample usage:           iocon_init(A8,PULLUP | DIGITAL);       // A8 ��������Ϊ����ģʽ������
//-------------------------------------------------------------------------------------------------------------------
void iocon_init_noalt(PIN_enum pin, uint32 cfg)
{
    uint16 port;
    uint16 pinn;
    
    if(A11 == pin || A12 == pin)    
    {
        ASSERT(0);
        while(1);   //������ʹ��A11 A12���ţ�����SWD�������⡣
    }
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK;        //��IOCONʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_IOCON_RST_MASK;    //�����λIOCONʱ��
    
    IOCON->PIO[port][pinn] = (IOCON->PIO[port][pinn] & IOCON_PIO_FUNC_MASK) | (~((uint32)IOCON_PIO_FUNC_MASK) & cfg);
}





