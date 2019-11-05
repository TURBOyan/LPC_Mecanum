/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		GPIO
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


//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��ʼ��
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      dir         ���ŷ��� GPI������   GPO�����
//  @param      dat         ���ŵ�ƽ 0���͵�ƽ   1���ߵ�ƽ  ����Ϊ�����Ǹò�����Ч
//  @param      pull        �������������� NOPULL:������    PULLDOWN:����    PULLUP:����  NOPULL:û������������
//  @return     void
//  Sample usage:           gpio_init(A8,GPO,0,NOPULL);     // ��ʼ��A8����Ϊ���ģʽ,����͵�ƽ,û������������
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(PIN_enum pin, GPIODIR_enum dir, uint8 dat, uint32 pull)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    if(P0 == port)      
    {
        SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_GPIO0_MASK;        //��GPIO0ʱ��
        SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_GPIO0_RST_MASK;    //�����λGPIO0ʱ��
    }
    else if(P1 == port) 
    {
        SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_GPIO1_MASK;        //��GPIO1ʱ��
        SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_GPIO1_RST_MASK;    //�����λGPIO1ʱ��
    }
    
    iocon_init(pin,ALT0 | DIGITAL | pull);//���ø��ù���Ϊͨ��IO
    
    if(GPI == dir)
    {
        if(pin<29)  GPIO->DIRCLR[port] = 1<<pinn;
        else
        {
            DisableInterrupts;
            GPIO->DIR[port] &= ~((uint32)1<<pinn);
            EnableInterrupts;
        }
        
    }
    else if(GPO == dir)
    {
        if(pinn<29)  GPIO->DIRSET[port] = 1<<pinn;
        else
        {
            DisableInterrupts;
            GPIO->DIR[port] |= 1<<pinn;
            EnableInterrupts;
        }
        if(!dat)        GPIO->CLR[port]  = 1<<pinn;
        else            GPIO->SET[port]  = 1<<pinn;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO����������������������
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      pull        �������������� NOPULL:������    PULLDOWN:����    PULLUP:����  NOPULL:û������������
//  @return     void
//  Sample usage:           gpio_pull_set(A8,NOPULL);       // ����A8����û������������
//-------------------------------------------------------------------------------------------------------------------
void gpio_pull_set(PIN_enum pin, uint8 pull)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK;        //��IOCONʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_IOCON_RST_MASK;    //�����λIOCONʱ��
    
    IOCON->PIO[port][pinn] = (IOCON->PIO[port][pinn] & ~((uint32)IOCON_PIO_MODE_MASK)) | pull;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO�������ŵ�ƽ
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      dat         ���ŵ�ƽ 0���͵�ƽ   1���ߵ�ƽ
//  @return     void
//  Sample usage:           gpio_set(A8,0);       // ����A8��������͵�ƽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_set(PIN_enum pin, uint8 dat)
{
    if(!dat)        GPIO->CLR[pin>>5]  = 1<<(pin&0x1f);
    else            GPIO->SET[pin>>5]  = 1<<(pin&0x1f);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO�������ŷ���
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @param      dir         ���ŷ��� GPI������   GPO�����
//  @return     void
//  Sample usage:           gpio_dir(A8,GPI);       // ����A8����Ϊ����ģʽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_dir(PIN_enum pin, GPIODIR_enum dir)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    if(GPI == dir)
    {
        if(pinn<29)  GPIO->DIRCLR[port] = 1<<pinn;
        else
        {
            DisableInterrupts;
            GPIO->DIR[port] &= ~((uint32)1<<pinn);
            EnableInterrupts;
        }
    }
    else if(GPO == dir)
    {
        if(pinn<29)  GPIO->DIRSET[port] = 1<<pinn;
        else
        {
            DisableInterrupts;
            GPIO->DIR[port] |= 1<<pinn;
            EnableInterrupts;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��ȡ���ŵ�ƽ
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @return     uint8       ���ŵ�ƽ 0���͵�ƽ   1���ߵ�ƽ
//  Sample usage:           uint8 dat gpio_get(A8);  // ��ȡA8���ŵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
uint8 gpio_get(PIN_enum pin)
{
    return (GPIO->B[pin>>5][pin&0x1f]);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO���������л�(��ת)
//  @param      pin         ѡ�����ţ�A0-A31  B0-B31��
//  @return     void
//  Sample usage:           gpio_toggle(A8);       // �л�A8���ŵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle(PIN_enum pin)
{
    GPIO->NOT[pin>>5]  = 1<<(pin&0x1f);
}
