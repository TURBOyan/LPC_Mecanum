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


#include "common.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_gint.h"


GINT_Type * GINTN[] = GINT_BASE_PTRS;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO0��GPIO1 ���жϳ�ʼ��
//  @param      gint        ���ж�ģ��ţ�GROUP0,GROUP1��,һ��ģ���Ӧһ���жϺ���
//  @param      pin         ���� A0 - A31  B0 - B31
//  @param      trigger     ������ʽ �鿴common.h�ļ�TRIGGER_enumö������
//  @return     void
//  Sample usage:           gint_init(GROUP0,A0,RISING,PULLUP);       //ʹ�����ж�0    A0  �����ش����ж�
//  @note                   ����ʹ�÷�ʽһ�����жϽ�ʹ��һ�������жϣ������Ҫʹ�ö����ÿ���ź���ҪΪһ���Ĵ�����ʽ
//-------------------------------------------------------------------------------------------------------------------
void gint_init(GINTN_enum gint, PIN_enum pin, TRIGGER_enum trigger)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    if((RISING == trigger) || (HIGH == trigger))    gpio_init(pin,GPI,0,PULLDOWN);
    else                                            gpio_init(pin,GPI,0,PULLUP);

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_GINT_MASK;     //����GINTʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_GINT_RST_MASK; //�����λGINTʱ��

    GINTN[gint]->PORT_POL[port] &= ~(uint32)(1<<pinn);
    GINTN[gint]->PORT_POL[port] |= ((RISING == trigger) || (HIGH == trigger))<<pinn;

    GINTN[gint]->PORT_ENA[port] |= 1<<pinn;
    
    GINTN[gint]->CTRL = ( 0
                        //| GINT_CTRL_COMB_MASK                                       //��������ʹ�û����� ����Ϊ��  ����Ϊ��
                        | GINT_CTRL_TRIG((LOW == trigger) || (HIGH == trigger))       //�������� 0�����ش���      1����ƽ����
                        );
    
    //enable_irq(GINT0_IRQn);//���ж� GINT0_IRQn GINT1_IRQn
    //set_irq_priority(GINT0_IRQn,0);//�������ȼ� Խ�����ȼ�Խ��
    
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������Ŵ�������ж�
//  @param      gint        ����ж�ģ��ţ�GROUP0,GROUP1��
//  @param      pin         ���� A0 - A31  B0 - B31
//  @return     void
//  Sample usage:           gint_disable(GROUP0,A0);       //����A0��������ж�0
//-------------------------------------------------------------------------------------------------------------------
void gint_enable(GINTN_enum gint, PIN_enum pin)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    GINTN[gint]->PORT_ENA[port] |= 1<<pinn;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������Ŵ�������ж�
//  @param      gint        ����ж�ģ��ţ�GROUP0,GROUP1��
//  @param      pin         ���� A0 - A31  B0 - B31
//  @return     void
//  Sample usage:           gint_disable(GROUP0,A0);       //����A0��������ж�0
//-------------------------------------------------------------------------------------------------------------------
void gint_disable(GINTN_enum gint, PIN_enum pin)
{
    uint16 port;
    uint16 pinn;
    
    port = pin>>5;
    pinn = pin&0x1f;
    
    GINTN[gint]->PORT_ENA[port] &= ~(((uint32)1)<<pinn);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO0��GPIO1 ���жϷ���ʼ��
//  @param      gint        ����ж�ģ��ţ�GROUP0,GROUP1��
//  @return     void
//  Sample usage:           gint_deinit(GROUP0);       //����ʼ�����ж�0
//-------------------------------------------------------------------------------------------------------------------
void gint_deinit(GINTN_enum gint)
{
    SYSCON->PRESETCTRLSET[0] = SYSCON_PRESETCTRL_GINT_RST_MASK; //��λGINTʱ��
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_GINT_MASK;     //�ر�GINTʱ��
    disable_irq((IRQn_Type)(GINT0_IRQn + (IRQn_Type)gint));
}


