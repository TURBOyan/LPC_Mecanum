/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		FLEXCOMM���๦�ܴ���ͨ�Žӿڣ�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#include "common.h"
#include "LPC546XX_flexcomm.h" 

FLEXCOMM_Type * FLEXCOMMN[] = FLEXCOMM_BASE_PTRS;

void flexcomm_clk_enable(FLEXCOMMN_enum flexcommn, FLEXCOMM_TYPE_enum type)
{
    vuint32  now_type;
    //У�鵱ǰ��FLEXCOMM�Ƿ���������ռ��
    if(FLEXCOMM_PSELID_LOCK_MASK & FLEXCOMMN[flexcommn]->PSELID)    
    {
        now_type = FLEXCOMMN[flexcommn]->PSELID & FLEXCOMM_PSELID_PERSEL_MASK;//1:USART 2:SPI 3:IIC 4:IIS
        ASSERT(0);//�������ʧ��
    }

    if(1 != (SYSCON_FROHFCLKDIV_DIV_MASK & SYSCON->FROHFCLKDIV))
    {
        SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK_MASK; //��FRO_HF ����Ϊ96MHz
        SYSCON->FROHFCLKDIV = SYSCON_FROHFCLKDIV_HALT_MASK;
        SYSCON->FROHFCLKDIV = SYSCON_FROHFCLKDIV_DIV(0x01)| SYSCON_FROHFCLKDIV_HALT_MASK | SYSCON_FROHFCLKDIV_RESET_MASK;
        while(!(SYSCON->FROHFCLKDIV & SYSCON_FROHFCLKDIV_REQFLAG_MASK));
        SYSCON->FROHFCLKDIV = SYSCON_FROHFCLKDIV_DIV(0x01);
    }
    SYSCON->FROCTRL |= SYSCON_FROCTRL_SEL(0x01) | SYSCON_FROCTRL_HSPDCLK(0x01); //��FRO_HF ����Ϊ96MHz
    
    SYSCON->FCLKSEL[flexcommn] = SYSCON_FCLKSEL_SEL(0x01);                      //ѡ��FRO_HF_DIV��ΪFLEXCOMMʱ������Դ
    
    if(FLEXCOMM_8 == flexcommn)
    {
        SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_FLEXCOMM8_MASK;            //��FLEXCOMMʱ��
        SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_FC8_RST_MASK;              //�����λFLEXCOMMʱ��
    }
    else
    {
        SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK<<flexcommn; //��FLEXCOMMʱ��
        SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_FC0_RST_MASK<<flexcommn;   //�����λFLEXCOMMʱ��
    }
    
    FLEXCOMMN[flexcommn]->PSELID |= (FLEXCOMM_PSELID_PERSEL(type) | FLEXCOMM_PSELID_LOCK_MASK);     
}

uint32 flexcomm_get_clk(FLEXCOMMN_enum flexcommn)
{
    return 48000000U;
}

void flexcomm_irq(FLEXCOMMN_enum flexcommn, uint8 status)
{
    if(status)
    {
        if(FLEXCOMM_8 == flexcommn)     enable_irq(FLEXCOMM8_IRQn);
        else                            enable_irq((IRQn_Type)(FLEXCOMM0_IRQn+(IRQn_Type)flexcommn));
    }
    else
    {
        if(FLEXCOMM_8 == flexcommn)     disable_irq(FLEXCOMM8_IRQn);
        else                            disable_irq((IRQn_Type)(FLEXCOMM0_IRQn+(IRQn_Type)flexcommn));
    }
}
