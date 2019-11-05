/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		SYSTEM_LPC54606
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
#include "LPC546XX_config.h"
#include "LPC546XX_uart.h"
#include "SYSTEM_LPC54606.h"


void clk_out(void)
{
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK;        //��IOCONʱ��
    IOCON->PIO[0][26] = (0
                       | IOCON_PIO_FUNC(2)          //�������Ÿ���Ϊ�ڶ����� CLK_OUT
                       | IOCON_PIO_MODE(2)          //����
                       | IOCON_PIO_DIGIMODE(1)      //����ģʽ
                       | IOCON_PIO_SLEW(1));        //������
    
    SYSCON->CLKOUTSELA = SYSCON_CLKOUTSELA_SEL(0);  //ѡ��CLK_OUTʱ��Դ 0:main_clk  1:clk_in 3:fro_hf 4:pll_clk
    
    SYSCON->CLKOUTDIV = SYSCON_CLKOUTDIV_HALT(1);   //��ͣCLK_OUT���
    SYSCON->CLKOUTDIV = (0
                       | SYSCON_CLKOUTDIV_HALT(1)   //��ͣCLK_OUT���
                       | SYSCON_CLKOUTDIV_DIV(200-1)//���÷�Ƶϵ��
                       | SYSCON_CLKOUTDIV_RESET(1));//��λCLK_OUT��Ƶ���Լ����µķ�Ƶϵ��
    while(!(SYSCON->CLKOUTDIV & SYSCON_CLKOUTDIV_REQFLAG_MASK));//�ȴ���Ƶϵ���������
    SYSCON->CLKOUTDIV = (0
                       | SYSCON_CLKOUTDIV_HALT(0)   //�ָ�CLK_OUT���
                       | SYSCON_CLKOUTDIV_DIV(200-1)//���÷�Ƶϵ��
                       | SYSCON_CLKOUTDIV_RESET(0)  //���㸴λ
                       | SYSCON_CLKOUTDIV_REQFLAG(0));//���������ɱ�־
}

void SystemInit (void) 
{
    extern void *__Vectors;

#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))         
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));      //����FPU
#endif 
    
    if(LPC546XX_CLOSE_ISP)
    {
        SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_OTP_MASK;      //��OTPʱ��
        SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_OTP_RST_SHIFT; //�����λʱ��
        (void)OTP_API->otpInit();   
        (void)OTP_API->otpEnableBankReadLock(3,1,0,0);          //�����ȡ
        if(!(OTPC->ECRP & OTPC_ECRP_CRP_ISP_DISABLE_PIN_MASK))  //���û�н���ISP�������ISPͨ����������
        {
            (void)OTP_API->otpInit();                           //��ʼ��
            (void)OTP_API->otpEnableBankWriteMask(0x8);         //����3����
            (void)OTP_API->otpEnableBankWriteLock(3,1,0,0);     //����д��
            (void)OTP_API->otpProgramReg(3,0,0x40);
        }
        (void)OTP_API->otpDisableBankWriteMask(0x8);            //����3����
        SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_OTP_MASK;  //�ر�OTPʱ��
    }
    
    
    SCB->VTOR = (uint32_t) &__Vectors;

    SYSCON->ARMTRACECLKDIV = 0; //��traceʱ��

    SYSCON->AHBCLKCTRLSET[0] = (0                       
                              | SYSCON_AHBCLKCTRL_SRAM1_MASK    //��SRAM1ʱ��
                              | SYSCON_AHBCLKCTRL_SRAM2_MASK    //��SRAM2ʱ��
                              | SYSCON_AHBCLKCTRL_SRAM3_MASK);  //��SRAM3ʱ��

    pll_init(); //PLL��ʼ����ѡ��PLL�����Ϊmain_clk����
  
#if (1 == CLK_OUT)
    clk_out();  //ͨ��P026 ���main_clk/200ʱ��   P026�����Ƶ�ʳ�200����main_clkƵ��
#endif
    

}

void NMI_Handler(void)
{
    ;
}

void HardFault_Handler(void)
{
#if defined(PRINTF) && defined(DEBUG_ASSERT)
    printf("Ӳ���Ϸã����ܷ�����δ��ʼ�������裬��������Խ�����");
#endif
    while(1);
}
