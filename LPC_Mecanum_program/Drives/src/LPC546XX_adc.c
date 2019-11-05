/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ADC
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
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_adc.h"

void adc_mux(ADCCH_enum ch)
{
    switch(ch)
    {
        case ADC_CH0_A10:   iocon_init(A10,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH1_A11:   iocon_init(A11,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH2_A12:   iocon_init(A12,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH3_A15:   iocon_init(A15,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH4_A16:   iocon_init(A16,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH5_A31:   iocon_init(A31,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH6_B0 :   iocon_init(B0 ,ALT0 | NOPULL | ANALOG | FILTEROFF); break;
        case ADC_CH11_A23:  iocon_init(A23,ALT0 | NOPULL | ANALOG | FILTEROFF); break;

        default:        ASSERT(0);//ͨ������ �������ʧ��
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC��ʼ��
//  @param      ch          ADCͨ����
//  @return     void
//  Sample usage:           adc_init(ADC_CH0_A10);     // ��ʼ��ADCͨ��0  �ֱ���Ϊ12λ   ͨ����Ӧ������ΪA10
//-------------------------------------------------------------------------------------------------------------------
void adc_init(ADCCH_enum ch)
{
    uint16 temp_div;
    
    
    SYSCON->PDRUNCFGCLR[0] = ( 0
                             | SYSCON_PDRUNCFGCLR_PDEN_ADC0_MASK 
                             | SYSCON_PDRUNCFGCLR_PDEN_VD2_ANA_MASK 
                             | SYSCON_PDRUNCFGCLR_PDEN_VDDA_MASK
                             | SYSCON_PDRUNCFGCLR_PDEN_VREFP_MASK
                             ); //��ADC��Դ
    systick_delay_us(20);                                       //��Ҫ��ʱ
    
    
    SYSCON->ADCCLKSEL = SYSCON_ADCCLKSEL_SEL(0x01);             //ѡ��ADCʱ��Դ
    SYSCON->ADCCLKDIV = 0;  
    
    SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_HALT_MASK;  
    //SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_HALT_MASK | SYSCON_ADCCLKDIV_DIV(0) | SYSCON_ADCCLKDIV_RESET_MASK;  
    //while(!(SYSCON_ADCCLKDIV_REQFLAG_MASK & SYSCON->ADCCLKDIV));
    SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_DIV(0);
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_ADC0_MASK;     //��ADCʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_ADC0_RST_MASK; //�����λADCʱ��
    
    
    adc_mux(ch);

    temp_div = (main_clk_mhz*100/80 + 99)/100;
    ADC0->CTRL = ( 0
                 | ADC_CTRL_CLKDIV(temp_div-1)      //��Ƶ��󲻳���80M
                 //| ADC_CTRL_ASYNMODE_MASK         //ģʽ      0:ͬ��ģʽ    1���첽ģʽ   ����Ϊ0
                 | ADC_CTRL_RESOL(0x3)              //Ĭ��12λ�ֱ���
                 //| ADC_CTRL_BYPASSCAL_MASK        //����У׼  0:����У׼����    1���ر�У׼   ����Ϊ0
                 | ADC_CTRL_TSAMP(0)                //��������
                 );

    ADC0->STARTUP = ADC_STARTUP_ADC_ENA_MASK;           //����ADC
    systick_delay_us(10);                               //��Ҫ��ʱ
    if (!(ADC0->STARTUP & ADC_STARTUP_ADC_ENA_MASK))
    {
        ASSERT(0);//ADCû���ϵ� �������ʧ��
    }
    
    
    ADC0->CALIB = ADC_CALIB_CALIB_MASK;                 //ADCУ׼
    while(ADC_CALIB_CALIB_MASK == (ADC0->CALIB & ADC_CALIB_CALIB_MASK));
    
    ADC0->STARTUP |= ADC_STARTUP_ADC_INIT_MASK;         //ADC��ʼ��
    while(ADC_STARTUP_ADC_INIT_MASK == (ADC0->STARTUP & ADC_STARTUP_ADC_INIT_MASK));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADCת��
//  @param      ch          ADCͨ����
//  @param      resolution  ADC�ֱ���
//  @return     void
//  Sample usage:           adc_test = adc_convert(ADC_CH0_A10,ADC_12BIT);     //�ɼ�ADCͨ��0��ѹֵ���ֱ���12λ     ��ѹ=adc_test*3300/2^n*(����)   nΪADC�ķֱ���
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_convert(ADCCH_enum ch, ADCRES_enum resolution)
{
    ADC0->CTRL &= ~ADC_CTRL_RESOL_MASK;
    ADC0->CTRL |= ADC_CTRL_RESOL(resolution);   //�ֱ���
    
    ADC0->SEQ_CTRL[1] = 0;
    ADC0->SEQ_CTRL[1] =( 0
                        | ADC_SEQ_CTRL_CHANNELS(1<<ch)  //����ͨ��
                        | ADC_SEQ_CTRL_SEQ_ENA_MASK
                        | ADC_SEQ_CTRL_TRIGPOL_MASK
                        | ADC_SEQ_CTRL_SINGLESTEP_MASK
                       );

    ADC0->SEQ_CTRL[1] |= ADC_SEQ_CTRL_START_MASK;               //����ADCת��

    while(!(ADC_SEQ_GDAT_DATAVALID_MASK & ADC0->SEQ_GDAT[1]));  //�ȴ�ת�����
    return ((ADC0->SEQ_GDAT[1]&ADC_SEQ_GDAT_RESULT_MASK)>>(ADC_SEQ_GDAT_RESULT_SHIFT+(3-resolution)*2));
}





