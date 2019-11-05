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

#ifndef _LPC546XX_adc_h
#define _LPC546XX_adc_h

//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
    ADC_CH0_A10,    //A10
    ADC_CH1_A11,    //A11   �����ؿڳ�ͻ
    ADC_CH2_A12,    //A12   �����ؿڳ�ͻ
    ADC_CH3_A15,    //A15
    ADC_CH4_A16,    //A16
    ADC_CH5_A31,    //A31
    ADC_CH6_B0 ,    //B0
    ADC_CH7,        //P2_0    LQFP100��װû���������
    ADC_CH8,        //P2_1    LQFP100��װû���������
    ADC_CH9,        //P3_21   LQFP100��װû���������
    ADC_CH10,       //P3_22   LQFP100��װû���������
    ADC_CH11_A23,   //A23   
}ADCCH_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
    ADC_6BIT,     //6λ�ֱ���
    ADC_8BIT,     //8λ�ֱ���
    ADC_10BIT,    //10λ�ֱ���
    ADC_12BIT,    //12λ�ֱ���

}ADCRES_enum;


void    adc_init(ADCCH_enum ch);
uint16  adc_convert(ADCCH_enum ch, ADCRES_enum resolution);


#endif
