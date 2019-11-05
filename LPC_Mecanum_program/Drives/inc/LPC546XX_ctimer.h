/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		��׼����/��ʱ��
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#ifndef _LPC546XX_ctimer_h
#define _LPC546XX_ctimer_h


//��ö�ٶ��岻�����û��޸�
typedef enum
{
    TIMER0,
    TIMER1,
    TIMER2,
    TIMER3,
    TIMER4,
}CTIMER_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    TIMER0_PWMCH0_A0=0,       TIMER0_PWMCH0_A30,      //��ʱ��0  0ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER0_PWMCH1_A3=2,       TIMER0_PWMCH1_A31,      //��ʱ��0  1ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER0_PWMCH2_A19=4,                              //��ʱ��0  2ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER0_PWMCH3_B2=6,       TIMER0_PWMCH3_B27,      //��ʱ��0  3ͨ�����PWM  ���ſ�ѡ��Χ
                                                      
    TIMER1_PWMCH0_A18=8,      TIMER1_PWMCH0_B10,      //��ʱ��1  0ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER1_PWMCH1_A20=10,     TIMER1_PWMCH1_B12,      //��ʱ��1  1ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER1_PWMCH2_A23=12,     TIMER1_PWMCH2_B14,      //��ʱ��1  2ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER1_PWMCH3_B16=14,                             //��ʱ��1  3ͨ�����PWM  ���ſ�ѡ��Χ
                                                      
    TIMER2_PWMCH0_A10=16,     TIMER2_PWMCH0_B5,       //��ʱ��2  0ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER2_PWMCH1_B4=18,      TIMER2_PWMCH1_B6,       //��ʱ��2  1ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER2_PWMCH2_A11=20,     TIMER2_PWMCH2_B7,       //��ʱ��2  2ͨ�����PWM  ���ſ�ѡ��Χ��A11�����ؽӿڳ�ͻ��
    TIMER2_PWMCH3_A29=22,     TIMER2_PWMCH3_B22,      //��ʱ��2  3ͨ�����PWM  ���ſ�ѡ��Χ
                                            
    TIMER3_PWMCH0_A5=24,                              //��ʱ��3  0ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER3_PWMCH1_B19=26,                             //��ʱ��3  1ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER3_PWMCH2_A27=28,     TIMER3_PWMCH2_B21,      //��ʱ��3  2ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER3_PWMCH3_A21=30,     TIMER3_PWMCH3_A23,      //��ʱ��3  3ͨ�����PWM  ���ſ�ѡ��Χ
                    
    TIMER4_PWMCH0_A6=32,                              //��ʱ��4  0ͨ�����PWM  ���ſ�ѡ��Χ
    TIMER4_PWMCH1_P4_14=34,                           //100����LPC �޴�����ռλ��������Ϊ����ͨ��
        
}CTIMER_PWMCH_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum
{
    //ÿһ����ʱ����ͬһʱ��ֻ�ܶ�һ�����ż���
    TIMER0_COUNT0_A1=0,       TIMER0_COUNT0_A13,     //��ʱ��0  0ͨ��������������  ���ſ�ѡ��Χ
    TIMER0_COUNT1_A2,         TIMER0_COUNT1_A14=2,   //��ʱ��0  1ͨ��������������  ���ſ�ѡ��Χ
    TIMER0_COUNT2_A28=4,                              //��ʱ��0  2ͨ��������������  ���ſ�ѡ��Χ
    TIMER0_COUNT3_B1=6,       TIMER0_COUNT3_B26,      //��ʱ��0  3ͨ��������������  ���ſ�ѡ��Χ
    //ÿһ����ʱ����ͬһʱ��ֻ�ܶ�һ�����ż���                                                                    
    TIMER1_COUNT0_A16,        TIMER1_COUNT0_B9=8,     //��ʱ��1  0ͨ��������������  ���ſ�ѡ��Χ
    TIMER1_COUNT1_B11=10,                             //��ʱ��1  1ͨ��������������  ���ſ�ѡ��Χ
    TIMER1_COUNT2_B13=12,                             //��ʱ��1  2ͨ��������������  ���ſ�ѡ��Χ
    TIMER1_COUNT3_B15=14,                             //��ʱ��1  3ͨ��������������  ���ſ�ѡ��Χ
    //ÿһ����ʱ����ͬһʱ��ֻ�ܶ�һ�����ż���                                                                    
    TIMER2_COUNT0_A24=16,                             //��ʱ��2  0ͨ��������������  ���ſ�ѡ��Χ
    TIMER2_COUNT1_A25=18,                             //��ʱ��2  1ͨ��������������  ���ſ�ѡ��Χ
    TIMER2_COUNT2_A10=20,                             //��ʱ��2  2ͨ��������������  ���ſ�ѡ��Χ
    TIMER2_COUNT3_A28=22,                             //��ʱ��2  3ͨ��������������  ���ſ�ѡ��Χ
    //ÿһ����ʱ����ͬһʱ��ֻ�ܶ�һ�����ż���                                                                    
    TIMER3_COUNT0_A4=24,                              //��ʱ��3  0ͨ��������������  ���ſ�ѡ��Χ
    TIMER3_COUNT1_A6=26,                              //��ʱ��3  1ͨ��������������  ���ſ�ѡ��Χ
    TIMER3_COUNT2_A26=28,     TIMER3_COUNT2_B20,      //��ʱ��3  2ͨ��������������  ���ſ�ѡ��Χ
    TIMER3_COUNT3_A20=30,     TIMER3_COUNT3_A22,      //��ʱ��3  3ͨ��������������  ���ſ�ѡ��Χ
                                                                        
    TIMER4_COUNT0_A15=32,                             //��ʱ��4  0ͨ��������������  ���ſ�ѡ��Χ
        
}CTIMER_COUNTCH_enum;

#define TIMER0_PWM_PERIOD_CH    TIMER0_PWMCH2_A19     //��ʱ��0 PWM����ռ�õĵ�ͨ������ռ�õ�ͨ�������������PWM��������ͨ����������Ҳ�������PWM��
#define TIMER1_PWM_PERIOD_CH    TIMER1_PWMCH3_B16     //��ʱ��1 PWM����ռ�õĵ�ͨ������ռ�õ�ͨ�������������PWM��������ͨ����������Ҳ�������PWM��
#define TIMER2_PWM_PERIOD_CH    TIMER2_PWMCH3_B22     //��ʱ��2 PWM����ռ�õĵ�ͨ������ռ�õ�ͨ�������������PWM��������ͨ����������Ҳ�������PWM��
#define TIMER3_PWM_PERIOD_CH    TIMER3_PWMCH0_A5      //��ʱ��3 PWM����ռ�õĵ�ͨ������ռ�õ�ͨ�������������PWM��������ͨ����������Ҳ�������PWM��
#define TIMER4_PWM_PERIOD_CH    TIMER4_PWMCH1_P4_14   //��ʱ��4 PWM����ռ�õĵ�ͨ������ռ�õ�ͨ�������������PWM��������ͨ����������Ҳ�������PWM��



#define TIMER0_PWM_DUTY_MAX     10000                 //��ʱ��0 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define TIMER1_PWM_DUTY_MAX     10000                 //��ʱ��1 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define TIMER2_PWM_DUTY_MAX     10000                 //��ʱ��2 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define TIMER3_PWM_DUTY_MAX     10000                 //��ʱ��3 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define TIMER4_PWM_DUTY_MAX     10000                 //��ʱ��4 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС



void    ctimer_pwm_init(CTIMER_PWMCH_enum pwmch, uint32 freq, uint32 duty);
void    ctimer_pwm_duty(CTIMER_PWMCH_enum pwmch, uint32 duty);
void    ctimer_pwm_freq(CTIMER_PWMCH_enum pwmch, uint32 freq);

void    ctimer_count_init(CTIMER_COUNTCH_enum countch);
uint32  ctimer_count_read(CTIMER_COUNTCH_enum countch);
void    ctimer_count_clean(CTIMER_COUNTCH_enum countch);

#endif
