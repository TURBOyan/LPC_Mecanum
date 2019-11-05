/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		SCTimer/PWM
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#ifndef _LPC546XX_sct_h
#define _LPC546XX_sct_h

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    SCT_INPUT_MUX0,
    SCT_INPUT_MUX1,
    SCT_INPUT_MUX2,
    SCT_INPUT_MUX3,
    SCT_INPUT_MUX4,
    SCT_INPUT_MUX5,
    SCT_INPUT_MUX6,
    SCT_INPUT_MUXMAX,    //ͨ������
}SCT_INPUT_MUX_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum
{
    SCT0_GPI0_A0  = 0*4,  SCT0_GPI0_A13, SCT0_GPI0_A24, SCT0_GPI0_B5,       //SCT0_GPI0
    SCT0_GPI1_A1  = 1*4,  SCT0_GPI1_A14, SCT0_GPI1_A25,                     //SCT0_GPI1
    SCT0_GPI2_A2  = 2*4,  SCT0_GPI2_A20,                                    //SCT0_GPI2
    SCT0_GPI3_A3  = 3*4,  SCT0_GPI3_A21, SCT0_GPI3_B6,                      //SCT0_GPI3
    SCT0_GPI4_A4  = 4*4,  SCT0_GPI4_B0,  SCT0_GPI4_B7,                      //SCT0_GPI4
    SCT0_GPI5_A5  = 5*4,  SCT0_GPI5_B1,  SCT0_GPI5_B22,                     //SCT0_GPI5
    SCT0_GPI6_A6  = 6*4,  SCT0_GPI6_B2,  SCT0_GPI6_B29,                     //SCT0_GPI6
    SCT0_GPI7_A12 = 7*4,  SCT0_GPI7_A17, SCT0_GPI7_B19, SCT0_GPI7_B30,      //SCT0_GPI7
}SCT_IUPUT_PIN_enum;


#define SCT0_CAMERA_PCLK  SCT_INPUT_MUX1    //��������ͷ����ʱ����ʹ�õ�SCT����ͨ��



//��ö�ٶ��岻�����û��޸�
typedef enum
{
    SCT_OUTPUT_CH0,
    SCT_OUTPUT_CH1,
    SCT_OUTPUT_CH2,
    SCT_OUTPUT_CH3,
    SCT_OUTPUT_CH4,
    SCT_OUTPUT_CH5,
    SCT_OUTPUT_CH6,
    SCT_OUTPUT_CH7,
    SCT_OUTPUT_CH8,
    SCT_OUTPUT_CH9,
    SCT_OUTPUT_CHMAX,    //ͨ������
}SCT_OUTPUT_CH_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum
{
    SCT0_OUT0_A2  = 0*4, SCT0_OUT0_A17, SCT0_OUT0_B4,  SCT0_OUT0_B23,       //SCT0_OUT0 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT1_A3  = 1*4, SCT0_OUT1_A18, SCT0_OUT1_B8,  SCT0_OUT1_B24,       //SCT0_OUT1 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT2_A15 = 2*4, SCT0_OUT2_A19, SCT0_OUT2_B9,  SCT0_OUT2_B25,       //SCT0_OUT2 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT3_A22 = 3*4, SCT0_OUT3_A31, SCT0_OUT3_B10, SCT0_OUT3_B26,       //SCT0_OUT3 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT4_A23 = 4*4, SCT0_OUT4_B3,  SCT0_OUT4_B17,                      //SCT0_OUT4 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT5_A26 = 5*4, SCT0_OUT5_B18,                                     //SCT0_OUT5 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT6_A27 = 6*4, SCT0_OUT6_B31,                                     //SCT0_OUT6 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT7_A28 = 7*4, SCT0_OUT7_B19,                                     //SCT0_OUT7 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT8_A29 = 8*4,                                                    //SCT0_OUT8 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
    SCT0_OUT9_A30 = 9*4,                                                    //SCT0_OUT9 ͨ����ѡ��Χ ͬһʱ��ÿһ��ͨ��ֻ����һ�������������PWM
}SCT_OUTPUT_PIN_enum;




#define SCT0_PWM_PERIOD_CH  SCT_OUTPUT_CH0  //���������¼���ռ�õ�ͨ��������ͨ�������������PWM�� SCT_OUTPUT_CH1Ĭ�ϱ�����ͷPCLKռ��





//���ڶ�ʱ��ʹ�õ���16λ�ģ���Ϊ�����õ����ռ�ձȳ���0XFFFF��û��ʵ�����壬������ͽ������������
//ͨ������������ռ�ձ�����Ϊ1000���㹻��
#define SCT0_OUTPUT_CH0_DUTY_MAX     10000                                   //SCT0 OUTPUT0 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH1_DUTY_MAX     10000                                   //SCT0 OUTPUT1 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH2_DUTY_MAX     10000                                   //SCT0 OUTPUT2 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH3_DUTY_MAX     10000                                   //SCT0 OUTPUT3 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH4_DUTY_MAX     10000                                   //SCT0 OUTPUT4 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH5_DUTY_MAX     10000                                   //SCT0 OUTPUT5 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH6_DUTY_MAX     10000                                   //SCT0 OUTPUT6 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH7_DUTY_MAX     10000                                   //SCT0 OUTPUT7 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH8_DUTY_MAX     10000                                   //SCT0 OUTPUT8 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС
#define SCT0_OUTPUT_CH9_DUTY_MAX     10000                                   //SCT0 OUTPUT9 PWM���ռ�ձ�  ���ռ�ձ�Խ��ռ�ձȵĲ���ֵԽС



#define READ_SCT_FLAG   SCT0->EVFLAG
#define CLEAR_SCT_FLAG(FLAG)  (SCT0->EVFLAG = FLAG)


void sct_camera_dma(SCT_INPUT_MUX_enum pclk_mux_ch, SCT_IUPUT_PIN_enum pclk_input_pin, TRIGGER_enum pclk_trigger);
void sct_start(void);
void sct_stop(void);
void sct_restart(void);
void sct_pwm_init(SCT_OUTPUT_PIN_enum pwm_pin, uint32 freq, uint32 duty);
void sct_pwm_duty(SCT_OUTPUT_PIN_enum pwm_pin, uint32 duty);

#endif

