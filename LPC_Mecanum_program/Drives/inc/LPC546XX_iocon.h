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

#ifndef _LPC546XX_iocon_h
#define _LPC546XX_iocon_h







//��ö�ٶ��岻�����û��޸�
typedef enum //ö�ٶ˿�״̬
{
    //����IO���������ø��ù���
    ALT0            = IOCON_PIO_FUNC(0),    //ͨ��IO
    ALT1            = IOCON_PIO_FUNC(1),    //���ù���1
    ALT2            = IOCON_PIO_FUNC(2),    //���ù���2 
    ALT3            = IOCON_PIO_FUNC(3),    //���ù���3
    ALT4            = IOCON_PIO_FUNC(4),    //���ù���4
    ALT5            = IOCON_PIO_FUNC(5),    //���ù���5
    ALT6            = IOCON_PIO_FUNC(6),    //���ù���6
    ALT7            = IOCON_PIO_FUNC(7),    //���ù���7

    
    //PIO0_13, PIO0_14, PIO3_23, PIO3_24��������IO��������������������
    NOPULL          = IOCON_PIO_MODE(0),    //û������������
    PULLDOWN        = IOCON_PIO_MODE(1),    //����
    PULLUP          = IOCON_PIO_MODE(2),    //����
    REPEATER        = IOCON_PIO_MODE(3),    //�м�ģʽ �м�ģʽ�ɲ����ֲ�182ҳ�е�ͼ16
            
    
    //����IO���������õ��ù���
    INVERT          = IOCON_PIO_INVERT(1),  //�źŵ���
    
    
    //����IO��������ģ��������ģʽ֮���л�
    ANALOG          = IOCON_PIO_DIGIMODE(0),//ģ��ģʽ
    DIGITAL         = IOCON_PIO_DIGIMODE(1),//����ģʽ
    
    
    //����IO�����Թرջ������������˲�
    FILTEROFF       = IOCON_PIO_FILTEROFF(1),//�ر������˲�
    
    
    //PIO0_13, PIO0_14, PIO3_23, PIO3_24, P0_10 to P0_12, P0_15 to P0_16, P0_23, P0_31], P1_0,��������IO����������IOת������
    STANDARD_SLEW   = IOCON_PIO_SLEW(0),    //��׼ת����
    FAST_SLEW       = IOCON_PIO_SLEW(1),    //����ת����
    
    
    //PIO0_13, PIO0_14, PIO3_23, PIO3_24��������IO���������ÿ�©
    OD              = IOCON_PIO_OD(1),      //��©
    
    
    
     
    //�������ý���PIO0_13, PIO0_14, PIO3_23, PIO3_24ʹ��
    //PIO0_13, PIO0_14, PIO3_23, PIO3_24Ϊ��©���IO���޷���������������
    I2CMODE         = IOCON_PIO_I2CSLEW(0), //i2cģʽ
    GPIOMODE        = IOCON_PIO_I2CSLEW(0), //gpioģʽ
    
    LOW_DRIVE       = IOCON_PIO_I2CDRIVE(0),//���������� ���ڱ�׼�Ϳ���i2c�ٶ���˵����������
    HIGH_DRIVE      = IOCON_PIO_I2CDRIVE(1),//����������
    
    I2CFILTEROFF    = IOCON_PIO_I2CFILTER(1),//�ر�50nsë���˲���    ����i2cģʽ��Ҫ����50nsë���˲���
    
}PINCONF_enum;



void iocon_init(PIN_enum pin, uint32 cfg);

void iocon_init_noalt(PIN_enum pin, uint32 cfg);




#endif

