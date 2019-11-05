/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		MRT(�����ʶ�ʱ��)
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-11-21
 ********************************************************************************************************************/


#ifndef _LPC546XX_mrt_h
#define _LPC546XX_mrt_h


//��ö�ٶ��岻�����û��޸�
typedef enum
{
    MRT_CH0,
    MRT_CH1,
    MRT_CH2,
    MRT_CH3,
}MRTNUM_enum;



#define MRT_FLAG_READ(MRTNUM)  (MRT0->CHANNEL[MRTNUM].STAT&MRT_CHANNEL_STAT_INTFLAG_MASK)
#define MRT_FLAG_CLR(MRTNUM)   MRT0->CHANNEL[MRTNUM].STAT = 1; MRT0->IRQ_FLAG = (1<<MRTNUM)


                 
//------------------------------------���´�������PIT�ж�------------------------------------
#define mrt_pit_init_ms(MRTNUM,time)    mrt_pit_init(MRTNUM,time*main_clk_mhz*1000) //��ʼ��MRT��ʱ��ΪPITģʽ����������ʱ�� (��λ ����)    �������Ϊ180MΪ93.2����
#define mrt_pit_init_us(MRTNUM,time)    mrt_pit_init(MRTNUM,time*main_clk_mhz)      //��ʼ��MRT��ʱ��ΪPITģʽ����������ʱ�� (��λ ΢��)    �������Ϊ180MΪ93.2����
#define mrt_pit_init_ns(MRTNUM,time)    mrt_pit_init(MRTNUM,time*main_clk_mhz/1000) //��ʼ��MRT��ʱ��ΪPITģʽ����������ʱ�� (��λ ����)    �������Ϊ180MΪ93.2���룬��С��λΪ5.55����


//------------------------------------���´�������PIT��ʱ------------------------------------
#define mrt_delay_ms(MRTNUM,time)       mrt_delay(MRTNUM,time*main_clk_mhz*1000)    //��ʼ��MRT��ʱ��Ϊ��ʱģʽ����������ʱ�� (��λ ����)   �����ʱ180MΪ93.2����
#define mrt_delay_us(MRTNUM,time)       mrt_delay(MRTNUM,time*main_clk_mhz)         //��ʼ��MRT��ʱ��Ϊ��ʱģʽ����������ʱ�� (��λ ΢��)   �����ʱ180MΪ93.2����
#define mrt_delay_ns(MRTNUM,time)       mrt_delay(MRTNUM,time*main_clk_mhz/1000)    //��ʼ��MRT��ʱ��Ϊ��ʱģʽ����������ʱ�� (��λ ����)   �����ʱ180MΪ93.2���룬��С��λΪ5.55����

//------------------------------------���´������ڻ�ȡPIT��ʱʱ��------------------------------------
#define mrt_get_ms(MRTNUM)              (mrt_get(MRTNUM)/main_clk_mhz/1000)         //��ȡMRT��ʱʱ��  (��λ ����) 
#define mrt_get_us(MRTNUM)              (mrt_get(MRTNUM)/main_clk_mhz)              //��ȡMRT��ʱʱ��  (��λ ΢��) 
#define mrt_get_ns(MRTNUM)              (mrt_get(MRTNUM)*1000/main_clk_mhz)         //��ȡMRT��ʱʱ��  (��λ ����) 



void mrt_pit_init(MRTNUM_enum mrtchx, uint32 time);
void mrt_delay(MRTNUM_enum mrtchx, uint32 time);
void mrt_start(MRTNUM_enum mrtchx);
uint32 mrt_get(MRTNUM_enum mrtchx);
void mrt_clean(MRTNUM_enum mrtchx);


#endif

