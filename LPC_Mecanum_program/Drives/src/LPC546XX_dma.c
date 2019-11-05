/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		DMA
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
#include "LPC546XX_pint.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_sct.h"
#include "LPC546XX_dma.h"


ALIGN(512) dma_descriptor_t s_dma_descriptor_table[DMA_CHMAX] = {0};//DMAͨ��������


//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA��ʼ��
//  @param      dmach       DMAͨ��
//  @param      *SADDR      Դ��ַ
//  @param      *DADDR      Ŀ�ĵ�ַ
//  @param      count       DMA�������
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //��ʼ��DMA  ͨ��0   Դ��ַΪA0-A7  Ŀ�ĵ�ַΪimage������׵�ַ   �����ֽ���Ϊ188��
//-------------------------------------------------------------------------------------------------------------------
void dam_init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //����DMA��������ͨ�� ΪSCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
    
    temp_pin = ((uint32)SADDR - (uint32)&GPIO_PIN(0,0)) * 8;
    n = 8;
    while(n--)
    {
        gpio_init((PIN_enum)(temp_pin+n),GPI,0,PULLUP | FILTEROFF);
    }

    DMA0->SRAMBASE = (uint32_t)s_dma_descriptor_table;
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :���ش���
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst����Ϊһ���ֽ�
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    s_dma_descriptor_table[dmach].xfercfg = ( 0
                                   //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                   | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                   | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                   | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                   | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                   | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                   | DMA_CHANNEL_XFERCFG_XFERCOUNT(count-1) //DMA����
                                   );
    
    s_dma_descriptor_table[dmach].srcEndAddr = SADDR;
    s_dma_descriptor_table[dmach].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[dmach].linkToNextDesc = 0;
    
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      DMA ���Ӵ��� ��ʼ��
//  @param      dmach       DMAͨ��
//  @param      *SADDR      Դ��ַ
//  @param      *DADDR      Ŀ�ĵ�ַ
//  @param      count       DMA�������
//  @return     void
//  Sample usage:           dam_init(DMA_CH0, (void *)&GPIO_PIN(0,0), (void *)&image[0][0], 188);     //��ʼ��DMA  ͨ��0   Դ��ַΪA0-A7  Ŀ�ĵ�ַΪimage������׵�ַ   �����ֽ���Ϊ188��
//  @note                   ʹ�����Ӵ������ʵ�� һ���Դ��䳬��1024 ����ռ������DMAͨ���� ͨ��������
//-------------------------------------------------------------------------------------------------------------------
void dam_init_linked(DMACH_enum dmach, void *SADDR, void *DADDR, uint32 count)
{
    uint8  n;
    uint32 temp_pin;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_DMA_MASK;              //��DMAʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_DMA0_RST_MASK;         //���DMA��λʱ��
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�򿪶�·����ʱ��
    INPUTMUX->DMA_ITRIG_INMUX[dmach] = INPUTMUX_DMA_ITRIG_INMUX_INP(2); //����DMA��������ͨ�� ΪSCT request0
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;         //�رն�·����ʱ��
    
    temp_pin = ((uint32)SADDR - (uint32)&GPIO_PIN(0,0)) * 8;
    n = 8;
    while(n--)
    {
        gpio_init((PIN_enum)(temp_pin+n),GPI,0,PULLUP | FILTEROFF);
    }

    DMA0->SRAMBASE = (uint32_t)s_dma_descriptor_table;
    DMA0->CTRL = DMA_CTRL_ENABLE_MASK;
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    
    DMA0->CHANNEL[dmach].CFG = ( 0
                               | DMA_CHANNEL_CFG_HWTRIGEN_MASK
                               | DMA_CHANNEL_CFG_TRIGPOL_MASK       //1 ������
                                //| DMA_CHANNEL_CFG_TRIGTYPE_MASK   //0 :���ش���
                               | DMA_CHANNEL_CFG_TRIGBURST_MASK     //����burst����
                               | DMA_CHANNEL_CFG_BURSTPOWER(0)      //burst����Ϊһ���ֽ�
                               | DMA_CHANNEL_CFG_CHPRIORITY(0)      //���ȼ�����   0Ϊ���
                               );
    
    DMA0->COMMON[0].SETVALID = 1<<dmach;
    DMA0->COMMON[0].INTENSET = 1<<dmach;
    
    n = 0;
    while((n+1)<<10 < count)//ʣ����������1024
    {
        s_dma_descriptor_table[n].xfercfg = ( 0
                                            | DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                            | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                            | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                            | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                            | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                            | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                            | DMA_CHANNEL_XFERCFG_XFERCOUNT(1024-1)  //DMA����
                                            );
        s_dma_descriptor_table[n].srcEndAddr = SADDR;
        s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + ((n+1)<<10) - 1);
        s_dma_descriptor_table[n].linkToNextDesc = (void *)(&s_dma_descriptor_table[n+1]);
        n++;
    }
    
    //ʣ������������ߵ���1024  
    s_dma_descriptor_table[n].xfercfg = ( 0
                                        //| DMA_CHANNEL_XFERCFG_RELOAD_MASK        //�����Զ�����
                                        | DMA_CHANNEL_XFERCFG_CFGVALID_MASK
                                        | DMA_CHANNEL_XFERCFG_SETINTA_MASK       //
                                        | DMA_CHANNEL_XFERCFG_WIDTH(0)           //���8λ
                                        | DMA_CHANNEL_XFERCFG_SRCINC(0)          //Դ��ַ������
                                        | DMA_CHANNEL_XFERCFG_DSTINC(1)          //Ŀ�ĵ�ַ����һ�����ݿ��
                                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - (n<<10) - 1)  //DMA����
                                        );
    s_dma_descriptor_table[n].srcEndAddr = SADDR;
    s_dma_descriptor_table[n].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    s_dma_descriptor_table[n].linkToNextDesc = 0;
        
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}





