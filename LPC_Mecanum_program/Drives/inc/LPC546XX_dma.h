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


#ifndef _LPC546XX_dma_h
#define _LPC546XX_dma_h

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    DMA_CH0,
    DMA_CH1,
    DMA_CH2,
    DMA_CH3,
    DMA_CH4,
    DMA_CH5,
    DMA_CH6,
    DMA_CH7,
    DMA_CH8,
    DMA_CH9,
    DMA_CH10,
    DMA_CH11,
    DMA_CH12,
    DMA_CH13,
    DMA_CH14,
    DMA_CH15,
    DMA_CH16,
    DMA_CH17,
    DMA_CH18,
    DMA_CH19,
    DMA_CH20,
    DMA_CH21,
    DMA_CH22,
    DMA_CH23,
    DMA_CH24,
    DMA_CH25,
    DMA_CH26,
    DMA_CH27,
    DMA_CH28,
    DMA_CH29,
    DMA_CHMAX,  //ͨ������
}DMACH_enum;

//��ö�ٶ��岻�����û��޸�
typedef struct _dma_descriptor
{
    uint32_t xfercfg;     //��������
    void *srcEndAddr;     //Դ��ַ�յ�
    void *dstEndAddr;     //Ŀ�ĵ�ַ�յ�
    void *linkToNextDesc; //DMA��һ����������ַ
} dma_descriptor_t;



ALIGN(512) extern dma_descriptor_t s_dma_descriptor_table[DMA_CHMAX];//DMAͨ��������




#define CLEAR_DMA_FLAG(n)   (DMA0->COMMON[0].INTA = 1<<n)       //DMA�жϱ�־λ���� nΪDMAͨ�����
#define READ_DMA_FLAG(n)    ((DMA0->COMMON[0].INTA & 1<<n)>>n)  //��ȡDMA�жϱ�־λ nΪDMAͨ�����


#define DMA_ENABLE(dmach)   (DMA0->CHANNEL[dmach].CFG |= DMA_CHANNEL_CFG_HWTRIGEN_MASK)            //����Ӳ������
#define DMA_DISABLE(dmach)  (DMA0->CHANNEL[dmach].CFG &= ~(uint32)DMA_CHANNEL_CFG_HWTRIGEN_MASK)   //����Ӳ������
#define DMA_ABORT(dmach)    (DMA0->COMMON[0].ABORT = 1 << dmach)                                   //ȡ����ǰDMA����
#define DMA_STATUS(dmach)   (DMA0->COMMON[0].ACTIVE & (1<<dmach))                                  //��ȡ��ǰDMA����״̬

//��ȡDMA��ǰʣ�ഫ�����
#define DMA_XFERCOUNT(dmach)  ((DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK & DMA0->CHANNEL[dmach].XFERCFG)>>DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT)
//��ȡ��ǰ������Ч��
#define DMA_CFGVALID(dmach)   (DMA_CHANNEL_XFERCFG_CFGVALID_MASK & DMA0->CHANNEL[dmach].XFERCFG)


//ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
__STATIC_INLINE void dma_reload(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count)//DMA��������  �������ò���������ó�ʼ��
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;
    DMA0->COMMON[0].ABORT = 1<<dmach;
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
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[dmach].xfercfg;
}

//ʹ��__STATIC_INLINEΪ�˽���κ�����Ƕ��ʹ�øú����ĵط����������Լ��ٺ������õ�ʱ��
__STATIC_INLINE void dma_reload_linked(DMACH_enum dmach, void *DADDR, uint32 count)//DMA��������  �������ò���������ó�ʼ��
{
    DMA0->COMMON[0].ENABLECLR = 1<<dmach;
    DMA0->COMMON[0].ABORT = 1<<dmach;
    s_dma_descriptor_table[0].linkToNextDesc = (void *)(&s_dma_descriptor_table[1]);
    if(count>1024)  s_dma_descriptor_table[dmach].dstEndAddr = (void *)((uint32)DADDR + 1024 - 1);
    else            s_dma_descriptor_table[dmach].dstEndAddr = (void *)((uint32)DADDR + count - 1);
    DMA0->CHANNEL[dmach].XFERCFG = s_dma_descriptor_table[0].xfercfg;
    DMA0->COMMON[0].ENABLESET = 1<<dmach;
}



void dam_init(DMACH_enum dmach, void *SADDR, void *DADDR, uint16 count);
void dam_init_linked(DMACH_enum dmach, void *SADDR, void *DADDR, uint32 count);



#endif



