/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		����
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#ifndef _LPC546XX_uart_h
#define _LPC546XX_uart_h

//��ö�ٶ��岻�����û��޸�
typedef enum //ö�ٴ�������
{
    UART0_TX_A25, UART0_TX_A30, UART0_TX_B6,  //����0 �������ſ�ѡ��Χ
    UART0_RX_A24, UART0_RX_A29, UART0_RX_B5,  //����0 �������ſ�ѡ��Χ

    UART1_TX_A10, UART1_TX_B11,               //����1 �������ſ�ѡ��Χ        
    UART1_RX_B10,                             //����1 �������ſ�ѡ��Χ
                                                  
    UART2_TX_A27, UART2_TX_B25,               //����2 �������ſ�ѡ��Χ
    UART2_RX_A26, UART2_RX_B24,               //����2 �������ſ�ѡ��Χ
                        
    UART3_TX_A2,  UART3_TX_A12,               //����3 �������ſ�ѡ��Χ 
    UART3_RX_A3,  UART3_RX_B1,                //����3 �������ſ�ѡ��Χ
                        
    UART4_TX_A16, UART4_TX_B20,               //����4 �������ſ�ѡ��Χ
    UART4_RX_A5,  UART4_RX_B21,               //����4 �������ſ�ѡ��Χ
    
    UART5_TX_A9,                              //����5 �������ſ�ѡ��Χ
    UART5_RX_A8,                              //����5 �������ſ�ѡ��Χ
    
    UART6_TX_A22, UART6_TX_B16,               //����6 �������ſ�ѡ��Χ
    UART6_RX_A11, UART6_RX_B13,               //����6 �������ſ�ѡ��Χ ��A11�����ؽӿڳ�ͻ��
    
    UART7_TX_A19, UART7_TX_B30,               //����7 �������ſ�ѡ��Χ
    UART7_RX_A20, UART7_RX_B29,               //����7 �������ſ�ѡ��Χ
    
    UART8_TX_B18,                             //����8 �������ſ�ѡ��Χ
    UART8_RX_B17,                             //����8 �������ſ�ѡ��Χ

}UARTPIN_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum //ö�ٴ��ں�
{
    USART_0,
    USART_1,
    USART_2,
    USART_3,
    USART_4,
    USART_5,
    USART_6,
    USART_7,
    USART_8,
}UARTN_enum;

//-------------------------------------------------------------------------------------------------------------------
//  @brief        ����FIFO�жϱ�־λ�궨��
//  @return       ��־λ�Ĵ���ֵ  
//  λ0��         TX FIFO�����ж�     1�������ж�  0��δ�����ж�
//  λ1��         RX FIFO�����ж�     1�������ж�  0��δ�����ж�
//  λ2��         TX FIFO�ȼ��ж�     1�������ж�  0��δ�����ж�
//  λ3��         RX FIFO�ȼ��ж�     1�������ж�  0��δ�����ж�
//  λ4��         �����ж�            1�������ж�  0��δ�����ж�
//-------------------------------------------------------------------------------------------------------------------
#define UART0_FIFO_FLAG  USART0->FIFOINTSTAT;
#define UART1_FIFO_FLAG  USART1->FIFOINTSTAT;
#define UART2_FIFO_FLAG  USART2->FIFOINTSTAT;
#define UART3_FIFO_FLAG  USART3->FIFOINTSTAT;
#define UART4_FIFO_FLAG  USART4->FIFOINTSTAT;
#define UART5_FIFO_FLAG  USART5->FIFOINTSTAT;
#define UART6_FIFO_FLAG  USART6->FIFOINTSTAT;
#define UART7_FIFO_FLAG  USART7->FIFOINTSTAT;
#define UART8_FIFO_FLAG  USART8->FIFOINTSTAT;

uint32 uart_init(UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin);
void   uart_putchar(UARTN_enum uartn, uint8 dat);
void   uart_putbuff (UARTN_enum uartn, uint8 *buff, uint32 len);
void   uart_putstr (UARTN_enum uartn, const uint8 *str);
void   uart_getchar(UARTN_enum uartn, uint8 *dat);
uint8  uart_query(UARTN_enum uartn, uint8 *dat);
void   uart_tx_irq(UARTN_enum uartn,uint8 status);
void   uart_rx_irq(UARTN_enum uartn,uint8 status);

#endif
