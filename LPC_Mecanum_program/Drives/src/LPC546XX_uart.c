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


     

#include "common.h"
#include "LPC546XX_iocon.h"
#include "SEEKFREE_FUN.h"
#include "LPC546XX_flexcomm.h" 
#include "LPC546XX_uart.h"
     

USART_Type    * UARTN[] = USART_BASE_PTRS;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������Ÿ��ü�������Ӧʱ��
//  @param      uartn       ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      tx_pin      ���ڷ�������
//  @param      rx_pin      ���ڽ�������
//  @return     void
//  Sample usage:           �����û�����
//-------------------------------------------------------------------------------------------------------------------
void uart_mux(UARTN_enum uartn, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin)
{
    switch(uartn)
    {
        case USART_0:
        {
            if     (UART0_TX_A25 == tx_pin)   iocon_init(A25,ALT1 | DIGITAL);
            else if(UART0_TX_A30 == tx_pin)   iocon_init(A30,ALT1 | DIGITAL);
            else if(UART0_TX_B6  == tx_pin)   iocon_init(B6, ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART0_RX_A24 == rx_pin)   iocon_init(A24,ALT1 | DIGITAL);
            else if(UART0_RX_A29 == rx_pin)   iocon_init(A29,ALT1 | DIGITAL);
            else if(UART0_RX_B5  == rx_pin)   iocon_init(B5, ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
   
        }break;
        
        case USART_1:
        {
            if     (UART1_TX_A10 == tx_pin)   iocon_init(A10,ALT4 | DIGITAL);
            else if(UART1_TX_B11 == tx_pin)   iocon_init(B11,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART1_RX_B10 == rx_pin)   iocon_init(B10,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_2:
        {
            if     (UART2_TX_A27 == tx_pin)   iocon_init(A27,ALT1 | DIGITAL);
            else if(UART2_TX_B25 == tx_pin)   iocon_init(B25,ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART2_RX_A26 == rx_pin)   iocon_init(A26,ALT1 | DIGITAL);
            else if(UART2_RX_B24 == rx_pin)   iocon_init(B24,ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_3:
        {
            if     (UART3_TX_A2  == tx_pin)   iocon_init(A2, ALT1 | DIGITAL);
            else if(UART3_TX_A12 == tx_pin)   iocon_init(A12,ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART3_RX_A3 == rx_pin)    iocon_init(A3, ALT1 | DIGITAL);
            else if(UART3_RX_B1 == rx_pin)    iocon_init(B1, ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_4:
        {
            if     (UART4_TX_A16 == tx_pin)   iocon_init(A16,ALT1 | DIGITAL);
            else if(UART4_TX_B20 == tx_pin)   iocon_init(B20,ALT5 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART4_RX_A5  == rx_pin)   iocon_init(A5, ALT2 | DIGITAL);
            else if(UART4_RX_B21 == rx_pin)   iocon_init(B21,ALT5 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_5:
        {
            if     (UART5_TX_A9 == tx_pin)    iocon_init(A9, ALT3 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART5_RX_A8 == rx_pin)    iocon_init(A8, ALT3 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_6:
        {
            if     (UART6_TX_A22 == tx_pin)   iocon_init(A22,ALT1 | DIGITAL);
            else if(UART6_TX_B16 == tx_pin)   iocon_init(B16,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART6_RX_A11 == rx_pin)   iocon_init(A11,ALT1 | DIGITAL);
            else if(UART6_RX_B13 == rx_pin)   iocon_init(B13,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_7:
        {
            if     (UART7_TX_A19 == tx_pin)   iocon_init(A19,ALT7 | DIGITAL);
            else if(UART7_TX_B30 == tx_pin)   iocon_init(B30,ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART7_RX_A20 == rx_pin)   iocon_init(A20,ALT7 | DIGITAL);
            else if(UART7_RX_B29 == rx_pin)   iocon_init(B29,ALT1 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case USART_8:
        {
            if     (UART8_TX_B18 == tx_pin)   iocon_init(B18,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (UART8_RX_B17 == rx_pin)   iocon_init(B17,ALT2 | DIGITAL);
            else                              ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        default:    ASSERT(0);//���ںŴ��� �������ʧ��
    }
    
    flexcomm_clk_enable((FLEXCOMMN_enum)uartn,USART);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڳ�ʼ��
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      baud            ���ڲ�����
//  @param      tx_pin          ���ڷ�������
//  @param      rx_pin          ���ڽ�������
//  @return     uint32          ʵ�ʲ�����
//  Sample usage:               uart_init(USART_0,115200,UART0_TX_A25,UART0_RX_A24);       // ��ʼ������0 ������115200 ��������ʹ��P025 ��������ʹ��P024
//  @note                       ��ͬʱʹ��USART��IIC��SPIͨѶ�˿ڵ�ʱ����Ҫ�ر�ע�⣬ͬһ��ģ��ŵ�ͨѶ�˿ڲ���ͬʱʹ��
//                              ��USART_0��IIC_0��SPI_0ģ��Ŷ�Ϊ0����ͬʱʹ�ã����ʹ����USART_0ģ�飬��IIC_0��SPI_0ģ�鶼����ʹ��
//                              ͬ��ģ���Ϊ1��2��3��4��5��6��7��8�������     
//-------------------------------------------------------------------------------------------------------------------
uint32 uart_init(UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin)
{
    uint32  baud_div;
    int32   baud_temp;
    vuint32 osr_temp;
    uint32  osr_val;
    int32   baud_diff;
    uint32  baud_diff_min = (uint32)(-1);
    uint32  baud_div_temp;
    uint32  real_baud;

                  
    uart_mux(uartn, tx_pin, rx_pin);//������Ӧʱ�Ӽ����Ÿ�������
    
    UARTN[uartn]->CFG = 0;    //�رմ���
    UARTN[uartn]->CTL = 0;
    
    
    for(osr_temp=0xf; osr_temp>=0x4; osr_temp--)
    {
        baud_div_temp = (flexcomm_get_clk((FLEXCOMMN_enum)uartn)*10/(osr_temp+1)/baud + 5)/10 - 1;
        if(USART_BRG_BRGVAL_MASK < baud_div_temp)
        {
            ASSERT(0);//����ʧ��   ��֧�ִ˲�����
        }

        baud_temp = flexcomm_get_clk((FLEXCOMMN_enum)uartn)/(osr_temp+1)/(baud_div_temp+1);
        baud_diff = myabs(baud_temp - baud);
        if(!baud_diff)//��ǰ�ҵ����ʵķ�Ƶϵ��
        {
            baud_div = baud_div_temp;
            osr_val = osr_temp;
            break;
        }
        
        if(baud_diff_min > baud_diff)
        {
            baud_diff_min = baud_diff;
            baud_div = baud_div_temp;
            osr_val = osr_temp;
        }
    }
    
    //����ʧ�ܣ�������������ʵ�ʲ����������ڰٷ�֮2.5
    //Ҳ��������Ȼ��ʹ�÷��ص���ʵ�����ʣ�Ȼ�󽫶�Ӧ��Ŀ���豸����Ϊ��ͬ�Ĳ�����
    if( ((long)baud_diff_min*1000/baud) > 25)   ASSERT(0);  
    
    
    real_baud = flexcomm_get_clk((FLEXCOMMN_enum)uartn)/(osr_val+1)/(baud_div+1);      //������ʵ�Ĳ�����
    UARTN[uartn]->OSR = osr_val;                            //���ò���
    UARTN[uartn]->BRG = USART_BRG_BRGVAL(baud_div);         //���ò����ʷ�Ƶϵ��


    UARTN[uartn]->FIFOCFG  = (0
                           | USART_FIFOCFG_ENABLETX_MASK    //����TX FIFO
                           | USART_FIFOCFG_ENABLERX_MASK    //����RX FIFO
                           //| USART_FIFOCFG_DMATX_MASK     //
                           //| USART_FIFOCFG_DMARX_MASK     //
                           | USART_FIFOCFG_EMPTYTX_MASK     //���TX FIFO
                           | USART_FIFOCFG_EMPTYRX_MASK     //���RX FIFO
                            );

    UARTN[uartn]->FIFOSTAT |= (USART_FIFOSTAT_RXERR_MASK | USART_FIFOSTAT_TXERR_MASK);
    UARTN[uartn]->FIFOTRIG = USART_FIFOTRIG_RXLVLENA_MASK;
    UARTN[uartn]->CFG = (0                                  
                       | USART_CFG_DATALEN(1)               //���ݳ���   0: 7λ  1��8λ  2:9λ
                       | USART_CFG_PARITYSEL(0)             //��żУ��   0������żУ��λ   2��żУ��  3����У��
                       | USART_CFG_ENABLE_MASK              //��������
                       //| USART_CFG_LOOP_MASK              //�Է�����   �����ⲿ���� �ڲ���ͨ
                       //| USART_CFG_STOPLEN_MASK           //ֹͣλ���� 0��1λ  1����λ   ����Ϊ0 
                         ); 
    return real_baud;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����ֽ����
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      dat             ��Ҫ���͵��ֽ�
//  @return     void        
//  Sample usage:               uart_putchar(USART_0,0xA5);       // ����0����0xA5
//-------------------------------------------------------------------------------------------------------------------
void uart_putchar(UARTN_enum uartn, uint8 dat)
{
    while(!(UARTN[uartn]->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK));
    UARTN[uartn]->FIFOWR = dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڷ�������
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      *buff           Ҫ���͵������ַ
//  @param      len             ���ͳ���
//  @return     void
//  Sample usage:               uart_putbuff(USART_0,&a[0],sizeof(a));  
//-------------------------------------------------------------------------------------------------------------------
void uart_putbuff (UARTN_enum uartn, uint8 *buff, uint32 len)
{
    while(len--)
    {
        uart_putchar(uartn, *buff);
        buff++;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڷ����ַ���
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      *str            Ҫ���͵��ַ�����ַ
//  @return     void
//  Sample usage:               uart_putstr(USART_0,"i lvoe you"); 
//-------------------------------------------------------------------------------------------------------------------
void uart_putstr (UARTN_enum uartn, const uint8 *str)
{
    while(*str)
    {
        uart_putchar(uartn, *str++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ڽ��յ����ݣ�whlie�ȴ���
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      *dat            �������ݵĵ�ַ
//  @return     void        
//  Sample usage:               uint8 dat; uart_getchar(USART_0,&dat);       // ���մ���0����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
void uart_getchar(UARTN_enum uartn, uint8 *dat)
{
    while(!(UARTN[uartn]->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK));
    *dat = UARTN[uartn]->FIFORD;        //ʹ��fifo����ʱ  Ĭ��ʹ��FIFO����
    //*dat = UARTN[uartn]->FIFORDNOPOP;   //��ʹ��fifo����ʱ
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ڽ��յ����ݣ���ѯ���գ�
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      *dat            �������ݵĵ�ַ
//  @return     uint8           1�����ճɹ�   0��δ���յ�����
//  Sample usage:               uint8 dat; uart_query(USART_0,&dat);       // ���մ���0����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query(UARTN_enum uartn, uint8 *dat)
{
    if((UARTN[uartn]->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK))
    {
        *dat = UARTN[uartn]->FIFORD;        //ʹ��fifo����ʱ  Ĭ��ʹ��FIFO����
        //*dat = UARTN[uartn]->FIFORDNOPOP;   //��ʹ��fifo����ʱ
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڷ��Ϳ����ж�����
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      status          1�����ж�   0���ر��ж�
//  @return     void        
//  Sample usage:               uart_tx_irq(USART_0,1);       // �򿪴���0���Ϳ����ж�
//-------------------------------------------------------------------------------------------------------------------
void uart_tx_irq(UARTN_enum uartn,uint8 status)
{
    if(status)  
    {
        UARTN[uartn]->FIFOTRIG &= ~(uint32)(USART_FIFOTRIG_TXLVL_MASK);
        UARTN[uartn]->FIFOTRIG |= USART_FIFOTRIG_TXLVLENA_MASK | USART_FIFOTRIG_TXLVL(0);
        UARTN[uartn]->FIFOINTENSET = USART_FIFOINTENSET_TXLVL_MASK;
        flexcomm_irq((FLEXCOMMN_enum)uartn,status);
    }
    else
    {
        UARTN[uartn]->FIFOTRIG &= ~(uint32)(USART_FIFOTRIG_TXLVLENA_MASK);
        UARTN[uartn]->FIFOINTENCLR = USART_FIFOINTENSET_TXLVL_MASK;
    }
    flexcomm_irq((FLEXCOMMN_enum)uartn,status);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ڽ����ж�����
//  @param      uartn           ����ģ���(USART_0,USART_1,USART_2,USART_3,USART_4,USART_5,USART_6,USART_7,USART_8)
//  @param      status          1�����ж�   0���ر��ж�
//  @return     void        
//  Sample usage:               uart_rx_irq(USART_0,1);       // �򿪴���0�����ж�
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_irq(UARTN_enum uartn,uint8 status)
{
    if(status)  
    {
        UARTN[uartn]->FIFOTRIG &= ~(uint32)(USART_FIFOTRIG_RXLVL_MASK);
        UARTN[uartn]->FIFOTRIG |= USART_FIFOTRIG_RXLVLENA_MASK | USART_FIFOTRIG_RXLVL(0);
        UARTN[uartn]->FIFOINTENSET = USART_FIFOINTENSET_RXLVL_MASK;
    }
    else
    {
        UARTN[uartn]->FIFOTRIG &= ~(uint32)(USART_FIFOTRIG_RXLVLENA_MASK);
        UARTN[uartn]->FIFOINTENCLR = USART_FIFOINTENSET_RXLVL_MASK;
    }
    flexcomm_irq((FLEXCOMMN_enum)uartn,status);
}


//����Ҫ�ض���printf �����ڣ������Ѿ�����д��printf�ĺ���
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ض���printf ������
//  @param      ch      ��Ҫ��ӡ���ֽ�
//  @param      stream  ������
//  @note       �˺����ɱ������Դ������printf������
//-------------------------------------------------------------------------------------------------------------------
//int fputc(int ch, FILE *stream)
//{
//    uart_putchar(DEBUG_UART, (char)ch);
//    return(ch);
//}
