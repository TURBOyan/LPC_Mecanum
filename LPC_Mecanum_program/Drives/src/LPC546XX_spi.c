/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		SPI
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
#include "LPC546XX_flexcomm.h" 
#include "LPC546XX_spi.h"


SPI_Type *SPIN[] = SPI_BASE_PTRS;



void spi_mux(SPIN_enum spi_n, SPI_PIN_enum sck_pin, SPI_PIN_enum mosi_pin, SPI_PIN_enum miso_pin, SPI_PIN_enum cs_pin)
{

    switch(spi_n)
    {
        case SPI_0:
        {
            if      (SPI0_SCK_A28 == sck_pin)       iocon_init(A28,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_SCK_B4 == sck_pin)        iocon_init(B4, ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI0_MOSI_A24 == mosi_pin)     iocon_init(A24,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_MOSI_A29 == mosi_pin)     iocon_init(B29,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_MOSI_B5  == mosi_pin)     iocon_init(B5, ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI0_MISO_A25 == miso_pin)     iocon_init(A25,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI0_MISO_A30 == miso_pin)     iocon_init(B30,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI0_MISO_B6  == miso_pin)     iocon_init(B6, ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI0_CS0_B8 == cs_pin)         iocon_init(B8 ,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_CS0_A31 == cs_pin)        iocon_init(A31,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_CS1_B0  == cs_pin)        iocon_init(B0, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI0_CS1_B7  == cs_pin)        iocon_init(B7, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_1:
        {
            if      (SPI1_SCK_A7 == sck_pin)        iocon_init(A7, ALT4 | DIGITAL | FILTEROFF);
            else if (SPI1_SCK_B9 == sck_pin)        iocon_init(B9, ALT2 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI1_MOSI_B10 == mosi_pin)     iocon_init(B10, ALT2 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI1_MISO_A10 == miso_pin)     iocon_init(A10,ALT4 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI1_MISO_B11 == miso_pin)     iocon_init(B11,ALT2 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI1_CS0_A13 == cs_pin)        iocon_init(A13,ALT1 | DIGITAL | FILTEROFF | GPIOMODE | I2CFILTEROFF | HIGH_DRIVE);
            else if (SPI1_CS1_A14 == cs_pin)        iocon_init(A14,ALT1 | DIGITAL | FILTEROFF | GPIOMODE | I2CFILTEROFF | HIGH_DRIVE);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_2:
        {
            if      (SPI2_SCK_B23 == sck_pin)       iocon_init(B23,ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI2_MOSI_A26 == mosi_pin)     iocon_init(A26,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI2_MOSI_B24 == mosi_pin)     iocon_init(B24,ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI2_MISO_A27 == miso_pin)     iocon_init(A27,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI2_MISO_B25 == miso_pin)     iocon_init(B25,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI2_CS0_B26 == cs_pin)        iocon_init(B26,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI2_CS1_B27 == cs_pin)        iocon_init(B27,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_3:
        {
            if      (SPI3_SCK_A0 == sck_pin)        iocon_init(A0, ALT2 | DIGITAL | FILTEROFF);
            else if (SPI3_SCK_A6 == sck_pin)        iocon_init(A6, ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI3_MOSI_A3 == mosi_pin)      iocon_init(A3, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_MOSI_B1 == mosi_pin)      iocon_init(B1, ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI3_MISO_A2 == miso_pin)      iocon_init(A2, ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI3_MISO_A12 == miso_pin)     iocon_init(A12,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI3_CS0_A1 == cs_pin)         iocon_init(A1 ,ALT2 | DIGITAL | FILTEROFF);
            else if (SPI3_CS0_A20 == cs_pin)        iocon_init(A20,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_CS1_A7  == cs_pin)        iocon_init(A7, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_CS1_A21  == cs_pin)       iocon_init(A21,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_CS2_A9  == cs_pin)        iocon_init(A9, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_CS2_B23  == cs_pin)       iocon_init(B23,ALT5 | DIGITAL | FILTEROFF);
            else if (SPI3_CS3_A8  == cs_pin)        iocon_init(A8, ALT1 | DIGITAL | FILTEROFF);
            else if (SPI3_CS3_B24  == cs_pin)       iocon_init(B24,ALT5 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_4:
        {
            if      (SPI4_SCK_A4 == sck_pin)        iocon_init(A4, ALT2 | DIGITAL | FILTEROFF);
            else if (SPI4_SCK_B19 == sck_pin)       iocon_init(B19,ALT5 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI4_MOSI_A5 == mosi_pin)      iocon_init(A5, ALT2 | DIGITAL | FILTEROFF);
            else if (SPI4_MOSI_B21 == mosi_pin)     iocon_init(B21,ALT5 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI4_MISO_A16 == miso_pin)     iocon_init(A16,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI4_MISO_B20 == miso_pin)     iocon_init(B20,ALT5 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI4_CS0_A18 == cs_pin)        iocon_init(A18,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI4_CS0_B9 == cs_pin)         iocon_init(B9, ALT5 | DIGITAL | FILTEROFF);
            else if (SPI4_CS1_A19  == cs_pin)       iocon_init(A19,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI4_CS1_B15  == cs_pin)       iocon_init(B15,ALT5 | DIGITAL | FILTEROFF);
            else if (SPI4_CS2_A17  == cs_pin)       iocon_init(A17,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI4_CS2_B8  == cs_pin)        iocon_init(B8, ALT5 | DIGITAL | FILTEROFF);
            else if (SPI4_CS3_B22  == cs_pin)       iocon_init(B22,ALT5 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_5:
        {
            if      (SPI5_SCK_A7 == sck_pin)        iocon_init(A7, ALT3 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI5_MOSI_A8 == mosi_pin)      iocon_init(A8, ALT3 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI5_MISO_A9 == miso_pin)      iocon_init(A9, ALT3 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI5_CS0_B14 == cs_pin)        iocon_init(B14,ALT4 | DIGITAL | FILTEROFF);
            else if (SPI5_CS1_B15 == cs_pin)        iocon_init(B15,ALT4 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_6:
        {
            if      (SPI6_SCK_A10 == sck_pin)       iocon_init(A10,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI6_SCK_B12 == sck_pin)       iocon_init(B12,ALT2 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI6_MOSI_A11 == mosi_pin)     iocon_init(A11,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI6_MOSI_B13 == mosi_pin)     iocon_init(B13,ALT2 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI6_MISO_A22 == miso_pin)     iocon_init(A22,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI6_MISO_B16 == miso_pin)     iocon_init(B16,ALT2 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI6_CS0_A15 == cs_pin)        iocon_init(A15,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_7:
        {
            if      (SPI7_SCK_A21 == sck_pin)       iocon_init(A21,ALT7 | DIGITAL | FILTEROFF);
            else if (SPI7_SCK_B28 == sck_pin)       iocon_init(B28,ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI7_MOSI_A20 == mosi_pin)     iocon_init(A20,ALT7 | DIGITAL | FILTEROFF);
            else if (SPI7_MOSI_B29 == mosi_pin)     iocon_init(B29,ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI7_MISO_A19 == miso_pin)     iocon_init(A19,ALT7 | DIGITAL | FILTEROFF | PULLUP);
            else if (SPI7_MISO_B30 == miso_pin)     iocon_init(B30,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI7_CS0_B21 == cs_pin)        iocon_init(B21,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI7_CS1_B20 == cs_pin)        iocon_init(B20,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
        case SPI_8:
        {
            if      (SPI8_SCK_B19 == sck_pin)       iocon_init(B19,ALT1 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI8_MOSI_B17 == mosi_pin)     iocon_init(B17,ALT2 | DIGITAL | FILTEROFF);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI8_MISO_B18 == miso_pin)     iocon_init(B18,ALT2 | DIGITAL | FILTEROFF | PULLUP);
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
            
            if      (SPI8_CS0_B31 == cs_pin)        iocon_init(B31,ALT5 | DIGITAL | FILTEROFF);
            else if (SPI8_CS1_B22 == cs_pin)        iocon_init(B22,ALT1 | DIGITAL | FILTEROFF);
            else if (SPI_CS_NOT_USE == cs_pin)      ;
            else                                    ASSERT(0);//���Ŵ��� �������ʧ��
        }break;
        
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      SPI��ʼ��
//  @param      spi_n           ѡ��SPIģ��(SPI_0-SPI_8)
//  @param      cs_pin          ѡ��SPIƬѡ����
//  @param      sck_pin         ѡ��SPIʱ������
//  @param      mosi_pin        ѡ��SPI MOSI����
//  @param      miso_pin        ѡ��SPI MISO����
//  @param      mode            SPIģʽ 0��CPOL=0 CPHA=0    1��CPOL=0 CPHA=1   2��CPOL=1 CPHA=0   3��CPOL=1 CPHA=1 //����ϸ�ڿ����аٶ�
//  @param      baud            ����SPI�Ĳ�����
//  @return     void
//  Sample usage:               spi_init(SPI_1, SPI1_CS0_A13, SPI1_SCK_A7, SPI1_MOSI_A11, SPI1_MISO_A10, 1*1000*1000);//Ӳ��SPI��ʼ��     
//  @note                       ��ͬʱʹ��USART��IIC��SPIͨѶ�˿ڵ�ʱ����Ҫ�ر�ע�⣬ͬһ��ģ��ŵ�ͨѶ�˿ڲ���ͬʱʹ��
//                              ��USART_0��IIC_0��SPI_0ģ��Ŷ�Ϊ0����ͬʱʹ�ã����ʹ����USART_0ģ�飬��IIC_0��SPI_0ģ�鶼����ʹ��
//                              ͬ��ģ���Ϊ1��2��3��4��5��6��7��8�������                                                                                                                                         
//-------------------------------------------------------------------------------------------------------------------
uint32 spi_init(SPIN_enum spi_n, SPI_PIN_enum cs_pin, SPI_PIN_enum sck_pin, SPI_PIN_enum mosi_pin, SPI_PIN_enum miso_pin, uint8 mode, uint32 baud)
{
    uint32 divval;
    uint32 temp_cfg = 0;
    
    flexcomm_clk_enable((FLEXCOMMN_enum)spi_n,SPI);      //����flexcommʱ��
    
    spi_mux(spi_n, sck_pin, mosi_pin, miso_pin, cs_pin);
    
    if(1 == mode || 3 == mode)  temp_cfg |= SPI_CFG_CPHA_MASK;//����SPI CPHA
    if(2 == mode || 3 == mode)  temp_cfg |= SPI_CFG_CPOL_MASK;//����SPI CPOL

    SPIN[spi_n]->CFG = temp_cfg | SPI_CFG_MASTER_MASK;        //����ģʽ

    divval = (flexcomm_get_clk((FLEXCOMMN_enum)spi_n)*10/baud + 5)/10 - 1;
    if(SPI_DIV_DIVVAL_MASK < divval)                ASSERT(0);  //Ƶ�ʹ�С �������ʧ��
    SPIN[spi_n]->DIV = divval;                                  //����SPI������  ��������
    SPIN[spi_n]->DLY = ( 0
                      | SPI_DLY_PRE_DELAY(0)            //
                      | SPI_DLY_POST_DELAY(0)           //
                      | SPI_DLY_FRAME_DELAY(0)          //
                      | SPI_DLY_TRANSFER_DELAY(0)       //
                      );
    
    SPIN[spi_n]->FIFOCFG = ( 0
                          | SPI_FIFOCFG_ENABLETX_MASK   //ʹ��TX FIFO
                          | SPI_FIFOCFG_ENABLERX_MASK   //ʹ��RX FIFO
                          | SPI_FIFOCFG_EMPTYTX_MASK    //���TX FIFO 
                          | SPI_FIFOCFG_EMPTYRX_MASK    //���TX FIFO
                          );
    
    SPIN[spi_n]->FIFOSTAT = SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;

    SPIN[spi_n]->CFG |= SPI_CFG_ENABLE_MASK;            //ʹ��SPI
    
    return (flexcomm_get_clk((FLEXCOMMN_enum)spi_n)/(divval+1));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      SPI���ͽ��պ���
//  @param      spi_n           ѡ��SPIģ��   (SPI_0-SPI_8)
//  @param      cs_pin          ѡ��SPIƬѡ����
//  @param      modata          ���͵����ݻ�������ַ
//  @param      midata          ��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
//  @param      len             ���͵��ֽ���
//  @param      eot             �Ƿ�Ϊ����ͨ�ŵ����һ���ֽ�
//  @return     void				
//  @since      v2.0
//  Sample usage:       		spi_mosi(SPI_1,SPI1_CS0_A13,buf,buf,1);    //����buff�����ݣ������յ�buf�����Ϊ1�ֽ�
//-------------------------------------------------------------------------------------------------------------------
void spi_mosi(SPIN_enum spi_n, SPI_PIN_enum cs_pin, uint8 *modata, uint8 *midata, uint32 len, uint8 eot)
{
    uint32 temp_cs;
    uint32 temp_fifowr;
    
    ASSERT(0 != len);               //�����ֽ�����Ϊ0
    
    if(SPI_CS_NOT_USE == cs_pin)    //��ʹ��CS����
    {
        temp_fifowr = 0;
    }
    
    else                            //ʹ��Ƭѡ����
    {
        temp_cs = cs_pin%21/3-3;
        ASSERT(4 > temp_cs);//����  Ƭѡ�����Ƿ���ȷ
        temp_fifowr = ((~(1U << temp_cs)&0xf) << SPI_FIFOWR_TXSSEL0_N_SHIFT);
    }

    while(--len)
    {
        while(!(SPIN[spi_n]->FIFOSTAT & SPI_FIFOSTAT_TXEMPTY_MASK));                    //�ȵ����ͻ���Ϊ��
        SPIN[spi_n]->FIFOWR = SPI_FIFOWR_TXDATA(*modata) | temp_fifowr | SPI_FIFOWR_LEN(7);  //��������   
        while(!(SPIN[spi_n]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK));                 //�ȴ����ջ��岻Ϊ��
        if(NULL != midata)  *midata = SPI_FIFORD_RXDATA_MASK & SPIN[spi_n]->FIFORD;     //��������
        else                SPIN[spi_n]->FIFORD;

        modata++;
        if(NULL != midata)  midata++;
    }
    
    if(eot && (SPI_CS_NOT_USE != cs_pin)) temp_fifowr |= SPI_FIFOWR_EOT_MASK;
    
    while(!(SPIN[spi_n]->FIFOSTAT & SPI_FIFOSTAT_TXEMPTY_MASK));                        //�ȵ����ͻ���Ϊ��
    SPIN[spi_n]->FIFOWR = SPI_FIFOWR_TXDATA(*modata) | temp_fifowr | SPI_FIFOWR_LEN(7); //��������   
    while(!(SPIN[spi_n]->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK));                     //�ȴ����ջ��岻Ϊ��
    if(NULL != midata)  *midata = SPI_FIFORD_RXDATA_MASK & SPIN[spi_n]->FIFORD;         //��������
    else                SPIN[spi_n]->FIFORD;
     
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      SPI���ͽ��պ���
//  @param      spi_n            ѡ��SPIģ��  (SPI_0-SPI_8) 
//  @param      cs_pin          ѡ��SPIƬѡ����
//  @param      mocmd           ���͵����������ַ
//  @param      micmd           ��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
//  @param      modata          ���͵����ݻ�������ַ
//  @param      midata          ��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
//  @param      cmdlen          ����������ֽ���
//  @param      len             �������ݵ��ֽ���
//  @return     void				
//  @since      v2.0
//  Sample usage:       		spi_mosi_cmd(SPI_1,SPI1_CS0_A13,cmd,NULL,buff,buff,1,2);    //����cmd/buff�����ݣ�������cmd����ʱ�����ݣ�����buff����ʱ�����ݵ�buff����ȷֱ�Ϊ1��2�ֽ�
//-------------------------------------------------------------------------------------------------------------------
void spi_mosi_cmd(SPIN_enum spi_n, SPI_PIN_enum cs_pin, uint8 *mocmd , uint8 *micmd , uint8 *modata , uint8 *midata, uint32 cmdlen , uint32 len)
{
    ASSERT(0 != cmdlen);            //�����ֽ�����Ϊ0
    ASSERT(0 != len);               //�����ֽ�����Ϊ0
    spi_mosi(spi_n,cs_pin,mocmd,micmd,cmdlen,0);
    spi_mosi(spi_n,cs_pin,modata,midata,len,1);
}
