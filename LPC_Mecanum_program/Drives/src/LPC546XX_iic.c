/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		IIC
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/





#include "common.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_flexcomm.h" 
#include "LPC546XX_iic.h"



I2C_Type *IICN[] = I2C_BASE_PTRS;




#define IIC_STRAT(iic_n)            {IICN[iic_n]->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;}    //���������ź�

#define IIC_STOP(iic_n)             {IICN[iic_n]->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;}     //����ֹͣ�ź�

#define IIC_CONTINUE(iic_n)         {IICN[iic_n]->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;} //���������ź�

#define DELAY_MAX                   10000//�ȴ����ʱ��    400K��ʱ��3000������ͨѶ  ������Խ��Ӧ���ø���

#define WAIT_TIME(iic_n,TIME)       {   TIME++;\
                                        if(TIME>DELAY_MAX){\
                                            IICN[iic_n]->CFG &= ~I2C_CFG_MSTEN_MASK; \
                                            IICN[iic_n]->CFG = I2C_CFG_MSTEN_MASK;\
                                            IIC_STOP(iic_n);\
                                            break;\
                                        }\
                                    }//�����ʱû�м�⵽��־λ��������

#define IIC_WAIT_IDLE(iic_n,TIME)   {TIME = 0;while(0X0 != (I2C_STAT_MSTSTATE_MASK & IICN[iic_n]->STAT) >> I2C_STAT_MSTSTATE_SHIFT) WAIT_TIME(iic_n,TIME);}    //�ȴ�IICΪ����

#define IIC_WAIT_RX(iic_n,TIME)     {TIME = 0;while(0X1 != (I2C_STAT_MSTSTATE_MASK & IICN[iic_n]->STAT) >> I2C_STAT_MSTSTATE_SHIFT) WAIT_TIME(iic_n,TIME);}    //�ȴ��������

#define IIC_WAIT_TX(iic_n,TIME)     {TIME = 0;while(0X2 != (I2C_STAT_MSTSTATE_MASK & IICN[iic_n]->STAT) >> I2C_STAT_MSTSTATE_SHIFT) WAIT_TIME(iic_n,TIME);}    //�ȴ��������

#define IIC_WAIT_PENDING(iic_n,TIME){TIME = 0;while(!(IICN[iic_n]->STAT & I2C_STAT_MSTPENDING_MASK)) WAIT_TIME(iic_n,TIME);}






void iic_mux(IICN_enum iic_n, IIC_PIN_enum sda_pin, IIC_PIN_enum scl_pin)
{
    switch(iic_n)
    {
        case IIC_0:
        {
            if     (IIC0_SDA_A24 == sda_pin)    iocon_init(A24,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SDA_A29 == sda_pin)    iocon_init(A29,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SDA_A31 == sda_pin)    iocon_init(A31,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SDA_B5  == sda_pin)    iocon_init(B5 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SDA_B8  == sda_pin)    iocon_init(B8 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC0_SCL_A25 == scl_pin)    iocon_init(A25,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SCL_A30 == scl_pin)    iocon_init(A30,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SCL_B0  == scl_pin)    iocon_init(A0 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SCL_B6  == scl_pin)    iocon_init(A6 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC0_SCL_B7  == scl_pin)    iocon_init(A7 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_1:
        {
            if     (IIC1_SDA_A13 == sda_pin)    iocon_init(A13,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if(IIC1_SDA_B10 == sda_pin)    iocon_init(B10,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
            if     (IIC1_SCL_A10 == scl_pin)    iocon_init(A10,ALT4 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC1_SCL_A14 == scl_pin)    iocon_init(A14,ALT1 | DIGITAL | FILTEROFF | PULLUP);
            else if(IIC1_SCL_B11 == scl_pin)    iocon_init(B11,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_2:
        {
            if     (IIC2_SDA_A26 == sda_pin)    iocon_init(A26,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC2_SDA_B24 == sda_pin)    iocon_init(B24,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC2_SDA_B26 == sda_pin)    iocon_init(B26,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC2_SCL_A27 == scl_pin)    iocon_init(A27,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC2_SCL_B25 == scl_pin)    iocon_init(B25,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC2_SCL_B27 == scl_pin)    iocon_init(B27,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_3:
        {
            if     (IIC3_SDA_A1  == sda_pin)    iocon_init(A1 ,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SDA_A3  == sda_pin)    iocon_init(A3 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SDA_A20 == sda_pin)    iocon_init(A20,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SDA_B1  == sda_pin)    iocon_init(B1 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC3_SCL_A2  == scl_pin)    iocon_init(A2 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SCL_A7  == scl_pin)    iocon_init(A7 ,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SCL_A12 == scl_pin)    iocon_init(A12,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC3_SCL_A21 == scl_pin)    iocon_init(A21,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_4:
        {
            if     (IIC4_SDA_A5  == sda_pin)    iocon_init(A5 ,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SDA_A18 == sda_pin)    iocon_init(A18,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SDA_B9  == sda_pin)    iocon_init(B9 ,ALT5 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SDA_B21 == sda_pin)    iocon_init(B21,ALT5 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC4_SCL_A16 == scl_pin)    iocon_init(A16,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SCL_A19 == scl_pin)    iocon_init(A19,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SCL_B15 == scl_pin)    iocon_init(B15,ALT5 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC4_SCL_B20 == scl_pin)    iocon_init(B20,ALT5 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_5:
        {
            if     (IIC5_SDA_A8  == sda_pin)    iocon_init(A8 ,ALT3 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC5_SDA_B14 == sda_pin)    iocon_init(B14,ALT4 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC5_SCL_A9  == scl_pin)    iocon_init(A9 ,ALT3 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC5_SCL_B15 == scl_pin)    iocon_init(B15,ALT4 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_6:
        {
            if     (IIC6_SDA_A11 == sda_pin)    iocon_init(A11,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC6_SDA_A15 == sda_pin)    iocon_init(A15,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC6_SDA_B13 == sda_pin)    iocon_init(B13,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC6_SCL_A22 == scl_pin)    iocon_init(A22,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC6_SCL_B16 == scl_pin)    iocon_init(B16,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_7:
        {
            if     (IIC7_SDA_A20 == sda_pin)    iocon_init(A20,ALT7 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC7_SDA_B21 == sda_pin)    iocon_init(B21,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC7_SDA_B29 == sda_pin)    iocon_init(B29,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC7_SCL_A19 == scl_pin)    iocon_init(A19,ALT7 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC7_SCL_B20 == scl_pin)    iocon_init(B20,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC7_SCL_B30 == scl_pin)    iocon_init(B30,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
        
        case IIC_8:
        {
            if     (IIC8_SDA_B17 == sda_pin)    iocon_init(B17,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC8_SDA_B31 == sda_pin)    iocon_init(B31,ALT5 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��                  
                                                                                                   
            if     (IIC8_SCL_B18 == scl_pin)    iocon_init(B18,ALT2 | DIGITAL | FILTEROFF | PULLUP | OD);
            else if(IIC8_SCL_B22 == scl_pin)    iocon_init(B22,ALT1 | DIGITAL | FILTEROFF | PULLUP | OD);
            else                                ASSERT(0);//���Ŵ��� �������ʧ��
            
        }break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      IIC��ʼ��
//  @param      iic_n           ѡ��IICģ��
//  @param      sda_pin         ѡ��IIC��������
//  @param      scl_pin         ѡ��IICʱ������
//  @param      baud            ����IIC�Ĳ�����
//  @return     void
//  Sample usage:               iic_init(IIC_2, IIC2_SDA_A26, IIC2_SCL_A27,400*1000);//Ӳ��IIC��ʼ��     
//  @note                       ��ͬʱʹ��USART��IIC��SPIͨѶ�˿ڵ�ʱ����Ҫ�ر�ע�⣬ͬһ��ģ��ŵ�ͨѶ�˿ڲ���ͬʱʹ��
//                              ��USART_0��IIC_0��SPI_0ģ��Ŷ�Ϊ0����ͬʱʹ�ã����ʹ����USART_0ģ�飬��IIC_0��SPI_0ģ�鶼����ʹ��
//                              ͬ��ģ���Ϊ1��2��3��4��5��6��7��8�������     
//-------------------------------------------------------------------------------------------------------------------
uint32 iic_init(IICN_enum iic_n, IIC_PIN_enum sda_pin, IIC_PIN_enum scl_pin, uint32 baud)
{
    uint32 divval;
    
    flexcomm_clk_enable((FLEXCOMMN_enum)iic_n,IIC);     //����flexcommʱ��
    
    iic_mux(iic_n, sda_pin, scl_pin);
    

    divval = (flexcomm_get_clk((FLEXCOMMN_enum)iic_n)*10 / (baud*4) + 5) / 10 - 1;
    if(I2C_CLKDIV_DIVVAL_MASK < divval)         ASSERT(0);//Ƶ�ʹ�С �������ʧ��
    IICN[iic_n]->CLKDIV = divval;                       //���÷�Ƶֵ
    IICN[iic_n]->MSTTIME = 0X00;                        //����SCL�ߵ͵�ƽʱ��
    
    IICN[iic_n]->CFG = I2C_CFG_MSTEN_MASK;              //��������ģʽ ��׼�ٶȺͿ���ģʽ
    
    return (flexcomm_get_clk((FLEXCOMMN_enum)iic_n)/4/(divval+1));
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      д��һ���ֽ����ݵ�I2C�豸ָ���Ĵ�����ַ
//  @param      iic_n       IICģ��
//  @param      slaveid     �ӻ���ַ(7λ��ַ)
//  @param      reg         �ӻ��Ĵ�����ַ
//  @param      data        ����
//  @return     void
//  @since      v2.0
//  Sample usage:       	iic_write_reg(IIC_2, 0x2D, 0x50,2);     //д������2��0x50��ַ���ӻ���ַΪ0x2D
//-------------------------------------------------------------------------------------------------------------------
void iic_write_reg(IICN_enum iic_n, uint8 slaveid, uint8 reg, uint8 data)
{
    int time = 0;
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    IICN[iic_n]->MSTDAT = (slaveid << 1) | MWSR;
    IIC_STRAT(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IICN[iic_n]->MSTDAT = reg;
    IIC_CONTINUE(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IICN[iic_n]->MSTDAT = data;
    IIC_CONTINUE(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    
    IIC_STOP(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡI2C�豸ָ����ַ�Ĵ���������
//  @param      iic_n       IICģ��
//  @param      slaveid     �ӻ���ַ(7λ��ַ)
//  @param      reg         �ӻ��Ĵ�����ַ
//  @return                 ��ȡ�ļĴ���ֵ
//  @since      v2.0
//  Sample usage:       	uint8 value = iic_read_reg(IIC_2, 0x2D, 0x50);//��ȡ0x50��ַ�����ݣ��ӻ���ַΪ0x2D
//-------------------------------------------------------------------------------------------------------------------
uint8 iic_read_reg(IICN_enum iic_n, uint8 slaveid, uint8 reg)
{
    uint8 result;
    int time = 0;
    //��ַ�ǵ���λ
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    IICN[iic_n]->MSTDAT = (slaveid << 1) | MWSR;    //���ʹӻ���ַ��дλ
    IIC_STRAT(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IICN[iic_n]->MSTDAT = reg;                      //���ͼĴ�����ַ
    IIC_CONTINUE(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IIC_STOP(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    IICN[iic_n]->MSTDAT = (slaveid << 1) | MRSW;    //���ʹӻ���ַ�Ͷ�λ
    IIC_STRAT(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_RX(iic_n,time);                        //�ȴ��������
    result = IICN[iic_n]->MSTDAT;                   //��ȡ����        
    IIC_STOP(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡI2C�豸ָ����ַ�Ĵ���������
//  @param      iic_n       IICģ��(iic0��iic1)
//  @param      slaveid     �ӻ���ַ(7λ��ַ)
//  @param      reg         �ӻ��Ĵ�����ַ
//  @param      addr        ��ȡ�����ݴ洢�ĵ�ַ
//  @param      num         ��ȡ�ֽ���
//  @return     void
//  @since      v2.0
//  Sample usage:       	uint8 value = iic_read_reg(IIC_2, 0x2D, 0x50, 10, buf);//��ȡ0x50��ַ�����ݣ��ӻ���ַΪ0x2D��ʼ��10���ֽ�
//-------------------------------------------------------------------------------------------------------------------
uint8 iic_read_reg_bytes(IICN_enum iic_n, uint8 slaveid, uint8 reg, uint8 * addr, uint8 num)
{
    int time = 0;
    //��ַ�ǵ���λ
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    IICN[iic_n]->MSTDAT = (slaveid << 1) | MWSR;
    IIC_STRAT(iic_n);
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IICN[iic_n]->MSTDAT = reg;
    IIC_CONTINUE(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_TX(iic_n,time);
    IIC_STOP(iic_n);

    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    IICN[iic_n]->MSTDAT = (slaveid << 1) | MRSW;
    IIC_STRAT(iic_n);
    
    
    while(--num)
    {
        IIC_WAIT_PENDING(iic_n,time);
        IIC_WAIT_RX(iic_n,time);
        *addr = IICN[iic_n]->MSTDAT;                //��ȡ����     
        IIC_CONTINUE(iic_n);
        addr++;
    }
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_RX(iic_n,time);                             //�ȴ��������
    *addr = IICN[iic_n]->MSTDAT;                    //��ȡ����        
    
    IIC_STOP(iic_n);
    
    
    IIC_WAIT_PENDING(iic_n,time);
    IIC_WAIT_IDLE(iic_n,time);
    return 0;
}
