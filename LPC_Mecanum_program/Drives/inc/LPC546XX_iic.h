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

#ifndef _LPC546XX_iic_h
#define _LPC546XX_iic_h

//��ö�ٶ��岻�����û��޸�
typedef enum //ö�ٴ��ں�
{
    IIC0_SDA_A24 = 0*10+0*5, IIC0_SDA_A29, IIC0_SDA_A31, IIC0_SDA_B5, IIC0_SDA_B8, 
    IIC0_SCL_A25 = 0*10+1*5, IIC0_SCL_A30, IIC0_SCL_B0,  IIC0_SCL_B6, IIC0_SCL_B7,  

    IIC1_SDA_A13 = 1*10+0*5, IIC1_SDA_B10,
    IIC1_SCL_A10 = 1*10+1*5, IIC1_SCL_A14, IIC1_SCL_B11, 
    
    IIC2_SDA_A26 = 2*10+0*5, IIC2_SDA_B24, IIC2_SDA_B26,
    IIC2_SCL_A27 = 2*10+1*5, IIC2_SCL_B25, IIC2_SCL_B27,
    
    IIC3_SDA_A1  = 3*10+0*5, IIC3_SDA_A3,  IIC3_SDA_A20, IIC3_SDA_B1,
    IIC3_SCL_A2  = 3*10+1*5, IIC3_SCL_A7,  IIC3_SCL_A12, IIC3_SCL_A21, 
    
    IIC4_SDA_A5  = 4*10+0*5, IIC4_SDA_A18, IIC4_SDA_B9,  IIC4_SDA_B21, 
    IIC4_SCL_A16 = 4*10+1*5, IIC4_SCL_A19, IIC4_SCL_B15, IIC4_SCL_B20, 
    
    IIC5_SDA_A8  = 5*10+0*5, IIC5_SDA_B14,
    IIC5_SCL_A9  = 5*10+1*5, IIC5_SCL_B15,
    
    IIC6_SDA_A11 = 6*10+0*5, IIC6_SDA_A15, IIC6_SDA_B13, 
    IIC6_SCL_A22 = 6*10+1*5, IIC6_SCL_B16,
    
    IIC7_SDA_A20 = 7*10+0*5, IIC7_SDA_B21, IIC7_SDA_B29,
    IIC7_SCL_A19 = 7*10+1*5, IIC7_SCL_B20, IIC7_SCL_B30,
    
    IIC8_SDA_B17 = 8*10+0*5, IIC8_SDA_B31,
    IIC8_SCL_B18 = 8*10+1*5, IIC8_SCL_B22,

}IIC_PIN_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum //ö�ٴ��ں�
{
    IIC_0,
    IIC_1,
    IIC_2,
    IIC_3,
    IIC_4,
    IIC_5,
    IIC_6,
    IIC_7,
    IIC_8,
    IIC_MAX,
}IICN_enum;

//��ö�ٶ��岻�����û��޸�
typedef enum
{
    MWSR =   0x00,  // ����дģʽ  
    MRSW =   0x01   // ������ģʽ  
} MSMODEN_enum;

uint32 iic_init(IICN_enum iic_n, IIC_PIN_enum sda_pin, IIC_PIN_enum scl_pin, uint32 baud);
void   iic_write_reg(IICN_enum iic_n, uint8 slaveid, uint8 reg, uint8 data);
uint8  iic_read_reg(IICN_enum iic_n, uint8 slaveid, uint8 reg);
uint8  iic_read_reg_bytes(IICN_enum iic_n, uint8 slaveid, uint8 reg, uint8 * addr, uint8 num);

#endif
