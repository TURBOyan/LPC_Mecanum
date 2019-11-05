/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴common.h��VERSION�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					���߶��壺
					------------------------------------ 
						SCL                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
						SDA                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
					------------------------------------ 
 ********************************************************************************************************************/

#include "common.h"
#include "LPC546XX_systick.h"
#include "LPC546XX_gpio.h"
#include "LPC546XX_iic.h"
#include "LPC546XX_iocon.h"  
#include "LPC546XX_spi.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_ICM20602.h"


int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self1_check(void)
{
    while(0x12 != simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,SIMIIC)) //��ȡICM20602 ID
    {
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
    systick_delay_ms(10);  //�ϵ���ʱ
    
    //���
    icm20602_self1_check();
    
    //��λ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);               //��λ�豸
    systick_delay_ms(2);                                                        //��ʱ
    while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,SIMIIC));//�ȴ���λ���
    
    //���ò���
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //ʱ������
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //���������Ǻͼ��ٶȼ�
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //��2000 dps
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);             //��8g
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x23);           //Average 8 samples   44.8HZ
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata(void)
{
    uint8 dat[6];
    
    simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, dat, 6, SIMIIC);  
    icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro(void)
{
    uint8 dat[6];

    simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, dat, 6, SIMIIC);  
    icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  ���Ϻ�����ʹ�����IICͨ�ţ���Ƚ�Ӳ��IIC�����IIC���Ÿ���������ʹ��������ͨIO
//-------------------------------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------------------------------
//  ���º�����ʹ��Ӳ��IICͨ�ţ���Ƚ����IIC��Ӳ��IIC�ٶȿ����������졣
//-------------------------------------------------------------------------------------------------------------------

#define IIC_NUM         IIC_5
#define IIC_SDA_PIN     IIC5_SDA_A8
#define IIC_SCL_PIN     IIC5_SCL_A9

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self2_check(void)
{
    while(0x12 != iic_read_reg(IIC_NUM, ICM20602_DEV_ADDR, ICM20602_WHO_AM_I))   //��ȡICM20602 ID
    {
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_hardware(void)
{
    systick_delay_ms(10);  //�ϵ���ʱ
    iic_init(IIC_NUM, IIC_SDA_PIN, IIC_SCL_PIN,400*1000);       //Ӳ��IIC��ʼ��     ������400K
    //���
    icm20602_self2_check();
    
    //��λ
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_PWR_MGMT_1, 0x80);	        //�������״̬
    systick_delay_ms(2);                                                            //��ʱ
    while(0x80 & iic_read_reg(IIC_NUM, ICM20602_DEV_ADDR, ICM20602_PWR_MGMT_1));    //�ȴ���λ���
    
    //���ò���
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_PWR_MGMT_1, 0x01);	        //ʱ������
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_PWR_MGMT_2, 0x00);            //���������Ǻͼ��ٶȼ�
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_CONFIG, 0x01);                //176HZ 1KHZ
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_SMPLRT_DIV, 0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_GYRO_CONFIG, 0x18);           //��2000 dps
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_ACCEL_CONFIG, 0x10);          //��8g
    iic_write_reg(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_ACCEL_CONFIG_2, 0x23);        //Average 8 samples   44.8HZ
          
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_hardware(void)
{
    uint8 dat[6];
    
    iic_read_reg_bytes(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, dat, 6);
    icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_hardware(void)
{
    uint8 dat[6];

    iic_read_reg_bytes(IIC_NUM,ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, dat, 6);  
    icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}






//-------------------------------------------------------------------------------------------------------------------
//  ���º�����ʹ��Ӳ��SPIͨ�ţ���Ƚ�IIC���ٶȱ�IIC��ǳ��ࡣ
//-------------------------------------------------------------------------------------------------------------------
#define SPI_NUM         SPI_4           
#define SPI_SCK_PIN     SPI4_SCK_B19    //��ģ��SPC
#define SPI_MOSI_PIN    SPI4_MOSI_B21   //��ģ��SDI
#define SPI_MISO_PIN    SPI4_MISO_B20   //��ģ��SDO
#define SPI_CS_PIN      SPI4_CS0_B9     //��ģ��CS

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_w_reg_byte(uint8 cmd, uint8 val)
{
    cmd |= ICM20602_SPI_W;
    spi_mosi_cmd(SPI_NUM,SPI_CS_PIN,&cmd,NULL,&val,NULL,1,1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_byte(uint8 cmd, uint8 *val)
{
    cmd |= ICM20602_SPI_R;
    spi_mosi_cmd(SPI_NUM,SPI_CS_PIN,&cmd,NULL,val,val,1,1);
}
  
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���ֽڶ��Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @param      num     ��ȡ����
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_bytes(uint8 cmd, uint8 * val, uint8 num)
{
    cmd |= ICM20602_SPI_R;
    spi_mosi_cmd(SPI_NUM,SPI_CS_PIN,&cmd,NULL,val,val,1,num);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(void)
{
    uint8 val;
    do
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&val);
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }while(0x12 != val);

}
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
    uint8 val = 0x0;

    systick_delay_ms(10);  //�ϵ���ʱ
    
    (void)spi_init(SPI_NUM, SPI_CS_PIN, SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, 3, 10*1000*1000);//Ӳ��SPI��ʼ��

    icm20602_self3_check();//���
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
    systick_delay_ms(2);
    do
    {//�ȴ���λ�ɹ�
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
    }while(0x41 != val);
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //ʱ������
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //���������Ǻͼ��ٶȼ�
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //��2000 dps
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);            //��8g
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x23);            //Average 8 samples   44.8HZ  
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
    uint8 dat[6];
    
    icm_spi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
    uint8 dat[6];
    
    icm_spi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6);
    icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}










