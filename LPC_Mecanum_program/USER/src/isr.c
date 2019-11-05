/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/

#include "headfile.h"
#include "isr.h"

void RIT_DriverIRQHandler(void)
{
    PIT_FLAG_CLEAR;
		uint8 zuoqian=0,zuohou=0,youqian=0,youhou=0;

/****************************���ݻ�ȡ**********************************************************/
		Refresh_MPUTeam(DMP_MPL);	//�������ݻ�ȡ
		Wheel_Speed_Get_cm_s();	//�ٶȻ�ȡ
		Read_ButtSwitData();
		if(Query_ButtSwitData(Button_Data,Button_Down_Data))
		{
			MPU_Data.Yaw_Aid=0;
			MPU_Data.Yaw_Save=MPU_Data.Yaw;
		}
//		Read_GrayData(0);			//�������ݶ�ȡ������ӡ
/******************************************************************************************************************/

/**********************************���ݼ���**********************************************************/
//		Wheel_Speed_Real_Get();	//���ٶ�ת��ΪXY�����������˶�����
//		
//		zuoqian=Base_Data.Gray_Data[0][0]+Base_Data.Gray_Data[0][1]+Base_Data.Gray_Data[0][2]+Base_Data.Gray_Data[1][0]+Base_Data.Gray_Data[2][0];
//		zuohou =Base_Data.Gray_Data[5][0]+Base_Data.Gray_Data[5][1]+Base_Data.Gray_Data[5][2]+Base_Data.Gray_Data[4][0]+Base_Data.Gray_Data[3][0];
//		youqian=Base_Data.Gray_Data[0][3]+Base_Data.Gray_Data[0][4]+Base_Data.Gray_Data[0][5]+Base_Data.Gray_Data[1][5]+Base_Data.Gray_Data[2][5];
//		youhou =Base_Data.Gray_Data[5][3]+Base_Data.Gray_Data[5][4]+Base_Data.Gray_Data[5][5]+Base_Data.Gray_Data[4][5]+Base_Data.Gray_Data[3][5];
//		
//		if(zuoqian<zuohou && zuoqian<youqian && zuoqian<youhou)
//		{
//			MECANUM_Motor_Data.Speed_X=-15;		//��ǰ
//			MECANUM_Motor_Data.Speed_Y=15;
//		}
//		
//		if(zuohou<zuoqian && zuohou<youqian && zuohou<youhou)
//		{
//			MECANUM_Motor_Data.Speed_X=-15;		//���
//			MECANUM_Motor_Data.Speed_Y=-15;
//		}
//		if(youqian<zuoqian && youqian<zuohou && youqian<youhou)
//		{
//			MECANUM_Motor_Data.Speed_X=15;		//��ǰ
//			MECANUM_Motor_Data.Speed_Y=15;
//		}
//		
//		if(youhou<zuohou && youhou<zuoqian && youhou<youqian)
//		{
//			MECANUM_Motor_Data.Speed_X=15;		//�Һ�
//			MECANUM_Motor_Data.Speed_Y=-15;
//		}
//		
//		if(youqian == youhou && youqian==zuohou && youqian==zuoqian)
//		{
//			MECANUM_Motor_Data.Speed_X=0;		//ͣ
//			MECANUM_Motor_Data.Speed_Y=0;
//		}

//		Save_GrayData(Base_Data.Gray_Data,Base_Data.Gray_Data_Last);	//�������ݱ���

		MPU_Data.Yaw_Real=Mpu_Normalization(MPU_Data.Yaw,MPU_Data.Yaw_Save);		
		MECANUM_Motor_Data.Speed_GyroZ_Out=PID_Calcu(MPU_Data.Yaw_Aid,MPU_Data.Yaw_Real,&PID_Dir,Local);
		MECANUM_Motor_Data.Speed_GyroZ_Out=RANGE(MECANUM_Motor_Data.Speed_GyroZ_Out,1.5,-1.5);
		Wheel_Analysis();		//Ŀ���ٶȼ���
    Wheel_Speed_PID();	//�ٶȱջ�
/******************************************************************************************************************/
		
/***************************************�����������Ӳ����***********************************************************/
    Motor_PWM_Set(9999);	//PWM��ֵ
/******************************************************************************************************************/		
		
/***************************************���ݱ���***********************************************************/		

/******************************************************************************************************************/


/***************************************��ʾ***********************************************************/		
//			OLED_P6x8Int(0 , 0, MPU_Data.Yaw_Real, -4);
/******************************************************************************************************************/
}

void FLEXCOMM0_DriverIRQHandler(void)
{
    vuint32 flag;
		uint8 Data;
    flag = UART0_FIFO_FLAG;
		static uint8 mode=1;
		MECANUM_Motor_Data.Speed_All =20;
    uart_getchar(USART_0,&Data);
    if(flag & USART_FIFOINTSTAT_RXLVL_MASK)//����FIFO�ﵽ�趨ˮƽ����Ĭ���趨ˮƽ ������FIFO��һ�����ݵ�ʱ�򴥷��жϣ�
    {
			
			if(mode)
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X=0;		//ǰ��
										MECANUM_Motor_Data.Speed_Y=MECANUM_Motor_Data.Speed_All; break;
					
					case 'H': MECANUM_Motor_Data.Speed_X=-MECANUM_Motor_Data.Speed_All/1.4;		//��ǰ
										MECANUM_Motor_Data.Speed_Y=MECANUM_Motor_Data.Speed_All/1.4; break;
					
					case 'G': MECANUM_Motor_Data.Speed_X=-MECANUM_Motor_Data.Speed_All;	//��ת
										MECANUM_Motor_Data.Speed_Y=0;   break;
					
					case 'F': MECANUM_Motor_Data.Speed_X=-MECANUM_Motor_Data.Speed_All/1.4;	//���
										MECANUM_Motor_Data.Speed_Y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'E': MECANUM_Motor_Data.Speed_X=0;		//����
										MECANUM_Motor_Data.Speed_Y=-MECANUM_Motor_Data.Speed_All;break;

					case 'D': MECANUM_Motor_Data.Speed_X=MECANUM_Motor_Data.Speed_All/1.4;	//�Һ�
										MECANUM_Motor_Data.Speed_Y=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'C': MECANUM_Motor_Data.Speed_X=MECANUM_Motor_Data.Speed_All;		//��ת
										MECANUM_Motor_Data.Speed_Y=0;   break;

					case 'B': MECANUM_Motor_Data.Speed_X=MECANUM_Motor_Data.Speed_All/1.4;	//��ǰ
										MECANUM_Motor_Data.Speed_Y=MECANUM_Motor_Data.Speed_All/1.4;   break;
										
					case 'Z': MECANUM_Motor_Data.Speed_X=0;		//ֹͣ
										MECANUM_Motor_Data.Speed_Y=0;
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;					break;
										
					case 'X': MPU_Data.Yaw_Save=MPU_Data.Yaw;	
										MPU_Data.Yaw_Aid=90; break;		//��ת90��
										
					case 'Y': MPU_Data.Yaw_Save=MPU_Data.Yaw;	
										MPU_Data.Yaw_Aid=-90;	break;//��ת90��
										
					case 'p':	Elema_Unabsorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;break;//������
					case 'o': Elema_Absorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;	break;//������
					case 'n': Elema_Unabsorb(Elema_Left);	break;//�����ϰ�
					case 'm': Elema_Unabsorb(Elema_Right);	break;//�����ϰ�
					
					case 'K': mode=0;break;
					default : break;
				}
			}
			else
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X=0;		//ǰ��
										MECANUM_Motor_Data.Speed_Y=MECANUM_Motor_Data.Speed_All; break;
					
					case 'E': MECANUM_Motor_Data.Speed_X=0;		//����
										MECANUM_Motor_Data.Speed_Y=-MECANUM_Motor_Data.Speed_All;break;

					case 'G': MECANUM_Motor_Data.Speed_GyroZ_Out=-MECANUM_Motor_Data.Speed_All;	//˳ʱ��
										break;
				
					case 'C': MECANUM_Motor_Data.Speed_GyroZ_Out=MECANUM_Motor_Data.Speed_All;			//��ʱ��
										break;
					
					case 'Z': MECANUM_Motor_Data.Speed_X=0;		//ֹͣ
										MECANUM_Motor_Data.Speed_Y=0;
										MECANUM_Motor_Data.Speed_GyroZ_Out=0;					break;
										
					case 'X': MECANUM_Motor_Data.Speed_All=1.4; break;		//����
										
					case 'Y': MECANUM_Motor_Data.Speed_All=0.6;break;//����
					
					case 'J': mode=1;break;
					default : break;
				}
			}
			LED_P6x8Char(0,0,Data);
    }
    if(flag & USART_FIFOINTSTAT_RXERR_MASK)//����FIFO����
    {
        USART0->FIFOCFG  |= USART_FIFOCFG_EMPTYRX_MASK;//���RX FIFO
        USART0->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
    }
}


/************************���²����޸�************************/
void PIN_INT7_DriverIRQHandler(void)
{
    mt9v032_vsync();        //����糡�жϴ��룬��ʹ��������ʱ��ִ�иô���
}

void DMA0_DriverIRQHandler(void)
{
    if(READ_DMA_FLAG(MT9V032_DMA_CH))
    {
        CLEAR_DMA_FLAG(MT9V032_DMA_CH);
        mt9v032_dma();      //�����dma�жϴ��룬��ʹ��������ʱ��ִ�иô���
    }
}



void FLEXCOMM5_DriverIRQHandler(void)
{
    vuint32 flag;
    flag = UART5_FIFO_FLAG;
    
    
    if(flag & USART_FIFOINTSTAT_RXLVL_MASK)//����FIFO�ﵽ�趨ˮƽ����Ĭ���趨ˮƽ ������FIFO��һ�����ݵ�ʱ�򴥷��жϣ�
    {
        mt9v032_cof_uart_interrupt();
    }
    
    if(flag & USART_FIFOINTSTAT_RXERR_MASK)//����FIFO����
    {
        USART5->FIFOCFG  |= USART_FIFOCFG_EMPTYRX_MASK;//���RX FIFO
        USART5->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
    }
}

uint32 tt;
void MRT0_DriverIRQHandler(void)
{
    if(MRT_FLAG_READ(MRT_CH0))
    {
        MRT_FLAG_CLR(MRT_CH0);
   
    }
    
    if(MRT_FLAG_READ(MRT_CH1))
    {
        MRT_FLAG_CLR(MRT_CH1);
        tt++;
    }
    
    if(MRT_FLAG_READ(MRT_CH2))
    {
        MRT_FLAG_CLR(MRT_CH2);
        
    }
    
    if(MRT_FLAG_READ(MRT_CH3))
    {
        MRT_FLAG_CLR(MRT_CH3);
        
    }
}

/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ�� ͨ��0���ж�
void RIT_DriverIRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ

WDT_BOD_DriverIRQHandler
DMA0_DriverIRQHandler
GINT0_DriverIRQHandler
GINT1_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
UTICK0_DriverIRQHandler
MRT0_DriverIRQHandler
CTIMER0_DriverIRQHandler
CTIMER1_DriverIRQHandler
SCT0_DriverIRQHandler
CTIMER3_DriverIRQHandler
FLEXCOMM0_DriverIRQHandler
FLEXCOMM1_DriverIRQHandler
FLEXCOMM2_DriverIRQHandler
FLEXCOMM3_DriverIRQHandler
FLEXCOMM4_DriverIRQHandler
FLEXCOMM5_DriverIRQHandler
FLEXCOMM6_DriverIRQHandler
FLEXCOMM7_DriverIRQHandler
ADC0_SEQA_DriverIRQHandler
ADC0_SEQB_DriverIRQHandler
ADC0_THCMP_DriverIRQHandler
DMIC0_DriverIRQHandler
HWVAD0_DriverIRQHandler
USB0_NEEDCLK_DriverIRQHandler
USB0_DriverIRQHandler
RTC_DriverIRQHandler
Reserved46_DriverIRQHandler
Reserved47_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER4_DriverIRQHandler
RIT_DriverIRQHandler
SPIFI0_DriverIRQHandler
FLEXCOMM8_DriverIRQHandler
FLEXCOMM9_DriverIRQHandler
SDIO_DriverIRQHandler
CAN0_IRQ0_DriverIRQHandler
CAN0_IRQ1_DriverIRQHandler
CAN1_IRQ0_DriverIRQHandler
CAN1_IRQ1_DriverIRQHandler
USB1_DriverIRQHandler
USB1_NEEDCLK_DriverIRQHandler
ETHERNET_DriverIRQHandler
ETHERNET_PMT_DriverIRQHandler
ETHERNET_MACLP_DriverIRQHandler
EEPROM_DriverIRQHandler
LCD_DriverIRQHandler
SHA_DriverIRQHandler
SMARTCARD0_DriverIRQHandler
SMARTCARD1_DriverIRQHandler
*/



