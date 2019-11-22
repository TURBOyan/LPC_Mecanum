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

void PIN_INT1_DriverIRQHandler(void)															
{
		PINT_IST_FLAG_CLEAR(PINT_CH1);
		disable_irq(PIN_INT1_IRQn);		//��ʱ�ر��ж�
		pit_init_ms(10);		//����10ms��ʱ����ֹ���ж�ִ��ʱ�䳬��10ms
		set_irq_priority(RIT_IRQn,0);
		enable_irq(RIT_IRQn);
	
/*********************���Ϸ��Լ��Ŀ��ƴ���******************************************************************************/

		Read_ButtSwitData();			//��ȡ����ֵ
		Refresh_MPUTeam(DMP_MPL); //��ȡ��̬��
		Read_GrayData(95,2,1);		//��ȡ����ֵ
		Gray_Calibration();				//���ܾ�ȷ����
		MPU_Yaw_Closeloop();  		//ƫ���Ǳջ�����
	
		Wheel_Analysis();					//Ŀ���ٶȼ���
    Motor_PWM_Set(9999);			//PWM��ֵ
	
/*********************���Ϸ��Լ��Ŀ��ƴ���******************************************************************************/	
	
		DataSend(0);//��λ�����ݷ��ͣ�1Ϊ������0Ϊ�ر�
    pit_clean();						//�����ʱ����ʱ
		disable_irq(RIT_IRQn);	//�رռ�ʱ
		pit_deinit();						//����ʼ��
		enable_irq(PIN_INT1_IRQn);	//���������жϣ�׼�������´�MPL����
}

void RIT_DriverIRQHandler(void)
{
    PIT_FLAG_CLEAR;
		LED_Fill(0x00);
	
		while(1)
		{
			LED_P6x8Str(20, 3, "MPU-Interrupt");			//MPU9250���жϷ�����ִ��ʱ�䲻�ܳ���10ms�����򽫳�ʱ����
			LED_P6x8Str(10, 4, "function TIMEOUT!!!!");
		}
}

void FLEXCOMM0_DriverIRQHandler(void)
{
    vuint32 flag;
		uint8 Data;
    flag = UART0_FIFO_FLAG;
		static uint8 mode=1;
    uart_getchar(USART_0,&Data);
    if(flag & USART_FIFOINTSTAT_RXLVL_MASK)//����FIFO�ﵽ�趨ˮƽ����Ĭ���趨ˮƽ ������FIFO��һ�����ݵ�ʱ�򴥷��жϣ�
    {
			
			if(mode)
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X_Real=0;		//ǰ��
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All; break;
					
					case 'H': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All/1.4;		//��ǰ
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All/1.4; break;
					
					case 'G': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All;	//��ת
										MECANUM_Motor_Data.Speed_Y_Real=0;   break;
					
					case 'F': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All/1.4;	//���
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'E': MECANUM_Motor_Data.Speed_X_Real=0;		//����
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All;break;

					case 'D': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All/1.4;	//�Һ�
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'C': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All;		//��ת
										MECANUM_Motor_Data.Speed_Y_Real=0;   break;

					case 'B': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All/1.4;	//��ǰ
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All/1.4;   break;
										
					case 'Z': MECANUM_Motor_Data.Speed_X_Real=0;		//ֹͣ
										MECANUM_Motor_Data.Speed_Y_Real=0;
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;					break;
										
					case 'X': MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=90; break;		//��ת90��
										
					case 'Y': MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=-90;	break;//��ת90��
										
					case 'p':	Elema_Unabsorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;break;//������
					case 'o': Elema_Absorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;	break;//������
					case 'n': Elema_Unabsorb(Elema_Left);	break;//�����ϰ�
					case 'm': Elema_Unabsorb(Elema_Right);	break;//�����ϰ�
					
					case 'K': mode=0;
										MPU_Data.Yaw_CloseLoop_Flag=0; //�ر�ƫ���Ǳջ�
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;
										break;
					default : break;
				}
			}
			else
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X_Real=0;		//ǰ��
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All; break;
					
					case 'E': MECANUM_Motor_Data.Speed_X_Real=0;		//����
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All;break;

					case 'G': MECANUM_Motor_Data.Speed_GyroZ_Set=-100;	//˳ʱ��
										break;
				
					case 'C': MECANUM_Motor_Data.Speed_GyroZ_Set=100;			//��ʱ��
										break;
					
					case 'Z': MECANUM_Motor_Data.Speed_X_Real=0;		//ֹͣ
										MECANUM_Motor_Data.Speed_Y_Real=0;
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;			
										break;
										
					case 'X': MECANUM_Motor_Data.Speed_All=2000; break;		//����
										
					case 'Y': MECANUM_Motor_Data.Speed_All=500;break;//����
					
					case 'J': mode=1;
										MPU_Data.Yaw_CloseLoop_Flag=1; //����ƫ���Ǳջ�
										MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=0;
										break;
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



