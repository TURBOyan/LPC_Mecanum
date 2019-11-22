/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看LPC546XX_config.h文件内版本宏定义
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
		disable_irq(PIN_INT1_IRQn);		//暂时关闭中断
		pit_init_ms(10);		//开启10ms计时，防止本中断执行时间超过10ms
		set_irq_priority(RIT_IRQn,0);
		enable_irq(RIT_IRQn);
	
/*********************以上放自己的控制代码******************************************************************************/

		Read_ButtSwitData();			//读取按键值
		Refresh_MPUTeam(DMP_MPL); //读取三态角
		Read_GrayData(95,2,1);		//读取光电管值
		Gray_Calibration();				//光电管精确控制
		MPU_Yaw_Closeloop();  		//偏航角闭环控制
	
		Wheel_Analysis();					//目标速度计算
    Motor_PWM_Set(9999);			//PWM赋值
	
/*********************以上放自己的控制代码******************************************************************************/	
	
		DataSend(0);//上位机数据发送，1为开启，0为关闭
    pit_clean();						//清除定时器计时
		disable_irq(RIT_IRQn);	//关闭计时
		pit_deinit();						//反初始化
		enable_irq(PIN_INT1_IRQn);	//开启引脚中断，准备接受下次MPL数据
}

void RIT_DriverIRQHandler(void)
{
    PIT_FLAG_CLEAR;
		LED_Fill(0x00);
	
		while(1)
		{
			LED_P6x8Str(20, 3, "MPU-Interrupt");			//MPU9250的中断服务函数执行时间不能超过10ms，否则将超时警告
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
    if(flag & USART_FIFOINTSTAT_RXLVL_MASK)//接收FIFO达到设定水平（库默认设定水平 当接收FIFO有一个数据的时候触发中断）
    {
			
			if(mode)
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X_Real=0;		//前进
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All; break;
					
					case 'H': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All/1.4;		//左前
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All/1.4; break;
					
					case 'G': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All;	//左转
										MECANUM_Motor_Data.Speed_Y_Real=0;   break;
					
					case 'F': MECANUM_Motor_Data.Speed_X_Real=-MECANUM_Motor_Data.Speed_All/1.4;	//左后
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'E': MECANUM_Motor_Data.Speed_X_Real=0;		//后退
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All;break;

					case 'D': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All/1.4;	//右后
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All/1.4;   break;
					
					case 'C': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All;		//右转
										MECANUM_Motor_Data.Speed_Y_Real=0;   break;

					case 'B': MECANUM_Motor_Data.Speed_X_Real=MECANUM_Motor_Data.Speed_All/1.4;	//右前
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All/1.4;   break;
										
					case 'Z': MECANUM_Motor_Data.Speed_X_Real=0;		//停止
										MECANUM_Motor_Data.Speed_Y_Real=0;
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;					break;
										
					case 'X': MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=90; break;		//右转90度
										
					case 'Y': MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=-90;	break;//左转90度
										
					case 'p':	Elema_Unabsorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;break;//放棋子
					case 'o': Elema_Absorb(Elema_Mid);Servo_Down;systick_delay_ms(2000);Servo_Up;	break;//吸棋子
					case 'n': Elema_Unabsorb(Elema_Left);	break;//放左障碍
					case 'm': Elema_Unabsorb(Elema_Right);	break;//放右障碍
					
					case 'K': mode=0;
										MPU_Data.Yaw_CloseLoop_Flag=0; //关闭偏航角闭环
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;
										break;
					default : break;
				}
			}
			else
			{
				switch(Data)
				{
					case 'A': MECANUM_Motor_Data.Speed_X_Real=0;		//前进
										MECANUM_Motor_Data.Speed_Y_Real=MECANUM_Motor_Data.Speed_All; break;
					
					case 'E': MECANUM_Motor_Data.Speed_X_Real=0;		//后退
										MECANUM_Motor_Data.Speed_Y_Real=-MECANUM_Motor_Data.Speed_All;break;

					case 'G': MECANUM_Motor_Data.Speed_GyroZ_Set=-100;	//顺时针
										break;
				
					case 'C': MECANUM_Motor_Data.Speed_GyroZ_Set=100;			//逆时针
										break;
					
					case 'Z': MECANUM_Motor_Data.Speed_X_Real=0;		//停止
										MECANUM_Motor_Data.Speed_Y_Real=0;
										MECANUM_Motor_Data.Speed_GyroZ_Set=0;			
										break;
										
					case 'X': MECANUM_Motor_Data.Speed_All=2000; break;		//高速
										
					case 'Y': MECANUM_Motor_Data.Speed_All=500;break;//低速
					
					case 'J': mode=1;
										MPU_Data.Yaw_CloseLoop_Flag=1; //开启偏航角闭环
										MPU_Data.Yaw_MapZero_Save=MPU_Data.Yaw_MapZero;	
										MPU_Data.Yaw_HeadZero_Aid=0;
										break;
					default : break;
				}
			}
			LED_P6x8Char(0,0,Data);
    }
    if(flag & USART_FIFOINTSTAT_RXERR_MASK)//接收FIFO错误
    {
        USART0->FIFOCFG  |= USART_FIFOCFG_EMPTYRX_MASK;//清空RX FIFO
        USART0->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
    }
}


/************************以下不可修改************************/
void PIN_INT7_DriverIRQHandler(void)
{
    mt9v032_vsync();        //总钻风场中断代码，当使用总钻风的时候执行该代码
}

void DMA0_DriverIRQHandler(void)
{
    if(READ_DMA_FLAG(MT9V032_DMA_CH))
    {
        CLEAR_DMA_FLAG(MT9V032_DMA_CH);
        mt9v032_dma();      //总钻风dma中断代码，当使用总钻风的时候执行该代码
    }
}



void FLEXCOMM5_DriverIRQHandler(void)
{
    vuint32 flag;
    flag = UART5_FIFO_FLAG;
    
    
    if(flag & USART_FIFOINTSTAT_RXLVL_MASK)//接收FIFO达到设定水平（库默认设定水平 当接收FIFO有一个数据的时候触发中断）
    {
        mt9v032_cof_uart_interrupt();
    }
    
    if(flag & USART_FIFOINTSTAT_RXERR_MASK)//接收FIFO错误
    {
        USART5->FIFOCFG  |= USART_FIFOCFG_EMPTYRX_MASK;//清空RX FIFO
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
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器 通道0得中断
void RIT_DriverIRQHandler(void)
{
    ;
}
记得进入中断后清除标志位

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



