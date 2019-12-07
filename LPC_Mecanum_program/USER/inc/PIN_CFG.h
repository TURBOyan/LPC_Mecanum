#ifndef _PIN_CFG_H_
#define _PIN_CFG_H_

//#define old_PCB
#define new_PCB

//----LED引脚----	 

#ifdef old_PCB
	#define  OLED_SCL_PIN	A14			//老板
	#define  OLED_SDA_PIN	A13
	#define  OLED_RST_PIN	B13
	#define  OLED_DC_PIN	B27
	#define  OLED_CS_PIN	B9
#endif

#ifdef new_PCB
	#define  OLED_SCL_PIN	B21			//新版
	#define  OLED_SDA_PIN	B22
	#define  OLED_RST_PIN	B13
	#define  OLED_DC_PIN	B27
	#define  OLED_CS_PIN	B9
#endif

//----MPU6050模拟IIC引脚-----
#define MPU6050_SCL_PIN     A9
#define MPU6050_SDA_PIN     A8

//----按键----
#ifdef old_PCB
	#define Button_Up    B26		//老板
	#define Button_Down  B3
	#define Button_Left  B2
	#define Button_Right B12
	#define Button_Mid   A7
#endif

#ifdef new_PCB
	#define Button_Up    B3
	#define Button_Down  A7
	#define Button_Left  B5
	#define Button_Right B26
	#define Button_Mid   B12
#endif

//----拨码开关----
#ifdef old_PCB
	#define Switch_1 B10	//无		//老板
	#define Switch_2 B23	//无
	#define Switch_3 B17	//无
	#define Switch_4 B1	  //无
#endif

#ifdef new_PCB
	#define Switch_1 B10	//无
	#define Switch_2 B23	//无
	#define Switch_3 B2	//无
	#define Switch_4 B1	  //无
#endif

//----蜂鸣器----
#define Beep   B9
#define Beep_Init   gpio_init(Beep,GPO,0,PULLUP)
#define Beep_On		  gpio_set(Beep,1)
#define Beep_Off		gpio_set(Beep,0)

//舵机
#ifdef old_PCB
	#define Servo B5				//引脚定义				//老板
	#define Servo_Init ctimer_pwm_init(TIMER2_PWMCH0_B5, 330,3500)	//舵机初始化
	#define Servo_Up   ctimer_pwm_duty(TIMER2_PWMCH0_B5, 2800)		//舵机拿起
	#define Servo_Down ctimer_pwm_duty(TIMER2_PWMCH0_B5, 6000)		//舵机放下
#endif

#ifdef new_PCB
	#define Servo A26				//引脚定义
	#define Servo_Init 	 sct_pwm_init(SCT0_OUT5_A26, 330, 2800);	//舵机初始化
	#define Servo_Up   	 sct_pwm_duty(SCT0_OUT5_A26, 2800)		//舵机拿起
	#define Servo_Down   sct_pwm_duty(SCT0_OUT5_A26, 5000)		//舵机放下

#endif

//电磁铁
#define Elema_Mid    A29		//引脚定义
#define Elema_Front  A30
#define Elema_Right  B4
#define Elema_Init(x)     gpio_init(x,GPO,0,PULLUP)	//电磁铁初始化
#define Elema_Absorb(x)   gpio_set(x,1)			//电磁铁吸起
#define Elema_Unabsorb(x) gpio_set(x,0)			//电磁铁放下

#define Elema_Mid_Sensor B0
#define Elema_Mid_Sensor_Init  gpio_init(Elema_Mid_Sensor,GPI,0,PULLUP)	//电磁铁初始化
#define Elema_Mid_Sensor_Read  gpio_get(Elema_Mid_Sensor)==0


////串口
//#define Bluetooth_UART     USART_0
//#define Bluetooth_UART_TXD UART0_TX_A25
//#define Bluetooth_UART_RXD UART0_RX_A24

#endif
