#ifndef _PIN_CFG_H_
#define _PIN_CFG_H_

//#define old_PCB
#define new_PCB

//----LED����----	 

#ifdef old_PCB
	#define  OLED_SCL_PIN	A14			//�ϰ�
	#define  OLED_SDA_PIN	A13
	#define  OLED_RST_PIN	B13
	#define  OLED_DC_PIN	B27
	#define  OLED_CS_PIN	B9
#endif

#ifdef new_PCB
	#define  OLED_SCL_PIN	B21			//�°�
	#define  OLED_SDA_PIN	B22
	#define  OLED_RST_PIN	B13
	#define  OLED_DC_PIN	B27
	#define  OLED_CS_PIN	B9
#endif

//----MPU6050ģ��IIC����-----
#define MPU6050_SCL_PIN     A9
#define MPU6050_SDA_PIN     A8

//----����----
#ifdef old_PCB
	#define Button_Up    B26		//�ϰ�
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

//----���뿪��----
#ifdef old_PCB
	#define Switch_1 B10	//��		//�ϰ�
	#define Switch_2 B23	//��
	#define Switch_3 B17	//��
	#define Switch_4 B1	  //��
#endif

#ifdef new_PCB
	#define Switch_1 B10	//��
	#define Switch_2 B23	//��
	#define Switch_3 B2	//��
	#define Switch_4 B1	  //��
#endif

//----������----
#define Beep   B9
#define Beep_Init   gpio_init(Beep,GPO,0,PULLUP)
#define Beep_On		  gpio_set(Beep,1)
#define Beep_Off		gpio_set(Beep,0)

//���
#ifdef old_PCB
	#define Servo B5				//���Ŷ���				//�ϰ�
	#define Servo_Init ctimer_pwm_init(TIMER2_PWMCH0_B5, 330,3500)	//�����ʼ��
	#define Servo_Up   ctimer_pwm_duty(TIMER2_PWMCH0_B5, 2800)		//�������
	#define Servo_Down ctimer_pwm_duty(TIMER2_PWMCH0_B5, 6000)		//�������
#endif

#ifdef new_PCB
	#define Servo A26				//���Ŷ���
	#define Servo_Init 	 sct_pwm_init(SCT0_OUT5_A26, 330, 2800);	//�����ʼ��
	#define Servo_Up   	 sct_pwm_duty(SCT0_OUT5_A26, 2800)		//�������
	#define Servo_Down   sct_pwm_duty(SCT0_OUT5_A26, 5000)		//�������

#endif

//�����
#define Elema_Mid    A29		//���Ŷ���
#define Elema_Front  A30
#define Elema_Right  B4
#define Elema_Init(x)     gpio_init(x,GPO,0,PULLUP)	//�������ʼ��
#define Elema_Absorb(x)   gpio_set(x,1)			//���������
#define Elema_Unabsorb(x) gpio_set(x,0)			//���������

#define Elema_Mid_Sensor B0
#define Elema_Mid_Sensor_Init  gpio_init(Elema_Mid_Sensor,GPI,0,PULLUP)	//�������ʼ��
#define Elema_Mid_Sensor_Read  gpio_get(Elema_Mid_Sensor)==0


////����
//#define Bluetooth_UART     USART_0
//#define Bluetooth_UART_TXD UART0_TX_A25
//#define Bluetooth_UART_RXD UART0_RX_A24

#endif
