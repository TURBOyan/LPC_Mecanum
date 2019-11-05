#ifndef _MPU9250_CONFIG_H_
#define _MPU9250_CONFIG_H_

#include "headfile.h"
#include "common.h"

#define MPU9250
//#define USE_DMP
#define USE_MPL

#define delay_us systick_delay_us
#define delay_ms systick_delay_ms 

#define MY_SCL_GPIO   MPU6050_SCL_PIN 	//MPU9250��SCL��
#define MY_SDA_GPIO   MPU6050_SDA_PIN		//MPU9250��SDA��

#define PIN_Init(x)   gpio_init(x, GPO, 1,PULLUP)	//��Ҫ����������Ϊ���ģʽ���Ҵ���������

//IO��������
#define Config_SDA_IN()  		gpio_dir(MY_SDA_GPIO, GPI)	
#define Config_SDA_OUT() 		gpio_dir(MY_SDA_GPIO, GPO) 
//IO��������	 
#define Config_IIC_SCL(x)    gpio_set(MY_SCL_GPIO, x) //SCL
#define Config_IIC_SDA(x)    gpio_set(MY_SDA_GPIO, x) //SDA	 
#define Config_READ_SDA   	 gpio_get(MY_SDA_GPIO)  //����SDA 

#define u8  unsigned char 
#define u16 unsigned short int

#endif
