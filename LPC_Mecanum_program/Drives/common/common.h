/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		common
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/
 
 
 
#ifndef _common_h
#define _common_h



//������������
typedef unsigned char       uint8;   //  8 bits 
typedef unsigned short int  uint16;  // 16 bits 
typedef unsigned long int   uint32;  // 32 bits 
typedef unsigned long long  uint64;  // 64 bits 

typedef char                int8;    //  8 bits 
typedef short int           int16;   // 16 bits 
typedef long  int           int32;   // 32 bits 
typedef long  long          int64;   // 64 bits 

typedef volatile int8       vint8;   //  8 bits 
typedef volatile int16      vint16;  // 16 bits 
typedef volatile int32      vint32;  // 32 bits 
typedef volatile int64      vint64;  // 64 bits 

typedef volatile uint8      vuint8;  //  8 bits 
typedef volatile uint16     vuint16; // 16 bits 
typedef volatile uint32     vuint32; // 32 bits 
typedef volatile uint64     vuint64; // 64 bits 

typedef enum // ö�ٶ˿ڱ�� 
{
    A  = 0,
    B  = 1,
    
    //���������ֲ� ������ʽ
    P0 = 0,
    P1 = 1,
    //A��P0�ȼ�  B��P1�ȼ�
}PORTN_enum;



typedef enum // ö�ٶ˿ڱ�� 
{
    //���������ֲ��������ʽ�����Ѻã����ϵͳ�彫������ʽ����Ϊ��K60����
    //����ʹ�ú��İ��ʱ���������
    //���İ�A0�����������ֲ��P0�˿�0�����ţ���P00���ţ��Ƕ�Ӧ�ģ�A��ʾP0�˿ڣ�A��������ݱ�ʾ���ű��
    A0=0,   A1,  A2,  A3,  A4,  A5,  A6,  A7,
    A8,     A9,  A10, A11, A12, A13, A14, A15,
    A16,    A17, A18, A19, A20, A21, A22, A23,
    A24,    A25, A26, A27, A28, A29, A30, A31,
            
    B0,     B1,  B2,  B3,  B4,  B5,  B6,  B7,
    B8,     B9,  B10, B11, B12, B13, B14, B15,
    B16,    B17, B18, B19, B20, B21, B22, B23,
    B24,    B25, B26, B27, B28, B29, B30, B31,
    
    
    //���������ֲ� ������ʽ
    P00=0,  P01,  P02,  P03,  P04,  P05,  P06,  P07,
    P08,    P09,  P010, P011, P012, P013, P014, P015,
    P016,   P017, P018, P019, P020, P021, P022, P023,
    P024,   P025, P026, P027, P028, P029, P030, P031,
            
    P10,    P11,  P12,  P13,  P14,  P15,  P16,  P17,
    P18,    P19,  P110, P111, P112, P113, P114, P115,
    P116,   P117, P118, P119, P120, P121, P122, P123,
    P124,   P125, P126, P127, P128, P129, P130, P131,

    //A0��P00����ͬһ������     ������������
}PIN_enum;



typedef enum //ö�ٶ˿ڷ���
{
    GPI = 0, //����ܽ����뷽��      
    GPO = 1, //����ܽ��������
}GPIODIR_enum;

typedef enum //ö�ٶ˿ڵ�ƽ
{
    GPIO_LOW = 0,  //����ܽ����뷽��      
    GPIO_HIGH = 1, //����ܽ��������
}GPIOLEVEL_enum;

typedef enum    // ö�ٴ�����ʽ
{
    LOW,
    HIGH,
    RISING,
    FALLING,
    BOTH,       //��PINTģ�� ���ô˴�����ʽ
}TRIGGER_enum;




// Compiler Related Definitions 
#ifdef __CC_ARM                         // ARM Compiler 
    #define ALIGN(n)                    __attribute__((aligned(n)))
#elif defined (__IAR_SYSTEMS_ICC__)     // for IAR Compiler 
    #define PRAGMA(x)                   _Pragma(#x)
    #define ALIGN(n)                    PRAGMA(data_alignment=n)
#elif defined (__GNUC__)                // GNU GCC Compiler 
    #define ALIGN(n)                    __attribute__((aligned(n)))
#endif // Compiler Related Definitions 



/*
 * ȷ��x�ķ�ΧΪ min~max
 */
#define RANGE(x,max,min)        (((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))



#include <math.h>
#include <string.h>

#include "SEEKFREE_PRINTF.h"
#include "LPC546XX_config.h"
#include "LPC54606.h"

#include "misc.h"

#include "assert.h"




#endif
