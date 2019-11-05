/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		EEPROM
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#ifndef _LPC546XX_eeprom_h
#define _LPC546XX_eeprom_h


#define EEPROM_BASE_ADDRESS         (0x40108000)                    //EEPROM ����ַ
#define EEPROM_SIZE                 (0x00004000)                    //EEPROM ��С
#define EEPROM_PAGE_COUNT           (128)                           //EEPROM ҳ��
#define EEPROM_PAGE_SIZE            (EEPROM_SIZE/EEPROM_PAGE_COUNT) //EEPROM ҳ��С
#define EEPROM_OFFSET_MAX           (EEPROM_SIZE/4)            		//EEPROM ƫ�����ֵ
#define EEPROM_PROGRAM_CMD          (6)                             //EEPROM ����
#define EEPROM_INTERNAL_FREQ        (1500000)                       //EEPROM Ƶ��



//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM��ȡһ����(4���ֽ�)
//  @param      type        д��EEPROM��ҳ һҳ��д32��uint32��������
//  @param      offset      ��Ҫд������ݵĵ�ַ 
//  @return     ��ȡ������
//  Sample usage:            uint32 test = EEPROM_READ_WORD(uint32,0);  ��ȡƫ��0  ����Ϊuint32
//  @note                    
//-------------------------------------------------------------------------------------------------------------------
#define EEPROM_READ_WORD(type,offset)    (*(type *)(EEPROM_BASE_ADDRESS + ((offset<EEPROM_OFFSET_MAX?offset:0)*4)))
//#define my_EEPROM_READ_WORD(Page_Num,offset,type)    (*(type *)(EEPROM_BASE_ADDRESS + ((offset<EEPROM_OFFSET_MAX?offset:0)*4)+Page_Num*EEPROM_PAGE_SIZE))
#define my_EEPROM_READ_WORD(Page_Num,offset,type)     (*(type *)((uint32)((EEPROM_BASE_ADDRESS + ((offset<EEPROM_OFFSET_MAX?offset:0)*4)+Page_Num*EEPROM_PAGE_SIZE))))
void eeprom_init(void);
void my_eeprom_write_word(uint16 Page_num,int16 offset, uint32 data);
void eeprom_write_page(uint16 pagenum, uint32 *data);
void my_eeprom_write_word(uint16 Page_num,int16 offset, uint32 data);


#endif

