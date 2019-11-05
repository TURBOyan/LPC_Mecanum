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

#include "common.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_eeprom.h"


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM��ʼ��
//  @param      void        
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void eeprom_init(void)
{
    uint32_t clockdiv = 0;
    
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_EEPROM_MASK;
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_EEPROM_RST_MASK;
    

    clockdiv = main_clk_mhz*1000000 / EEPROM_INTERNAL_FREQ;
    //��������
    if ((main_clk_mhz*1000000 % EEPROM_INTERNAL_FREQ) > (EEPROM_INTERNAL_FREQ>>1)) clockdiv += 1U;

    EEPROM->CLKDIV = clockdiv - 1;
    
    EEPROM->AUTOPROG = 1;
    //������ʱ����
    EEPROM->RWSTATE = EEPROM_RWSTATE_RPHASE1(14) | EEPROM_RWSTATE_RPHASE2(7);
    
    EEPROM->WSTATE = ( 0 
                     | EEPROM_WSTATE_PHASE1(4)
                     | EEPROM_WSTATE_PHASE2(8)
                     | EEPROM_WSTATE_PHASE3(2)
                     //| EEPROM_WSTATE_LCK_PARWEP_MASK
                      );
    EEPROM->CMD = EEPROM_PROGRAM_CMD;
    while(!(EEPROM->INTSTAT & EEPROM_INTSTAT_END_OF_PROG_MASK));

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROMд��һ����(4���ֽ�)
//  @param      offset       д��EEPROM�ĵ�ַƫ����
//  @param      *data        ��Ҫд�����ݵ�ַ
//  @return     void
//  Sample usage:            eeprom_write_word(0,12345); ����12345д�뵽ƫ����Ϊ0��EEPROM����
//-------------------------------------------------------------------------------------------------------------------
void my_eeprom_write_word(uint16 Page_num,int16 offset, uint32 data)
{
    uint32 *addr;

    if (offset >= EEPROM_OFFSET_MAX)    ASSERT(0);
    if(EEPROM->AUTOPROG)    EEPROM->AUTOPROG = 1;           //�����Զ����
    EEPROM->INTSTATCLR = EEPROM_INTENSET_PROG_SET_EN_MASK;  //�����־λ
    addr = (uint32 *)(EEPROM_BASE_ADDRESS + Page_num * EEPROM_PAGE_SIZE+offset*4);//�����ַ
    *addr = data;                                           //д������
    if(0x01 != EEPROM->AUTOPROG)                            //�����Ƿ���Ҫ�ֶ����
    {
        EEPROM->CMD = EEPROM_PROGRAM_CMD;
    }
    while(!(EEPROM->INTSTAT & EEPROM_INTSTAT_END_OF_PROG_MASK));//�ȴ���ɲ���
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROMд��һ����(4���ֽ�)
//  @param      pagenum      д��EEPROM��ҳ һҳ��д32��uint32��������
//  @param      *data        ��Ҫд������ݵĵ�ַ 
//  @return     void
//  Sample usage:            uint32 dat[32];  eeprom_write_page(0,dat); ��dat���������ȫ��д��EEPROM�ĵ�0ҳ
//  @note                    �������� ����Ԫ�����Ϊ32�� �����������Խ�����
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_page(uint16 pagenum, uint32 *data)
{
    uint8 i;
    uint32 *addr;

    if (pagenum >= EEPROM_PAGE_COUNT || !data)    ASSERT(0);

    if(EEPROM->AUTOPROG)    EEPROM->AUTOPROG = 2;           //�����Զ����
    EEPROM->INTSTATCLR = EEPROM_INTENSET_PROG_SET_EN_MASK;  //�����־λ
    addr = (uint32 *)(EEPROM_BASE_ADDRESS + pagenum * EEPROM_PAGE_SIZE);//�����ַ
    for(i=0;i<EEPROM_PAGE_SIZE/4;i++)
    {
        addr[i] = data[i];                                  //д������
    }

    if(0x00 == EEPROM->AUTOPROG)                            //�����Ƿ���Ҫ�ֶ����
    {
        EEPROM->CMD = EEPROM_PROGRAM_CMD;   
    }
    while(!(EEPROM->INTSTAT & EEPROM_INTSTAT_END_OF_PROG_MASK));//�ȴ���ɲ���
}






