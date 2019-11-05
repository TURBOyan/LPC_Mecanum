/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		pll
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#include "common.h"
#include "LPC546XX_uart.h"
#include "LPC546XX_pll.h"
     
     
#define EX_REF_CLK  12000U //(�����ⲿ�ο�ʱ��Ϊ12000KHZ)




#define NVALMAX (0x100U)
#define PVALMAX (0x20U)
#define MVALMAX (0x8000U)

#define PLL_MDEC_VAL_P (0U)                                      /*!<  MDEC is in bits  16 downto 0 */
#define PLL_MDEC_VAL_M (0x1FFFFUL << PLL_MDEC_VAL_P)             /*!<  NDEC is in bits  9 downto 0 */
#define PLL_NDEC_VAL_P (0U)                                      /*!<  NDEC is in bits  9:0 */
#define PLL_NDEC_VAL_M (0x3FFUL << PLL_NDEC_VAL_P)
#define PLL_PDEC_VAL_P (0U)                                      /*!<  PDEC is in bits 6:0 */
#define PLL_PDEC_VAL_M (0x7FUL << PLL_PDEC_VAL_P)



//���ݺ궨������main_clk_mhzĬ��ֵ
//���´��벻�ý��и���
#if (0 == LPC546XX_MAIN_CLOCK)
    uint16 main_clk_mhz = 180; 
#elif (1 == LPC546XX_MAIN_CLOCK)
    uint16 main_clk_mhz = 200; 
#elif (2 == LPC546XX_MAIN_CLOCK)
    uint16 main_clk_mhz = 220; 
#else
    uint16 main_clk_mhz = 180; 
#endif
//���ϴ��벻�ý��и���
    



static uint32 pllDecodeN(uint32 NDEC)
{
    uint32 n, x, i;

    switch (NDEC)
    {
        case 0x3FF:
            n = 0;
            break;
        case 0x302:
            n = 1;
            break;
        case 0x202:
            n = 2;
            break;
        default:
            x = 0x080;
            n = 0xFFFFFFFFU;
            for (i = NVALMAX; ((i >= 3) && (n == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 2) ^ (x >> 3) ^ (x >> 4)) & 1) << 7) | ((x >> 1) & 0x7F);
                if ((x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P)) == NDEC)   n = i;
            }
            break;
    }
    return n;
}


static uint32 pllDecodeM(uint32 MDEC)
{
    uint32 m, i, x;

    switch (MDEC)
    {
        case 0x1FFFF:
            m = 0;
            break;
        case 0x18003:
            m = 1;
            break;
        case 0x10003:
            m = 2;
            break;
        default:
            x = 0x04000;
            m = 0xFFFFFFFFU;
            for (i = MVALMAX; ((i >= 3) && (m == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 1)) & 1) << 14) | ((x >> 1) & 0x3FFFU);
                if ((x & (PLL_MDEC_VAL_M >> PLL_MDEC_VAL_P)) == MDEC)   m = i;
            }
            break;
    }
    return m;
}

static uint32 pllDecodeP(uint32 PDEC)
{
    uint32 p, x, i;

    switch (PDEC)
    {
        case 0x7F:
            p = 0;
            break;
        case 0x62:
            p = 1;
            break;
        case 0x42:
            p = 2;
            break;
        default:
            x = 0x10;
            p = 0xFFFFFFFFU;
            for (i = PVALMAX; ((i >= 3) && (p == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 2)) & 1) << 4) | ((x >> 1) & 0xFU);
                if ((x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P)) == PDEC)   p = i;
            }
            break;
    }
    return p;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ݼĴ���ֵ����Nֵ
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
static uint32 findPllPreDiv(uint32 ctrlReg, uint32 nDecReg)
{
    uint32 preDiv = 1;

    if ((ctrlReg & SYSCON_SYSPLLCTRL_DIRECTI_MASK) == 0)
    {
        preDiv = pllDecodeN(nDecReg & 0x3FF);
        if (preDiv == 0)    preDiv = 1;
    }
    return preDiv;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ݼĴ���ֵ����Pֵ
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
static uint32 findPllPostDiv(uint32 ctrlReg, uint32 pDecReg)
{
    uint32 postDiv = 1;
    
    if ((ctrlReg & SYSCON_SYSPLLCTRL_DIRECTO_MASK) == 0)
    {
        postDiv = 2 * pllDecodeP(pDecReg & 0x7F);
        if (postDiv == 0)   postDiv = 2;
    }
    return postDiv;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ݼĴ���ֵ����Mֵ
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
static uint32 findPllMMult(uint32 ctrlReg, uint32 mDecReg)
{
    uint32 mMult = 1;
    
    mMult = pllDecodeM(mDecReg & 0x1FFFF);
    if (mMult == 0) mMult = 1;
    return mMult;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����N�Ĵ���ֵ��д��
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
void N_divider(uint16 N)
{
    uint32 x=0x00000080;
    uint32 i;
    uint16 N_max = 256;
    
    if(N > N_max)   N = N_max;

    switch (N) 
    {
        case 0: x = 0xFFFFFFFF;break;
        case 1: x = 0x00000302;break;
        case 2: x = 0x00000202;break;
        default: 
            for (i = N; i <= N_max; i++)
                x = (((x ^ (x>>2) ^ (x>>3) ^ (x>>4)) & 1) << 7) | ((x>>1) & 0x7F); 
    }
    x &= SYSCON_SYSPLLNDEC_NDEC_MASK;
    SYSCON->SYSPLLNDEC = SYSCON_SYSPLLNDEC_NDEC(x);
    SYSCON->SYSPLLNDEC |= SYSCON_SYSPLLNDEC_NREQ(1);
    SYSCON->SYSPLLNDEC = SYSCON_SYSPLLNDEC_NDEC(x) | SYSCON_SYSPLLNDEC_NREQ(0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����P�Ĵ���ֵ��д��
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
void P_divider(uint16 P)
{
    uint32 x=0x10;
    uint32 i;
    uint16 P_max = 0x20;
    
    if(P > P_max)   P = P_max;
    
    switch (P) 
    {
        case 0: x = 0xFFFFFFFF;break;
        case 1: x = 0x00000062;break;
        case 2: x = 0x00000042;break;
        default: 
            for (i = P; i <= P_max; i++)
                x = (((x ^ (x>>2)) & 1) << 4) | ((x>>1) & 0xF); 
    }
    x &= SYSCON_SYSPLLPDEC_PDEC_MASK;
    SYSCON->SYSPLLPDEC = SYSCON_SYSPLLPDEC_PDEC(x);
    SYSCON->SYSPLLPDEC |= SYSCON_SYSPLLPDEC_PREQ(1);
    SYSCON->SYSPLLPDEC = SYSCON_SYSPLLPDEC_PDEC(x) | SYSCON_SYSPLLPDEC_PREQ(0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����M�Ĵ���ֵ��д��
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
void M_divider(uint16 M)
{
    uint32 x=0x00004000;
    uint32 i;
    uint16 M_max = 0x00008000;
    
    if(M > M_max)   M = M_max;
    
    switch (M) 
    {
        case 0: x = 0xFFFFFFFF;break;
        case 1: x = 0x00018003;break;
        case 2: x = 0x00010003;break;
        default: 
            for (i = M; i <= M_max; i++)
                x = (((x ^ (x>>1)) & 1) << 14) | ((x>>1) & 0x3FFF); 
    }
    x &= SYSCON_SYSPLLMDEC_MDEC_MASK;
    SYSCON->SYSPLLMDEC = SYSCON_SYSPLLMDEC_MDEC(x);
    SYSCON->SYSPLLMDEC |= SYSCON_SYSPLLMDEC_MREQ(1);
    SYSCON->SYSPLLMDEC = SYSCON_SYSPLLMDEC_MDEC(x) | SYSCON_SYSPLLMDEC_MREQ(0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����PLLCTRL�Ĵ���ֵ��д��
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
void write_syspllctrl(void)
{
    uint32 M;
    uint32 SELR,SELI,SELP;
    
    M = SYSCON->SYSPLLMDEC & SYSCON_SYSPLLMDEC_MDEC_MASK;
    
    
    if (M < 60)         SELP = (M>>1) + 1;
    else                SELP = 31;
    
    if (M > 16384)      SELI = 1;
    else if (M > 8192)  SELI = 2;
    else if (M > 2048)  SELI = 4;
    else if (M >= 501)  SELI = 8;
    else if (M >=60)    SELI = 4*(1024/(M+9));
    else                SELI = (M & 0x3C) + 4; /* & denotes bitwise AND */
    
    SELR = 0;
    SYSCON->SYSPLLCTRL = 0
                        //| SYSCON_SYSPLLCTRL_UPLIMOFF(1)
                        | SYSCON_SYSPLLCTRL_SELR(SELR)
                        | SYSCON_SYSPLLCTRL_SELI(SELI)
                        | SYSCON_SYSPLLCTRL_SELP(SELP);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PLL��ʼ��
//  @return     void
//  @Note       �û��������
//-------------------------------------------------------------------------------------------------------------------
void pll_init(void)
{
    SYSCON->SYSOSCCTRL = SYSCON_SYSOSCCTRL_FREQRANGE(0);//�����ⲿʱ�ӷ�Χ 1-25MHZ
    
    SYSCON->FLASHCFG |= SYSCON_FLASHCFG_FLASHTIM(0x07); //����flash����ʱ��
  
    SYSCON->PDRUNCFGCLR[0] = (0
                            | SYSCON_PDRUNCFGCLR_PDEN_SYS_PLL_MASK  //����ϵͳPLL��Դ
                            | SYSCON_PDRUNCFGCLR_PDEN_VD2_ANA_MASK  //����ϵͳ������ģ���Դ
                            | SYSCON_PDRUNCFGCLR_PDEN_VD3_MASK);    //��������PLL��Դ
    
    

    if(0 == LPC546XX_PLL_CLOCK_SOURCE)      
    {
        SYSCON->SYSPLLCLKSEL = SYSCON_SYSPLLCLKSEL_SEL(0x00);           //ѡ��PLL����ʱ��Ϊ�ڲ�����
    }
    else if(1 == LPC546XX_PLL_CLOCK_SOURCE) 
    {
        SYSCON->PDRUNCFGCLR[1] = SYSCON_PDRUNCFGCLR_PDEN_SYSOSC_MASK;   //�����ⲿ�����Դ
        SYSCON->SYSPLLCLKSEL = SYSCON_SYSPLLCLKSEL_SEL(0x01);           //ѡ��PLL����ʱ��Ϊ�ⲿ����
    }
    else                                    
    {
        SYSCON->SYSPLLCLKSEL = SYSCON_SYSPLLCLKSEL_SEL(0x00);           //ѡ��PLL����ʱ��Ϊ�ڲ�����
    }
    
    main_clk_mhz = 0;   //����
    P_divider(2);       //���÷�Ƶϵ��
    //���ݺ궨�����ñ�Ƶϵ��
    if(0 == LPC546XX_MAIN_CLOCK)        M_divider(90);      //���ñ�Ƶϵ��
    else if(1 == LPC546XX_MAIN_CLOCK)   M_divider(100);     //���ñ�Ƶϵ��
    else if(2 == LPC546XX_MAIN_CLOCK)   M_divider(105);     //���ñ�Ƶϵ��
    else                                M_divider(90);      //���ñ�Ƶϵ��
    N_divider(3);       //���÷�Ƶϵ��
    
    write_syspllctrl(); //����PLLģʽ���������
    
    SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFGCLR_PDEN_SYS_PLL_MASK;  //�ر�ϵͳPLL��Դ
    SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFGCLR_PDEN_SYS_PLL_MASK;  //����ϵͳPLL��Դ
    
    while(!SYSCON->SYSPLLSTAT);                     //�ȴ�PLL����
    
    SYSCON->AHBCLKDIV = 0;
    
    
    SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(2);//�л�main_clk������ԴΪpll_clk
}



void get_clk(void)
{
    uint32 temp_reg;
    uint32 M_val,P_val,N_val;
    
    temp_reg = SYSCON->SYSPLLCTRL;
    M_val = findPllMMult(temp_reg,SYSCON->SYSPLLMDEC);
    P_val = findPllPostDiv(temp_reg,SYSCON->SYSPLLPDEC);
    N_val = findPllPreDiv(temp_reg,SYSCON->SYSPLLNDEC);
    
    main_clk_mhz = (uint32)EX_REF_CLK*(2*M_val) / N_val / P_val / 1000;//����main_clkƵ��

    //���PRINTF����Ϊ1�����ʼ��printf��ʹ�õĴ���
    //PRINTF�궨��ֵ������LPC546XX_config.h�ļ��ڽ����޸�
#if (1 == PRINTF)
    //��ʼ��printf��ʹ�õĴ��� ���ںš������ʡ����ſ�����LPC546XX_config.h�ļ��ڽ����޸�
    uart_init(DEBUG_UART,DEBUG_UART_BAUD,DEBUG_UART_TX_PIN,DEBUG_UART_RX_PIN);  
#endif
    
    DisableInterrupts;//���������ж�
}

