/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		SCTimer/PWM
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴LPC546XX_config.h�ļ��ڰ汾�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#include "common.h"
#include "LPC546XX_iocon.h"
#include "LPC546XX_pll.h"
#include "LPC546XX_sct.h"



void sct_input_mux(SCT_INPUT_MUX_enum mux_ch, SCT_IUPUT_PIN_enum input_pin)
{
    uint8 sct_gpin;

    sct_gpin = input_pin>>2;                                                //���������SCT_GPINģ���

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;             //�򿪶�·����ʱ��
    SYSCON->PRESETCTRLCLR[0] = SYSCON_PRESETCTRL_MUX_RST_MASK;              //�����·���ø�λʱ��
    INPUTMUX->SCT0_INMUX[mux_ch] = INPUTMUX_SCT0_INMUX_INP_N(sct_gpin);     //����SCT���븴��ͨ����SCT_GPINģ��
    SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_INPUTMUX_MASK;             //�رն�·����ʱ��
    
    //����SCT_GPIN  �ⲿ����
    switch(sct_gpin)
    {
        case 0:
        {
            if(SCT0_GPI0_A0       == input_pin)     iocon_init(A0,  ALT4 | PULLUP | DIGITAL | FILTEROFF);// | FILTEROFF
            else if(SCT0_GPI0_A13 == input_pin)     iocon_init(A13, ALT4 | PULLUP | DIGITAL | FILTEROFF);// | FILTEROFF
            else if(SCT0_GPI0_A24 == input_pin)     iocon_init(A24, ALT4 | PULLUP | DIGITAL | FILTEROFF);// | FILTEROFF
            else if(SCT0_GPI0_B5  == input_pin)     iocon_init(B5,  ALT4 | PULLUP | DIGITAL | FILTEROFF);// | FILTEROFF
        }break;                                                                            
                                                                                           
        case 1:                                                                            
        {                                                                                  
            if(SCT0_GPI1_A1       == input_pin)     iocon_init(A1,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI1_A14 == input_pin)     iocon_init(A14, ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI1_A25 == input_pin)     iocon_init(A25, ALT4 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 2:                                                                            
        {                                                                                  
            if(SCT0_GPI2_A2       == input_pin)     iocon_init(A2,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI2_A20 == input_pin)     iocon_init(A20, ALT4 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 3:                                                                            
        {                                                                                  
            if(SCT0_GPI3_A3       == input_pin)     iocon_init(A3,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI3_A21 == input_pin)     iocon_init(A21, ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI3_B6  == input_pin)     iocon_init(B6,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 4:                                                                            
        {                                                                                  
            if(SCT0_GPI4_A4       == input_pin)     iocon_init(A4,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI4_B0  == input_pin)     iocon_init(B0,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI4_B7  == input_pin)     iocon_init(B7,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 5:                                                                            
        {                                                                                  
            if(SCT0_GPI5_A5       == input_pin)     iocon_init(A5,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI5_B1  == input_pin)     iocon_init(B1,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI5_B22 == input_pin)     iocon_init(B22, ALT4 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 6:                                                                            
        {                                                                                  
            if(SCT0_GPI6_A6       == input_pin)     iocon_init(A6,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI6_B2  == input_pin)     iocon_init(B2,  ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI6_B29 == input_pin)     iocon_init(B29, ALT3 | PULLUP | DIGITAL | FILTEROFF);
        }break;                                                                            
                                                                                           
        case 7:                                                                            
        {                                                                                  
            if(SCT0_GPI7_A12      == input_pin)     iocon_init(A12, ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI7_A17 == input_pin)     iocon_init(A17, ALT3 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI7_B19 == input_pin)     iocon_init(B19, ALT4 | PULLUP | DIGITAL | FILTEROFF);
            else if(SCT0_GPI7_B30 == input_pin)     iocon_init(B30, ALT3 | PULLUP | DIGITAL | FILTEROFF);
        }break;
    }

}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      SCT����ͷ�ɼ���ʼ��
//  @param      pclk_mux_ch         ����ʱ���ź�ʹ�õ�SCT����ͨ��
//  @param      pclk_input_pin      ����ʱ�����ӵ�����
//  @param      pclk_trigger        ����ʱ�Ӵ��� ��������ѡ��    RISING��FALLING
//  @return     void
//  Sample usage:           sct_camera_dma(SCT_INPUT_MUX1, SCT0_GPI1_A14, RISING);     //��ʼ��SCT  ����ʱ��ʹ��SCTͨ��1 ����ʹ��A14  �����ش��� 
//-------------------------------------------------------------------------------------------------------------------
void sct_camera_dma(SCT_INPUT_MUX_enum pclk_mux_ch, SCT_IUPUT_PIN_enum pclk_input_pin, TRIGGER_enum pclk_trigger)
{
    uint8  pclk_trig;
    uint32 temp_ctrl;
    
    SYSCON->FROCTRL |= SYSCON_FROCTRL_SEL(0x01) | SYSCON_FROCTRL_HSPDCLK(0x01); //��FRO_HF ����Ϊ96MHz
    
    //����SCTʱ�Ӽ�����Ƶ��
    SYSCON->SCTCLKSEL = SYSCON_SCTCLKSEL_SEL(0x2);  //ѡ��FRO_HF��Ϊʱ��Դ
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_SCT0_RST_MASK;
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_SCT0_MASK;
    
    sct_input_mux(pclk_mux_ch,  pclk_input_pin);    //����SCT���븴��ͨ��������
    
    if(RISING == pclk_trigger)          pclk_trig = 1;
    else if(FALLING == pclk_trigger)    pclk_trig = 2;

    temp_ctrl = SCT0->CTRL;
    SCT0->CTRL |= SCT_CTRL_HALT_H_MASK | SCT_CTRL_HALT_L_MASK;  //��ͣSCT
    
    SCT0->CONFIG = ( 0
                   | SCT_CONFIG_UNIFY(0)                        //˫��ʱ��ģʽ
                   | SCT_CONFIG_CLKMODE(0)                      //ʹ��System Clock��������SCT 
                   | SCT_CONFIG_INSYNC(1<<SCT0_CAMERA_PCLK)     //����ͬ��ģʽ 
                   );
    
    SCT0->STATE = (SCT0->STATE & SCT_STATE_STATE_H_MASK) | SCT_STATE_STATE_L(0U);   //״̬����
    SCT0->EVENT[pclk_mux_ch].STATE = 0xffff;                    //����PCLK�ź��л���SCT_STATE_WAIT_PCLK״̬
    SCT0->EVENT[pclk_mux_ch].CTRL = ( 0 
                                    | SCT_EVENT_CTRL_HEVENT(0)                      //ʹ��State_L״̬��
                                    | SCT_EVENT_CTRL_OUTSEL(0)                      //����Ϊ���� �����������ŵĴ���
                                    | SCT_EVENT_CTRL_IOSEL(pclk_mux_ch)             //ѡ��pclk_mux_chΪ��������
                                    | SCT_EVENT_CTRL_IOCOND(pclk_trig)              //�����ش���  1��������    2���½���
                                    | SCT_EVENT_CTRL_COMBMODE(2)                    //��ʹ��IO���Ų����Ĵ����ź�
                                    | SCT_EVENT_CTRL_STATELD(1)                     //�޸ķ�ʽ��ֱ��������ֵ
                                    | SCT_EVENT_CTRL_STATEV(1)                      //�¼������������µ�״ֵ̬
                                    );
    SCT0->DMA0REQUEST = SCT_DMA0REQUEST_DEV_0(1U<<pclk_mux_ch);                     //ʹ��PCLK�¼�����DMA
    SCT0->CTRL = temp_ctrl;//����SCT���ƼĴ���
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      sct��������ʼ�ɼ�����ͷ���ݣ�
//  @param      void
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void sct_start(void)
{
    SCT0->EVFLAG = (1<<SCT_INPUT_MUX1); //���־λ 
    SCT0->CTRL &= ~SCT_CTRL_HALT_L_MASK;//����SCT
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      sctֹͣ��ֹͣ�ɼ�����ͷ���ݣ�
//  @param      void
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void sct_stop(void)
{
    SCT0->CTRL |= SCT_CTRL_HALT_L_MASK; //ֹͣSCT
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      sctֹͣ�� ��������
//  @param      void
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void sct_restart(void)
{
    SCT0->CTRL |= SCT_CTRL_HALT_L_MASK; //��ͣSCT
    SCT0->EVFLAG = (1<<SCT_INPUT_MUX1); //���־λ 
    SCT0->CTRL &= ~SCT_CTRL_HALT_L_MASK;//����SCT
}


//----------------------------------------------SCT  PWM�������-----------------------------------------------------

const uint32 sct_duty_max[]={SCT0_OUTPUT_CH0_DUTY_MAX,SCT0_OUTPUT_CH1_DUTY_MAX,SCT0_OUTPUT_CH2_DUTY_MAX,SCT0_OUTPUT_CH3_DUTY_MAX,SCT0_OUTPUT_CH4_DUTY_MAX,
                             SCT0_OUTPUT_CH5_DUTY_MAX,SCT0_OUTPUT_CH6_DUTY_MAX,SCT0_OUTPUT_CH7_DUTY_MAX,SCT0_OUTPUT_CH8_DUTY_MAX,SCT0_OUTPUT_CH9_DUTY_MAX};

void sct_pwm_mux(SCT_OUTPUT_PIN_enum pwm_pin)
{
    uint8 sct_out_ch;
    sct_out_ch = pwm_pin>>2;

    switch(sct_out_ch)
    {
        case 0:
        {
            if     (SCT0_OUT0_A2  == pwm_pin)       iocon_init(A2,  ALT3 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT0_A17 == pwm_pin)       iocon_init(A17, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT0_B4  == pwm_pin)       iocon_init(B4,  ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT0_B23 == pwm_pin)       iocon_init(B23, ALT2 | DIGITAL | FILTEROFF);
        }break;
        
        case 1:
        {
            if     (SCT0_OUT1_A3  == pwm_pin)       iocon_init(A3,  ALT3 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT1_A18 == pwm_pin)       iocon_init(A18, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT1_B8  == pwm_pin)       iocon_init(B8,  ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT1_B24 == pwm_pin)       iocon_init(B24, ALT2 | DIGITAL | FILTEROFF);
        }break;
        
        case 2:
        {
            if     (SCT0_OUT2_A15 == pwm_pin)       iocon_init(A15, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT2_A19 == pwm_pin)       iocon_init(A19, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT2_B9  == pwm_pin)       iocon_init(B9,  ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT2_B25 == pwm_pin)       iocon_init(B25, ALT2 | DIGITAL | FILTEROFF);
        }break;
        
        case 3:
        {
            if     (SCT0_OUT3_A22 == pwm_pin)       iocon_init(A22, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT3_A31 == pwm_pin)       iocon_init(A31, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT3_B10 == pwm_pin)       iocon_init(B10, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT3_B26 == pwm_pin)       iocon_init(B26, ALT2 | DIGITAL | FILTEROFF);
        }break;
        
        case 4:
        {
            if     (SCT0_OUT4_A23 == pwm_pin)       iocon_init(A23, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT4_B3  == pwm_pin)       iocon_init(B3,  ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT4_B17 == pwm_pin)       iocon_init(B17, ALT4 | DIGITAL | FILTEROFF);
        }break;
        
        case 5:
        {
            if     (SCT0_OUT5_A26 == pwm_pin)       iocon_init(A26, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT5_B18 == pwm_pin)       iocon_init(B18, ALT4 | DIGITAL | FILTEROFF);
        }break;
        
        case 6:
        {
            if     (SCT0_OUT6_A27 == pwm_pin)       iocon_init(A27, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT6_B31 == pwm_pin)       iocon_init(B31, ALT4 | DIGITAL | FILTEROFF);
        }break;
        
        case 7:
        {
            if     (SCT0_OUT7_A28 == pwm_pin)       iocon_init(A28, ALT4 | DIGITAL | FILTEROFF);
            else if(SCT0_OUT7_B19 == pwm_pin)       iocon_init(B19, ALT2 | DIGITAL | FILTEROFF);
        }break;
        
        case 8:
        {
            if     (SCT0_OUT8_A29 == pwm_pin)       iocon_init(A29, ALT4 | DIGITAL | FILTEROFF);
        }break;
        
        case 9:
        {
            if     (SCT0_OUT9_A30 == pwm_pin)       iocon_init(A30, ALT4 | DIGITAL | FILTEROFF);
        }break;
    }
    
    
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      SCT PWMģʽ��ʼ��
//  @param      pwm_pin     PWMͨ���ż�����
//  @param      freq        PWMƵ��
//  @param      duty        PWMռ�ձ�
//  @return     void
//  Sample usage:           sct_pwm_init(SCT0_OUT2_A15,50,5000);     //��ʼ��SCT0 2ͨ�� ʹ������A15ΪPWMģʽ  Ƶ��50HZ   ռ�ձ�Ϊ�ٷ�֮ 5000/TIMER0_PWM_DUTY_MAX*100
//  @note                   ��������ͷ��PCLK��PWM������Ҫ��ռ��һ��ͨ�������SCTһ��ʮ��ͨ�����ֻ�����8·PWM
//  @note                   ��Ҫ�ر�ע�⣬sctģ���ܹ�������8·PWM������8·PWMƵ�ʶ�����һ�¡������ʺ����ڿ��ƶ�����
//-------------------------------------------------------------------------------------------------------------------
void sct_pwm_init(SCT_OUTPUT_PIN_enum pwm_pin, uint32 freq, uint32 duty)
{
    uint8  sct_out_ch;
    uint16 temp_div,prescale;
    uint32 temp,temp_res;
    uint32 temp_ctrl;
    uint32 match_temp;
    uint32 period_temp;
    uint32 sct_clk = main_clk_mhz*1000*1000;

    for(temp_div=1;temp_div<=256;temp_div++)
    {
        temp = sct_clk/freq/temp_div;
        if(temp<=(0xffff)) break;             //�ҵ���Ѳ���
    }

    if(257 == temp_div)   ASSERT(0);        //����ʧ�� Ƶ�ʹ��ͻ�����Ƶ����
    
    sct_out_ch = pwm_pin>>2;                //����ͨ��
    ASSERT(duty<=sct_duty_max[sct_out_ch]); //���ռ�ձȶ���
    
    
    prescale = temp_div - 1;                //�����Ƶϵ��
    period_temp = sct_clk/freq/temp_div;    //���ڼ�������
    match_temp = duty*period_temp/sct_duty_max[sct_out_ch];//����ռ�ձ�ƥ����

    SYSCON->FROCTRL |= SYSCON_FROCTRL_SEL(0x01) | SYSCON_FROCTRL_HSPDCLK(0x01); 
    SYSCON->SCTCLKSEL = SYSCON_SCTCLKSEL_SEL(0x2);
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_SCT0_RST_MASK;
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_SCT0_MASK; //����SCTʱ��

    sct_pwm_mux(pwm_pin);
    
    temp_ctrl = SCT0->CTRL;
    SCT0->CTRL |= SCT_CTRL_HALT_H_MASK | SCT_CTRL_HALT_L_MASK;  //��ͣSCT
    
    temp_ctrl &= ~SCT_CTRL_PRE_H_MASK;             //�����Ƶϵ��
    temp_ctrl |= SCT_CTRL_PRE_H(prescale);         //���÷�Ƶϵ��
        
    SCT0->CONFIG = ( 0
                   | SCT_CONFIG_UNIFY(0)                    //˫��ʱ��ģʽ
                   | SCT_CONFIG_CLKMODE(0)                  //ʹ��System Clock��������SCT 
                   | SCT_CONFIG_INSYNC(1<<SCT0_CAMERA_PCLK) //����ͬ��ģʽ
                   );
    
    SCT0->LIMIT |= SCT_LIMIT_LIMMSK_H(1U << SCT0_PWM_PERIOD_CH);
    SCT0->REGMODE = 0;//ƥ��ģʽ
    
    SCT0->STATE = (SCT0->STATE & SCT_STATE_STATE_L_MASK) | SCT_STATE_STATE_H(0U);   //״̬����
    
    //���������¼�
    SCT0->SCTMATCH[SCT0_PWM_PERIOD_CH] = period_temp<<SCT_SCTMATCH_MATCHn_H_SHIFT;
    SCT0->SCTMATCHREL[SCT0_PWM_PERIOD_CH] = period_temp<<SCT_SCTMATCH_MATCHn_H_SHIFT;
    SCT0->EVENT[SCT0_PWM_PERIOD_CH].STATE = 1;
    SCT0->EVENT[SCT0_PWM_PERIOD_CH].CTRL = ( 0
                                           | SCT_EVENT_CTRL_MATCHSEL(SCT0_PWM_PERIOD_CH)
                                           | SCT_EVENT_CTRL_HEVENT(1)                   //ʹ��State_H״̬��
                                           | SCT_EVENT_CTRL_OUTSEL(1)                   //����Ϊ��� 
                                           | SCT_EVENT_CTRL_IOSEL(SCT0_PWM_PERIOD_CH)   //ѡ�������ͨ��
                                           | SCT_EVENT_CTRL_COMBMODE(1)                 //��ʹ��ָ����ƥ����
                                           );
    //����ƥ���¼�
    SCT0->SCTMATCH[sct_out_ch] = match_temp<<SCT_SCTMATCH_MATCHn_H_SHIFT;               //ƥ��ֵ
    SCT0->SCTMATCHREL[sct_out_ch] = match_temp<<SCT_SCTMATCH_MATCHn_H_SHIFT;            //ƥ��ֵ
    SCT0->EVENT[sct_out_ch].STATE = 1;  
    SCT0->EVENT[sct_out_ch].CTRL = ( 0  
                                     | SCT_EVENT_CTRL_MATCHSEL(sct_out_ch)  
                                     | SCT_EVENT_CTRL_HEVENT(1)                         //ʹ��State_H״̬��
                                     | SCT_EVENT_CTRL_OUTSEL(1)                         //����Ϊ��� 
                                     | SCT_EVENT_CTRL_IOSEL(sct_out_ch)                 //ѡ�������ͨ��
                                     | SCT_EVENT_CTRL_COMBMODE(1)                       //��ʹ��ָ����ƥ����
                                     );
    
    SCT0->OUT[sct_out_ch].CLR = 1<<sct_out_ch;             //�������� �����¼�

    temp_res = SCT0->RES;                   //��ȡ�Ĵ���
    temp_res &= ~((uint32)SCT_RES_O0RES_MASK<<(2*sct_out_ch));
    if(0 == match_temp)
    {
        temp_res |= 2<<(2*sct_out_ch);  
        SCT0->RES = temp_res;               //ͬʱ����
        SCT0->OUT[sct_out_ch].SET = 1<<sct_out_ch;             //������һ �����¼�
    }
    else
    {
        temp_res |= 1<<(2*sct_out_ch);  
        SCT0->RES = temp_res;               //ͬʱ����
        SCT0->OUT[sct_out_ch].SET = 1<<SCT0_PWM_PERIOD_CH;     //������һ �����¼�
    }

    temp_ctrl &= ~SCT_CTRL_HALT_H_MASK;     //����SCT
    SCT0->CTRL = temp_ctrl;                 //����SCT���ƼĴ���
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      SCT PWMռ�ձ�����
//  @param      pwm_pin     PWMͨ���ż�����
//  @param      duty        PWMռ�ձ�
//  @return     void
//  Sample usage:           sct_pwm_duty(SCT0_OUT2_A15,5000);     //SCT0 2ͨ�� ռ�ձ�����Ϊ�ٷ�֮ 5000/TIMER0_PWM_DUTY_MAX*100
//-------------------------------------------------------------------------------------------------------------------
void sct_pwm_duty(SCT_OUTPUT_PIN_enum pwm_pin, uint32 duty)
{
    uint8  sct_out_ch;
    uint32 period_temp;
    uint32 match_temp;
    uint32 temp_res;
        
    sct_out_ch = pwm_pin>>2;                //����ͨ��
    ASSERT(duty<=sct_duty_max[sct_out_ch]); //���ռ�ձȶ���
    
    period_temp = SCT0->SCTMATCHREL[SCT0_PWM_PERIOD_CH]>>SCT_SCTMATCHREL_RELOADn_H_SHIFT;
    match_temp = duty*period_temp/sct_duty_max[sct_out_ch];     //����ռ�ձ�ƥ����
    
    temp_res = SCT0->RES;                   //��ȡ�Ĵ���
    temp_res &= ~((uint32)SCT_RES_O0RES_MASK<<(2*sct_out_ch));
    if(0 == match_temp)
    {
        temp_res |= 2<<(2*sct_out_ch);  
        SCT0->RES = temp_res;               //ͬʱ����
        SCT0->OUT[sct_out_ch].SET = 1<<sct_out_ch;             //������һ �����¼�
    }
    else
    {
        temp_res |= 1<<(2*sct_out_ch);  
        SCT0->RES = temp_res;               //ͬʱ����
        SCT0->OUT[sct_out_ch].SET = 1<<SCT0_PWM_PERIOD_CH;     //������һ �����¼�
    }

    SCT0->SCTMATCHREL[sct_out_ch] = match_temp<<SCT_SCTMATCH_MATCHn_H_SHIFT;    //ƥ��ֵ
}

