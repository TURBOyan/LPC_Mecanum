/****************************************************************************
 * @�ļ�       SelfBuild_OLEDMenu.c
 * @����       ������
 * @����       �˵������ڵ�������
 * @ע��       �ó���ʹ�û���OLED�������������Ӧ��������ȷ�ϣ���FLASH��д
               ��һ�����������ɹرգ�����ֲʱ��ȷ����ʼ��
 * @���ʱ��   2018-4-25
****************************************************************************/
#include "SelfBuild_OLEDMenu.h"

Menu_TypeDef Menu[MENU_MAX]={0};

/******************�޸Ĳ��ִ����п�ʼ�����������벻Ҫ����Ķ�******************/

//����˵�ʹ�õ��ⲿȫ�ֱ�������


__STATIC_INLINE void Menu_Data_Init(void)
{       
//�˵�����
//����
//
//Menu[3].Title = "PID_SPEED";                  //һ���˵���ʾ����PID_SPEED
//Menu[3].Sub_Value[0] = &PID.Proportion;       //�����˵���һ���ɵ����Ĳ�����PID.Proportion
//Menu[3].Sub_Title[0] = "SPEED_P";             //�ñ����ڲ˵�����ʾ����SPEED_P
//Menu[3].Sub_Type[0] = -4;                     //�ñ���������Ϊfloat ע�⣺-4����float -2����int16 2����uint16
//ע�⣬Menu����ѡ��ΧΪ1~(MENU_MAX-1)��Menu.Sub�������ѡ��Χ��0~(SUBMENU_MAX-1)������
//ʹ��Menu[0]��Ч������
//Sub_Type����ѡ����������������
//���ı�����Ϊ16(16������������?)
	Menu[1].Title = "PID_Dis";
	Menu[1].Sub_Value[0] = &PID_Dis.Param_Kp;
	Menu[1].Sub_Title[0] = "P";
	Menu[1].Sub_Type[0] =-4;
	Menu[1].Sub_Value[1] = &PID_Dis.Param_Kd;
	Menu[1].Sub_Title[1] = "D";
	Menu[1].Sub_Type[1] =-4;
	
	Menu[2].Title = "PID_Speed";
	Menu[2].Sub_Value[0] = &PID_Speed.Param_Kd;
	Menu[2].Sub_Title[0] = "D";
	Menu[2].Sub_Type[0] =-4;
	Menu[2].Sub_Value[1] = &PID_Speed.Param_Ki;
	Menu[2].Sub_Title[1] = "I";
	Menu[2].Sub_Type[1] =-4;
	
	Menu[3].Title = "PID_Dir";
	Menu[3].Sub_Value[0] = &PID_Dir.Param_Kp;
	Menu[3].Sub_Title[0] = "P";
	Menu[3].Sub_Type[0] =-4;
	Menu[3].Sub_Value[1] = &PID_Dir.Param_Kd;
	Menu[3].Sub_Title[1] = "D";
	Menu[3].Sub_Type[1] =-4;

}

/******************�޸Ĳ��ִ����н��������������벻Ҫ����Ķ�******************/

static void Menu_HomePage_Show(void);
static void Menu_SubPage_Show(Menu_TypeDef* Menu_Type);
static int8 Button_Get(void);
//��ʾ�����˵�����
static void Menu_HomePage_Show(void)
{
	uint8 temp;
	LED_Fill(0x00);//����
	LED_P6x8Str((OLED_X_MAX - 1 - 6 * (sizeof("Menu") - 1)) / 2, 0, "Menu");//��ʾ����
	
	for(temp = 0; temp < MENU_MAX; ++temp)
	{
		if(Menu[temp].Title != 0)
		{
			LED_P6x8Str(MENU_SHOW((temp % 2)) + 8, temp / 2 + 1, (char*)Menu[temp].Title);
		}
	}
}

//��ʾ�����˵�����
static void Menu_SubPage_Show(Menu_TypeDef* Menu_Type)
{
	uint8 temp;
		LED_Fill(0x00);//����
	
	for(temp = 0; Menu_Type->Title[temp] != 0; ++temp);//��ʾ����
	LED_P6x8Str((OLED_X_MAX - 1 - 6 * temp) / 2, 0, (char*)Menu_Type->Title);
	
	for(temp = 0; temp < Menu_Type->Sub_Value_Num; ++temp)
	{
		LED_P6x8Str(8, temp + 1, (char*)Menu_Type->Sub_Title[temp]);
		if(Menu_Type->Sub_Type[temp] == -2)
			OLED_P6x8Int(80, temp + 1, *((int16*)Menu_Type->Sub_Value[temp]), -3);
		else if(Menu_Type->Sub_Type[temp] == 2)
			OLED_P6x8Int(80, temp + 1, *((uint16*)Menu_Type->Sub_Value[temp]), -3);
		else if(Menu_Type->Sub_Type[temp] == -4)
		{
			if(*((float*)Menu_Type->Sub_Value[temp]) >= 0)
				OLED_P6x8Flo(80, temp + 1, *((float*)Menu_Type->Sub_Value[temp]) + 0.0001f, -2);
			else
				OLED_P6x8Flo(80, temp + 1, *((float*)Menu_Type->Sub_Value[temp]) - 0.0001f, -2);
		}
	}
	
	LED_P6x8Str(8, 6, "SAVE");
	LED_P6x8Str(8, 7, "BACK");
}

static int8 Button_Get(void)
{
	int8 return_flag = 0;
	if(gpio_get(BUTTON_UP) == 0)
		return_flag |= 0x01;
	if(gpio_get(BUTTON_DOWN) == 0)
		return_flag |= 0x02;
	if(gpio_get(BUTTON_LEFT) == 0)
		return_flag |= 0x04;
	if(gpio_get(BUTTON_RIGHT) == 0)
		return_flag |= 0x08;
	if(gpio_get(BUTTON_CONFIRM) == 0)
		return_flag |= 0x10;
	
	return return_flag;
}

void* Copy_Data[16] = {0};
int8 Copy_Type[16] = {0};
uint32 ZERO[32]={0};
//-------------------------------------------------------------------------
//  @����         �˵���ʼ������
//  @����         ���ڳ�ʼ��Menu���������
//-------------------------------------------------------------------------
void Menu_Init(void)
{
	uint8 temp, Sub_temp, Sub_Menu_Sum = 0, Sub_Sum_temp = 0;
	
	Menu_Data_Init();
	Menu[0].Title = "CloseMenu";
	Menu[0].Sub_Title[0] = 0;
	eeprom_init();
	
	//��Menu��������б���������Copy_Data�����У�������FLASH�е������бȽϣ������Menu���ж��ٱ���
	for(temp = 1; temp < MENU_MAX; ++temp)
	{
     if(Menu[temp].Title == 0) continue;
		for(Sub_temp = 0; (Sub_temp < SUBMENU_MAX) && (Menu[temp].Sub_Title[Sub_temp] != 0); ++Sub_temp)
		{
			Copy_Data[Sub_Menu_Sum & 0x0f] = Menu[temp].Sub_Value[Sub_temp];
			Copy_Type[Sub_Menu_Sum & 0x0f] = Menu[temp].Sub_Type[Sub_temp];
			if(Menu[temp].Sub_Type[Sub_temp] == -2)
			{
				if(*((int16*)Copy_Data[Sub_Menu_Sum & 0x0f]) !=  my_EEPROM_READ_WORD(FLASH_SAVE_RAW_MENU,Sub_Menu_Sum ,int16))
					Sub_Menu_Sum |= 0x10;
			}
			else if(Menu[temp].Sub_Type[Sub_temp] == 2)
			{
				if(*((uint16*)Copy_Data[Sub_Menu_Sum & 0x0f]) !=  my_EEPROM_READ_WORD(FLASH_SAVE_RAW_MENU,Sub_Menu_Sum ,uint16))
					Sub_Menu_Sum |= 0x10;
			}
			else if(Menu[temp].Sub_Type[Sub_temp] == -4)
			{

				if(*((float*)Copy_Data[Sub_Menu_Sum & 0x0f]) != my_EEPROM_READ_WORD(FLASH_SAVE_RAW_MENU,Sub_Menu_Sum ,float)) 
				 {
					
					Sub_Menu_Sum |= 0x10;
				 }
			}
			++Sub_Menu_Sum;
		}
		Menu[temp].Sub_Value_Num = Sub_temp;
	}
	
	if(Sub_Menu_Sum & 0x10)//�������Ľ��Ϊ��һ������˵��Menu����ı���ֵ�����޸ģ���֮ǰFLASH�еĲ�������д�룬�Ա���һ�μ����
	{
		Sub_Menu_Sum &= ~0x10;
//		FLASH_EraseSector(FLASH_SAVE_RAW_MENU);
//		FLASH_EraseSector(FLASH_SAVE_PRO_MENU);
//		eeprom_write_page(FLASH_SAVE_RAW_MENU, ZERO);
//		eeprom_write_page(FLASH_SAVE_PRO_MENU, ZERO);
		for(temp = 0; temp < Sub_Menu_Sum; ++temp)
		{
			if(Copy_Type[temp] == -2)
			{
				 my_eeprom_write_word(FLASH_SAVE_RAW_MENU,temp,*((int16*)Copy_Data[temp]));
				 my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((int16*)Copy_Data[temp]));
			}
			else if(Copy_Type[temp] == 2)
			{
				 my_eeprom_write_word(FLASH_SAVE_RAW_MENU,temp,*((uint16*)Copy_Data[temp]));
				 my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((uint16*)Copy_Data[temp]));
			}
			else if(Copy_Type[temp] == -4)
			{
				 my_eeprom_write_word(FLASH_SAVE_RAW_MENU,temp,*((int32*)Copy_Data[temp]));
				 my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((int32*)Copy_Data[temp]));
				
			}
		}
	}
	else//�������Ľ��Ϊһ��������Ϊ����û�жԱ���ֵ�����޸ģ���Ĭ��ʹ�ñ�����FLASH�е�ֵ��Ϊ��������ֵ��������Щֵ���Ƶ�Copy_Data��
	{
		for(temp = 0, Sub_Sum_temp = 0; temp < MENU_MAX; ++temp)
		{
			if(Menu[temp].Title != 0)
			{
				for(Sub_temp = 0; Sub_temp < Menu[temp].Sub_Value_Num; ++Sub_temp)
				{
					if(Menu[temp].Sub_Type[Sub_temp] == -2)
					{
						  	*((int16*)Copy_Data[Sub_Sum_temp])= my_EEPROM_READ_WORD(FLASH_SAVE_PRO_MENU,Sub_Sum_temp,int16); 
					}
					else if(Menu[temp].Sub_Type[Sub_temp] == 2)
					{
						*((uint16*)Copy_Data[Sub_Sum_temp]) =  my_EEPROM_READ_WORD(FLASH_SAVE_PRO_MENU,Sub_Sum_temp,uint16); 
					}
					else if(Menu[temp].Sub_Type[Sub_temp] == -4)
					{
						*((float*)Copy_Data[Sub_Sum_temp]) =  my_EEPROM_READ_WORD(FLASH_SAVE_PRO_MENU,Sub_Sum_temp,float);
					}
					++Sub_Sum_temp;
				}
			}
		}
	}
	
	Menu_HomePage_Show();
}

//-------------------------------------------------------------------------
//  @����         �˵���������
//  @����         ���øú���ʹ�˵�����
//  @����ֵ       0:�˵�û�а��¡�CloseMenu��ѡ�� 1:�˵����¡�CloseMenu��ѡ��
//-------------------------------------------------------------------------
uint8 Menu_Work(void)
{
	uint16 temp, temp_Sum = 0;
	int8 Button_Flag = 0;
	static int8 Current_Count = 0, Current_Count_Last = 0, Confirm_Flag = 0, Sub_Current_Count = 0, Sub_Current_Count_Last = 0, Sub_Confirm_Flag = 0;
	//Current_Count:һ���˵��ļ���ֵ�����ڿ�����ʾ����λ�ã���֪�������ָ�Ĳ˵�
	//Current_Count_Last:һ���˵�����һ�μ���ֵ�����ڿ����������λ��
	//Confirm_Flag:һ���˵���ȷ�ϱ�־������һ���������˵�
	//Sub_Current_Count:�����˵��ļ���ֵ�����ڿ�����ʾ����λ�ã���֪�������ָ�ı���
	//Sub_Current_Count_Last:�����˵�����һ�μ���ֵ�����ڿ����������λ��
	//Sub_Confirm_Flag:�����˵���ȷ�ϱ�־������һ������������ʼ�޸�
	static uint16 Copy_Data_Cnt = 0, Return_Flag = 0;
	static float temp_Data[SUBMENU_MAX]={0};
	Current_Count_Last = Current_Count;
	Sub_Current_Count_Last = Sub_Current_Count;
	Button_Flag = Button_Get();
	
	if(Button_Flag & 0x0f)//������������ұ�����
	{
		if(Confirm_Flag == 0)//��һ���˵���ȷ�ϱ�־û����һʱ����ʱOLED��ʾΪһ���˵����棬�������������ƶ�
		{
			if(Button_Flag & 0x01) Current_Count -= 2;
			if(Button_Flag & 0x02) Current_Count += 2;
			if(Button_Flag & 0x04) Current_Count -= 1;
			if(Button_Flag & 0x08) Current_Count += 1;
		}
		else//һ���˵�ȷ�ϱ�־��һ����ʱOLED��ʾΪ�����˵�
		{
			if(Sub_Confirm_Flag == 0)
			{
				if(Button_Flag & 0x01) Sub_Current_Count -= 1;//�������˵�ȷ�ϱ�־û����һ������������������
				if(Button_Flag & 0x02) Sub_Current_Count += 1;
				if(Button_Flag & 0x04) Sub_Current_Count -= 1;
				if(Button_Flag & 0x08) Sub_Current_Count += 1;
			}
			else//���򣬰��������Ϊ����������ֵ
			{
				if(Menu[Current_Count].Sub_Type[Sub_Current_Count] == -2)
				{
					if(Button_Flag & 0x01) temp_Data[Sub_Current_Count] += 5;
					if(Button_Flag & 0x02) temp_Data[Sub_Current_Count] -= 5;
					if(Button_Flag & 0x04) temp_Data[Sub_Current_Count] -= 1;
					if(Button_Flag & 0x08) temp_Data[Sub_Current_Count] += 1;
					OLED_P6x8Int(80, Sub_Current_Count + 1, (int16)temp_Data[Sub_Current_Count], -3);
				}
				else if(Menu[Current_Count].Sub_Type[Sub_Current_Count] == 2)
				{
					if(Button_Flag & 0x01) temp_Data[Sub_Current_Count] += 5;
					if(Button_Flag & 0x02) temp_Data[Sub_Current_Count] -= 5;
					if(Button_Flag & 0x04) temp_Data[Sub_Current_Count] -= 1;
					if(Button_Flag & 0x08) temp_Data[Sub_Current_Count] += 1;
					if(temp_Data[Sub_Current_Count] < 0) temp_Data[Sub_Current_Count] = 0;
					OLED_P6x8Int(80, Sub_Current_Count + 1, (int16)temp_Data[Sub_Current_Count], -3);
				}
				else if(Menu[Current_Count].Sub_Type[Sub_Current_Count] == -4)
				{
					if(Button_Flag & 0x01) temp_Data[Sub_Current_Count] += 0.1f;
					if(Button_Flag & 0x02) temp_Data[Sub_Current_Count] -= 0.1f;
					if(Button_Flag & 0x04) temp_Data[Sub_Current_Count] -= 0.01f;
					if(Button_Flag & 0x08) temp_Data[Sub_Current_Count] += 0.01f;
					if(temp_Data[Sub_Current_Count] >= 0)
						OLED_P6x8Flo(80, Sub_Current_Count + 1, temp_Data[Sub_Current_Count] + 0.0001f, -2);
					else
						OLED_P6x8Flo(80, Sub_Current_Count + 1, temp_Data[Sub_Current_Count] - 0.0001f, -2);
				}
			}
		}
	}
	
	//����Current_Count��Sub_Current_Count�Ĵ�С
	if(Confirm_Flag == 0)
	{
		if(Current_Count < 0) Current_Count += 14;
		if(Current_Count > 13) Current_Count -= 14;
	}
	else if(Confirm_Flag == 1)
	{
		if(Sub_Current_Count < 0) 
			Sub_Current_Count = 6;
		//������Ϊ�����˵���BACK��SAVEѡ�������ѡ�����������Ҫ�������
		if(Sub_Current_Count > Menu[Current_Count].Sub_Value_Num - 1 && Sub_Current_Count < 5 && Sub_Current_Count > Sub_Current_Count_Last) 
			Sub_Current_Count = (Sub_Current_Count - Menu[Current_Count].Sub_Value_Num) + 5;
		if(Sub_Current_Count > Menu[Current_Count].Sub_Value_Num - 1 && Sub_Current_Count < 5 && Sub_Current_Count < Sub_Current_Count_Last) 
			Sub_Current_Count = Sub_Current_Count - 5 + Menu[Current_Count].Sub_Value_Num;
		else if(Sub_Current_Count > 6) 
			Sub_Current_Count = 0;
	}
	else if(Confirm_Flag == 2)
	{
		if(Sub_Current_Count < 5) 
			Sub_Current_Count = 6;
		else if(Sub_Current_Count > 6) 
			Sub_Current_Count = 5;
	}
	
	if(Confirm_Flag == 0)//��ʾ��꣬����֮ǰ�Ĺ��
	{
		LED_P6x8Str(MENU_SHOW(Current_Count_Last % 2), (Current_Count_Last) / 2 + 1, " ");
		LED_P6x8Str(MENU_SHOW(Current_Count % 2), (Current_Count) / 2 + 1, ">");
	}
	else
	{
		LED_P6x8Str(0, Sub_Current_Count_Last + 1, " ");
		if(Sub_Current_Count < Menu[Current_Count].Sub_Value_Num && Sub_Confirm_Flag == 1)
			LED_P6x8Str(0, Sub_Current_Count + 1, "*");
		else LED_P6x8Str(0, Sub_Current_Count + 1, ">");
	}
	
	if(Button_Flag & 0x10)//ȷ�ϰ���
	{
		if(Current_Count != 0)
		{
			if(Confirm_Flag == 0)//�����ڵ�һ�˵���ʱ��
			{
				if(Menu[Current_Count].Title != 0)
				{
					Confirm_Flag = 1;
					Sub_Current_Count_Last = 0, Sub_Current_Count = 0;//������ƶ����˵��ı���
					Menu_SubPage_Show(&(Menu[Current_Count]));//��ʾ�����˵�
					for(temp = 0; temp < Menu[Current_Count].Sub_Value_Num; ++temp)//���˵��еı���ֵ�ȴ��浽temp_Data��
					{
						if(Menu[Current_Count].Sub_Type[temp] == -2)
							temp_Data[temp] = *((int16 *)Menu[Current_Count].Sub_Value[temp]);
						else if(Menu[Current_Count].Sub_Type[temp] == 2)
							temp_Data[temp] = *((uint16 *)Menu[Current_Count].Sub_Value[temp]);
						else if(Menu[Current_Count].Sub_Type[temp] == -4)
							temp_Data[temp] = *((float *)Menu[Current_Count].Sub_Value[temp]);
					}
					for(temp = 0, Copy_Data_Cnt = 0; temp < Current_Count; ++temp)//����õ���ֵ��Copy_Data�����е�λ��
						Copy_Data_Cnt += Menu[temp].Sub_Value_Num;
				}
			}
			else if(Confirm_Flag == 1)//�����ڵڶ�������ʱ��
			{
				if(Sub_Current_Count == 6 || Sub_Current_Count == 5)//������µ���SAVE����BACK��ʱ��
				{
					if(Sub_Current_Count == 5)//������µ���SAVE����
					{
						for(temp = 0; temp < Menu[Current_Count].Sub_Value_Num; ++temp)//��temp_Data�е���ʱ����ֵ���浽����
							{
								if(Menu[Current_Count].Sub_Type[temp] == -2)
								{
									if(temp_Data[temp] >= 0)
										*((int16*)Copy_Data[temp + Copy_Data_Cnt]) = (int16)(temp_Data[temp] + 0.0001f);
									else
										*((int16*)Copy_Data[temp + Copy_Data_Cnt]) = (int16)(temp_Data[temp] - 0.0001f);
								}
								else if(Menu[Current_Count].Sub_Type[temp] == 2)
								{
									*((uint16*)Copy_Data[temp + Copy_Data_Cnt]) = (uint16)(temp_Data[temp] + 0.0001f);
								}
								else if(Menu[Current_Count].Sub_Type[temp] == -4)
								{
									if(temp_Data[temp] >= 0)
										*((float*)Copy_Data[temp + Copy_Data_Cnt]) = (int16)((temp_Data[temp] + 0.0001f) * 100)  / 100.0;//���̶ȵı�֤float�ľ���
									else
										*((float*)Copy_Data[temp + Copy_Data_Cnt]) = (int16)((temp_Data[temp] - 0.0001f) * 100)  / 100.0;//���̶ȵı�֤float�ľ���
								}
							}
						//FLASH_EraseSector(FLASH_SAVE_PRO_MENU);//���޸�֮���ֵ����FLASH��
           //	eeprom_write_page(FLASH_SAVE_PRO_MENU, ZERO);
						for(temp = 0, temp_Sum = 0; temp < MENU_MAX; ++temp) temp_Sum += Menu[temp].Sub_Value_Num;
						for(temp = 0; temp < temp_Sum; ++ temp)
						{
							if(Copy_Type[temp] == -2)
							  my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((int16*)Copy_Data[temp]));
							else if(Copy_Type[temp] == -4)
							 my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((int32*)Copy_Data[temp]));
							else if(Copy_Type[temp] == 2)
							 my_eeprom_write_word(FLASH_SAVE_PRO_MENU,temp,*((uint16*)Copy_Data[temp]));
						}
					}
					Copy_Data_Cnt = 0;
					for(temp = 0; temp < Menu[Current_Count].Sub_Value_Num; ++temp) temp_Data[temp] = 0;
					Confirm_Flag = 0;
					Menu_HomePage_Show();//������ʾһ���˵�������־λ����
					LED_P6x8Str(MENU_SHOW(Current_Count % 2), (Current_Count) / 2 + 1, ">");
				}
				else if(Sub_Current_Count < Menu[Current_Count].Sub_Value_Num)//������µĲ���BACK��SAVEѡ����µ�ȷ����Ϊ�����ͽ����������
				{
					Sub_Confirm_Flag ^= 1;
				}
			}
		}
		else
		{
			Return_Flag = 1;
			Button_Flag = 0;
			while(gpio_get(BUTTON_CONFIRM) == 0);
				LED_Fill(0x00);
		}
	}
        
#if BUZZER_EN
	if(Button_Flag != 0) gpio_set(BUZZER, 0);
	else gpio_set(BUZZER, 0);
#endif
	return Return_Flag;
}
