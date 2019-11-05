#include "SelfBuild_Picture_Deal.h"

extern float expect_L_speed;
extern float expect_R_speed;
extern int choose;
int SPEED_A=0; //210  ֱ��
int SPEED_B=0; //155   ƽ��
int SPEED_C=0; //155   ���
//--ȫ�ֱ���-------------------------------------
uint8 imageUse[Row][Col];			//����ͼ�����
uint8 CameraBinary[Row][Col]={0};
uint8 Three_Line[Row][Col];            //����
uint8 Left_Line[Row]={0};              //����
uint8 Right_Line[Row]={0};             //����
int Mid_Line[Row];                   //����

uint8 Left_Jump_Value;                 //��߶�������
uint8 Right_Jump_Value;                //�ұ߶�������
uint8 Mid_Jump_Value;                  //����ͬʱ��������

int diatance=0;                     //���߼��
uint8 ValueNum;                        //������Ч��
uint8 Road_Type=0;                     //��������
uint8 Road_Keep1=0;
uint8 Road_Keep2=0;
uint8 Mut_Jump_Value;                  //��һ�η�ȫ����
int  Mid_Value;                     //����ֵ

uint8 Mid_Real[Row]=
{
  94,94,93,92,91,90,90,89,88,87,    //0-9
  86,85,84,83,82,81,80,79,78,77,    //10-19
  76,75,74,73,72,72,71,70,69,68,    //20-29
  67,66,65,64,63,62,61,60,59,58,    //30-39
  57,56,55,54,53,52,51,50,49,49,    //40-49
  48,47,46,45,44,43,42,42,41,41,    //50-59
  40,40,40,39,39,39,38,38,38,37,    //60-69
  37,36,36,35,35,33,32,30,29,28,    //70-79
  27,26,25,24,23,21,20,19,18,17,    //80-89
  16,15,14,13,12,11,10,9,8,7,       //90-99
  6,5,5,4,4,3,3,2,2,2,              //100-109
  1,1,1,1,1,1,1,1,1,1               //110-119
};


void Save_image(void)		//����ͼ��
{
	for(int a = 0; a < Col; ++ a)
	{
		for(int b = 0; b < Row; ++b)
		   imageUse[b][a] = image[b][a];
	}
}

void Road_Get(void)                   //������Ϣ��ȡ
{
    int LeftStart=4,RightStart=183;
    uint8  x,y; 
  
    for(y=119;y>116;y--)
    {
         Left_Line[119-y]=0;			   
         Right_Line[119-y]=0;
			   
    }
  
    for(x=2;x<(Col-3);x++)
    {
         switch(CameraBinary[y][x])
         { 
              case Black://��ߵ�
              {
                   if( CameraBinary[y][x-2]==Black
                     &&CameraBinary[y][x-1]==Black
                     &&CameraBinary[y][x+1]==White
                     &&CameraBinary[y][x+2]==White)
                   { 
                        Left_Line[y]=x;
                        LeftStart=x;
                        // break;
                   }
      
              }

              case White://�ұߵ�
              {
                   if( CameraBinary[y][x-2]==White
                     &&CameraBinary[y][x-1]==White
                     &&CameraBinary[y][x+1]==Black
                     &&CameraBinary[y][x+2]==Black) 
                   {
                        Right_Line[y]=x; 
                        RightStart=x;
                        //  break;
                   }
              }
              //break;
        }
   }


  
  //=========================================================================*/
  //----����׷��---------
  for(y=116;y>3;y--)
  {    
       //---����׷��----------
       switch(CameraBinary[y][LeftStart])
       {
            case White:
                 for(x=LeftStart;x>3;x--)
                 {        
                      if( CameraBinary[y][x]==Black
                        &&CameraBinary[y][x-1]==Black
                        /* &&image_Two_Value[y][x-2]==Black
                        &&image_Two_Value[y][x-3]==Black*/)
                      {
                           Left_Line[y]=x;
                           LeftStart=x;                
                           break;  
                      }
                      if(x==4) { Left_Line[y]=1;LeftStart=4;break;}
                 }         
            break;
      
            case Black: 
                 for(x=LeftStart;x<(Col-4);x++)
                 {
                      if(  CameraBinary[y][x+1]==White
                         &&CameraBinary[y][x+2]==White
                         /* &&image_Two_Value[y][x+2]==White
                         &&image_Two_Value[y][x+3]==White*/)                   
                      {
                           Left_Line[y]=x;
                           LeftStart=x;
                           break;
                      }
                       if(x==(Col-5)) { Left_Line[y]=MissLine;LeftStart=(Col-5);break;}    
                 }
            break;
       }      
    
       //----����׷��----------           
       switch(CameraBinary[y][RightStart])
       {
            case White:
                 for(x=RightStart;x<(Col-4);x++)
                 {
                      if(CameraBinary[y][x]==Black
                       &&CameraBinary[y][x+1]==Black
                       /*&&image_Two_Value[y][x+2]==Black
                       &&image_Two_Value[y][x+3]==Black*/)
                      {
                           Right_Line[y]=x;
                           RightStart=x;
                           break;
                      }
                      if(x==(Col-5)) { Right_Line[y]=187;RightStart=(Col-5);break;}            
                 }
                 break;
      
            case Black: 
                 for(x=RightStart;x>3;x--)
                 {
                      if(CameraBinary[y][x-1]==White
                       &&CameraBinary[y][x-2]==White
                       /*&&image_Two_Value[y][x-2]==White
                       &&image_Two_Value[y][x-3]==White*/)
                      {
                           Right_Line[y]=x;
                           RightStart=x;
                           break;
                      }
                       if(x==4) { Right_Line[y]=MissLine;RightStart=4;break;}    
                 }
                 break;
       }
       ValueNum=y;
       diatance=RightStart-LeftStart;//�������ߵľ���   
       if(diatance<15)//̫���������ߴ������߷�������
            break;    
    
       if(Left_Line[119-y]==1&&Right_Line[119-y]==187)
            if(CameraBinary[y][LeftStart]==Black&&CameraBinary[y][RightStart]==Black
             &&CameraBinary[y][LeftStart+2]==Black&&CameraBinary[y][RightStart-2]==Black
             &&CameraBinary[y][LeftStart+4]==Black&&CameraBinary[y][RightStart-4]==Black
             &&CameraBinary[y][LeftStart+6]==Black&&CameraBinary[y][RightStart-6]==Black)
                 break;
  }
}


void Three_Line_Hander(void)      //�������ݴ洢
{
  uint8 x,y; 
  
  for(y=(Row-5);y>3;y--)
    for(x=(Col-1);x>0;x--)
      Three_Line[y][x]=Black;
   
  for(y=0;y<(Row-4);y++)
    for(x=1;x<=(Col-1);x++)
    {        
      if(x==Left_Line[y])
        Three_Line[y][x]=White;
      
      if(x==Right_Line[y])
        Three_Line[y][x]=White; 
      
      if(x==Mid_Line[y])
        Three_Line[y][x]=White;
    }	
}


void Mid_Line_Hander(void)           //���ߴ���
{
     uint8  y,change_point=0; 
     Mid_Jump_Value=0;
     Left_Jump_Value=0;
     Right_Jump_Value=0;
	   for(y=116;y>30;y--)                                 // for(y=Mut_Jump_Value;y>ValueNum;y--)  �ӵ�һ�ηǰ��п�ʼ����
	   {  
	        if(!change_point)
		      if((Left_Line[y]!=1||Right_Line[y]!=187)&&
			 (Left_Line[y-1]!=1||Right_Line[y-1]!=187)&&
			 (Left_Line[y-2]!=1||Right_Line[y-2]!=187)  )
		      {    
                     change_point=1;
		             Mut_Jump_Value=y;//��һ���ǰ���
		       }
		      if(Right_Line[y]!=MissLine||Left_Line[y]!=MissLine)    
		      {
    			     if(Left_Line[y]==1&&Right_Line[y]==187)              //����ͬʱ����
			     {     
			            Mid_Line[y]=MidLine_Expect;
				    if(change_point)
                                                Mid_Jump_Value++;
		              }
		      					
		             else if(Left_Line[y]==1&&Right_Line[y]!=187)         //���߶�ʧ
		             {
			            Throw_The_Left_Line(y);
				    if(change_point)
						Left_Jump_Value++;
		              }
		             else if(Left_Line[y]!=1&&Right_Line[y]==187)         //���߶�ʧ
		             {
                                    Throw_The_Right_Line(y);
				    if(change_point)
						Right_Jump_Value++;
		              }						
		             else                                                 //û����
	                            Mid_Line[y]=(Left_Line[y]+Right_Line[y])/2;
                      }

          }
          for(y=30;y>3;y--)
          {
               if((Left_Line[y]!=MissLine&&Right_Line[y]!=MissLine)||(Left_Line[y]!=1&&Right_Line[y]!=187))
               { 
                      Mid_Line[y]=(Right_Line[y]+Left_Line[y])/2;
                }

               else
               {
                      Mid_Line[y]=MissLine;
                }
           }
}


void Throw_The_Left_Line(uint8 y)//���߶�ʧ
{
      uint8 x,n;
      float k;
      for(n=5;n<15;n++)
      { 
        if((y-n)<0||(y+n)>(Row-4))
        {
          Mid_Line[y]=MissLine;
          break;
        }
        else if((Right_Line[y+n]!=187&&Right_Line[y-n]!=187)&&(Right_Line[y+n]!=MissLine&&Right_Line[y-n]!=MissLine))
        {
          k=Sqrt2(4*n*n+(Right_Line[y-n]-Right_Line[y+n])*(Right_Line[y-n]-Right_Line[y+n]))/(2*n);    
          Mid_Line[y]=(int)(Right_Line[y]-Mid_Real[119-y]*k);
          break;
        }
         else 
            {
              for(x=1;x<3;x++)
              {
                if(Mid_Line[y+x]!=MissLine)
                {
                  Mid_Line[y]=Mid_Line[y+x];
                }
              }
              if(x==3)
              {
                Mid_Line[y]=MissLine;
              }
              break;
            }
	  }  
}


void Throw_The_Right_Line(uint8 y)//���߶�ʧ
{
      uint8 x,n;
      float k;
      for(n=5;n<15;n++)
          { 
            if((y-n)<0||(y+n)>(Row-4))
            {
              Mid_Line[y]=MissLine;
              break;
            }
            else if((Left_Line[y+n]!=1&&Left_Line[y-n]!=1)&&(Left_Line[y+n]!=MissLine&&Left_Line[y-n]!=MissLine))
            {
              k=Sqrt2(4*n*n+(Left_Line[y+n]-Left_Line[y-n])*(Left_Line[y+n]-Left_Line[y-n]))/(2*n);   
              Mid_Line[y]=(int)(Left_Line[y]+Mid_Real[119-y]*k);
              break;
            }
            else 
            {
              for(x=1;x<3;x++)
              {
                if(Mid_Line[y+x]!=MissLine)
                {
                  Mid_Line[y]=Mid_Line[y+x];
                }
              }
              if(x==3)
              {
                Mid_Line[y]=MissLine;
              }
              break;
            }
          }
}

void Road_Juage(void)//���������ж�{0��ֱ��   1�����    2����·   3��·��    4������   5��������   6��ʮ��}
{    
//      uint8 y=0,Left_point_Value=0,Right_point_Value=0,Mid_point_Value=0;
//      Road_Type=0;
//   for(y=Row-3;y>ValueNum;y=y-3)
//  {
//    if(Left_Line[y]!=1&&Left_Line[y-1]!=1) 
//    {
//      
//      if(((Left_Line[y]-Left_Line[y-1])>-5)&&((Left_Line[y]-Left_Line[y-1])<2))
//       {
//         Left_point_Value++;
//         
//       }
//    }
//    if(Right_Line[y]!=187&&Right_Line[y-1]!=187)
//    {
//      if(((Right_Line[y]-Right_Line[y-1])<5)&&((Right_Line[y]-Right_Line[y-1])>-2))
//       {
//         Right_point_Value++;
//
//       } 
//    }
//    
//    if(Mid_Line[y]!=MissLine&&Mid_Line[y-1]!=MissLine)
//    {
//      
//      if(((Mid_Line[y]-Mid_Line[y-1])>-2)&&((Mid_Line[y]-Mid_Line[y-1])<2))
//       {
//         Mid_point_Value++;
//       }
//    }   
//  }
//  SPEED_A=RIGHT->speed+25;//  ֱ��
//  SPEED_B=RIGHT->speed+15;  //  ƽ��
//  SPEED_C=RIGHT->speed; //   ���
//
//  if(Mid_point_Value > 30)
//  {
//     expect_L_speed = SPEED_A;
//     expect_R_speed = SPEED_A; 
//   //Flag_Delay = 5;
//  }
//  else if(Mid_point_Value > 24)
//  {
//     expect_L_speed = SPEED_B;
//     expect_R_speed = SPEED_B; 
// //   Flag_Delay = 5;
//  }
//
//  else
//  {
//     expect_L_speed = SPEED_C;
//     expect_R_speed = SPEED_C;   
//    
//  }
//  
//  
//  /*�����б�*/
// 
//  if(Mut_Jump_Value<=Col-15)//����ʮ���й�ͬ����
//  {
//
//        if(Right_Jump_Value-Left_Jump_Value>20)//�Ҳ�Բ������
//            if(Left_point_Value>15)            //������������15
//            for(y=116;y>20;y--)
//               if(Right_Line[y]-Right_Line[y-2]>20)  //��ͻ���
//                 if((Right_Line[y-4]-Right_Line[y-8]<5)
//                     &&(Right_Line[y-5]-Right_Line[y-6]<5)
//                     &&(Right_Line[y-6]-Right_Line[y-7]<5))   //ͻ���ǰ����������
//       Road_Type=1;          
//      
//       
//       
//        if(Left_Jump_Value-Right_Jump_Value>20)//���Բ������
//            if(Right_point_Value>15)            //������������15
//            for(y=116;y>20;y--)
//               if(Left_Line[y]-Left_Line[y-2]<-20)  //��ͻ���
//                 if((Left_Line[y-4]-Left_Line[y-5]>-5)
//                     &&(Left_Line[y-5]-Left_Line[y-6]>-5)
//                     &&(Left_Line[y-6]-Left_Line[y-7]>-5))   //ͻ���ǰ����������
//       Road_Type=2;
//          
//  }
//  if(Mut_Jump_Value>Col-15)//С����ʮ���й�ͬ����
//  {
//      if(Mid_Jump_Value<15)//��ͬ����С��15��
//      {
//        if(Right_Jump_Value-Left_Jump_Value>5)//�Ҳ�Բ������
//            if(Left_point_Value>15)            //������������20
//            for(y=116;y>20;y--)
//               if(Right_Line[y]-Right_Line[y-2]>25)  //��ͻ���
//                     if((Right_Line[y-4]-Right_Line[y-5]<5)
//                     &&(Right_Line[y-5]-Right_Line[y-6]<5)
//                     &&(Right_Line[y-6]-Right_Line[y-7]<5))   //ͻ���ǰ����������
//                 Road_Type=1;
//        
//        if(Right_Jump_Value-Left_Jump_Value>50)
//            if(Left_point_Value>25&&Right_point_Value<10)
//            for(y=116;y>20;y--)
//               if(Right_Line[y]-Right_Line[y-2]>10)  //��ͻ���
//                    if((Right_Line[y-4]-Right_Line[y-5]<5)
//                     &&(Right_Line[y-5]-Right_Line[y-6]<5)
//                     &&(Right_Line[y-6]-Right_Line[y-7]<5))   //ͻ���ǰ����������
//        Road_Type=1;         
//        
//        
//        
//        
//        if(Left_Jump_Value-Right_Jump_Value>5)//�Ҳ�Բ������
//            if(Right_point_Value>15)            //������������20
//            for(y=116;y>20;y--)
//               if(Left_Line[y]-Left_Line[y-2]<-25)  //��ͻ���
//                 if((Left_Line[y-4]-Left_Line[y-5]>-5)
//                     &&(Left_Line[y-5]-Left_Line[y-6]>-5)
//                     &&(Left_Line[y-6]-Left_Line[y-7]>-5))   //ͻ���ǰ����������
//        Road_Type=2;
//        if(Left_Jump_Value-Right_Jump_Value>50)
//            if(Right_point_Value>25&&Left_point_Value<10)
//            for(y=116;y>20;y--)
//               if(Left_Line[y]-Left_Line[y-2]<-10)  //��ͻ���
//                 if((Left_Line[y-4]-Left_Line[y-5]>-5)
//                     &&(Left_Line[y-5]-Left_Line[y-6]>-5)
//                     &&(Left_Line[y-6]-Left_Line[y-7]>-5))   //ͻ���ǰ����������
//        Road_Type=2;  
//      }
//  }
//  if(Road_Type==1)
//  {
//       Road_Keep1=1;
//  }
//    if(Road_Type==2)
//  {
//       Road_Keep2=1;
//  }
//  
//  
//  if((Right_Jump_Value<20)&&(Mut_Jump_Value>Row-15))
//  {
//       Road_Keep1=0;
//  }
//  if((Left_point_Value<20)&&(Mut_Jump_Value>Row-15))
//  {
//       Road_Keep2=0;
//  }
//  
//  
//  if(Road_Keep1)
//     Road_Type=1;
//  if(Road_Keep2)
//     Road_Type=2;
//  if(choose==4)
//  {
//        OLED_P6x8Int(0,0,Left_point_Value,3);
//        OLED_P6x8Int(0,1,Right_point_Value,3);
//        OLED_P6x8Int(0,2,Mid_point_Value,3);
//  }
//  
//  
//  
////      uint8 x,y,Cross_Junp=0;
////	   Road_Type=0;        //Ĭ��Ϊֱ��
////	   if(Right_Line[ValueNum-1]-Left_Line[ValueNum-1]>40&&
////	      Right_Line[ValueNum-2]-Left_Line[ValueNum-2]>40&&
////	      Right_Line[ValueNum-3]-Left_Line[ValueNum-3]>40&&
////	      Right_Line[ValueNum-4]-Left_Line[ValueNum-4]>40)
////	   {     
////		      Road_Type=2;   //��·
////            }
////	  
////           for(y=Row/2+2;y>Row/2-3;y--)
////	        for(x=Col-5;x>=4;x--)
////	             if(CameraBinary[y][x-2]==White&&
////                        CameraBinary[y][x-1]==White&&
////                        CameraBinary[y][x+1]==Black&&
////                        CameraBinary[y][x+2]==Black)	            
////		                Cross_Junp++;
////           if(Cross_Junp>12)
////	                    Road_Type=5;   //������
////		 
////           if(abs(Left_Jump_Value-Right_Jump_Value)>15)
////			    Road_Type=1;   //���
////		 
////           if(Mid_Jump_Value>10)
////			    Road_Type=6;   //ʮ��			
}

void Mid_read(void)
{
//   int H=119;
//   while(1)
//   {
//        OLED_P6x8Flo(0,0,MidLine_Aver,-4);
//        if(H>=0&&H<=119)
//        {  
//               if(!gpio_get(Button_Up)  && H<119)
//               {
//                     H++;
//                    
//                     while(!gpio_get(Button_Up));
//               }
//               if(!gpio_get(Button_Down) && H>0)
//               {
//                     H--;
//                     while(!gpio_get(Button_Down));
//               }
//               OLED_P6x8Int(0,1,H,3);
//               OLED_P6x8Int(0,2,Mid_Line[H],3);
//        }

  
        OLED_P6x8Int(0,4,Left_Jump_Value,3);
        OLED_P6x8Int(0,5,Right_Jump_Value,3);
        OLED_P6x8Int(0,6,Mid_Jump_Value,3);
        
        OLED_P6x8Int(0,3,Road_Type,3);
        
//        if(!gpio_get(Button_Mid))     
//        {
//               OLED_ClearScreen((0x00));
//               //break;
//        }
//   }
}

float Sqrt2(int a)               //����ƽ����
{    
  int i; 
  float b,c;     
  for(i=0;;i++)    
  {        
    if((a>i* i)&&(a<=(1+i)*(1+i)))       
    {           
      b=(float)(i+(1+i))/2;          
      break;        
    }    
  }     
  for(i=0;i<3;i++)    
  {        
    c=b+(a/b-b)/2;       
    b=c;    
  }     
  return b; 
}