/*
 * @file       main.c
 * @brief      ������
 */

#include "common.h"
#include "include.h"


float adc_ave(ADCn_Ch_e adcn_ch, ADC_nbit bit, int N);//��ֵ�˲�����
uint16 adc_once(ADCn_Ch_e adcn_ch, ADC_nbit bit);
void adc_maxmin_update();
void roadturncal();
void sendimg();

extern uint8 Style;
extern uint8 RoadType;
unsigned char cmos[60][80]={0};                         //���д��������

                                                        //��Ų��� s 8.5
//extern int   AD_val_1;
//extern int   AD_val_2;
//extern int   AD_val_3;
//extern int   AD_val_4;
//extern int   adtmp1,adtmp2,adtmp3,adtmp4;
//extern int   dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4;
//extern int   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4;
//extern int   AD_val_1_min;
//extern int   AD_val_2_min;
//extern int   AD_val_3_min;
//extern int   AD_val_4_min;
//extern int   AD_val_1_max;
//extern int   AD_val_2_max;
//extern int   AD_val_3_max;
//extern int   AD_val_4_max;

float display1=0,display2=0,display3=0;
float display4=0,display5=0,display6=0;

void  main(void)
{ //unsigned char xx,yy;
  uart_init(UART0,9600);
  uart_putchar(UART0,'O');
  uart_putchar(UART0,'K');
  init();
  Stop=1;
  
//  int sum;
//  int i;
//    
//  if(1){                                                //����������Сֵ
//   for(i=0;i<50;i++)                    //������е���Сֵ
//   {
//     AD_val_1 =adc_once(ADC1_SE8, ADC_16bit);
//     sum+=AD_val_1;
//     DELAY_MS(5);
//   }
//   AD_val_1_min=sum/50;
//   sum=0;
//   for(i=0;i<50;i++)                    
//   {
//     AD_val_2 =adc_once(ADC1_SE13, ADC_16bit);
//     sum+=AD_val_2;
//     DELAY_MS(5);
//   }
//   AD_val_2_min=sum/50;
//   sum=0;
//   for(i=0;i<50;i++)                    
//   {
//     AD_val_3 =adc_once(ADC1_SE10, ADC_16bit);
//     sum+=AD_val_3;
//     DELAY_MS(5);
//   }
//   AD_val_3_min=sum/50;
//   sum=0;
//   for(i=0;i<50;i++)                    
//   {
//     AD_val_4 =adc_once(ADC1_SE11, ADC_16bit);
//     sum+=AD_val_4;
//     DELAY_MS(5);
//   }
//   AD_val_4_min=sum/50;
//   sum=0;
//
//   
//   for(i=0;i<150;i++)                   //������е����ֵ
//   {
//     AD_val_1 =adc_once(ADC1_SE8, ADC_16bit);
//     if(AD_val_1>=AD_val_1_max) 
//       AD_val_1_max=AD_val_1;
//     DELAY_MS(1);
//     
//     AD_val_2 =adc_once(ADC1_SE13, ADC_16bit);
//     if(AD_val_2>=AD_val_2_max) 
//       AD_val_2_max=AD_val_2;
//     DELAY_MS(1);
//    	
//     AD_val_3 =adc_once(ADC1_SE10, ADC_16bit);
//     if(AD_val_3>=AD_val_3_max) 
//       AD_val_3_max=AD_val_3;
//     DELAY_MS(1);
//     AD_val_4 =adc_once(ADC1_SE11, ADC_16bit);
//     if(AD_val_4>=AD_val_4_max) 
//       AD_val_4_max=AD_val_4;
//     DELAY_MS(1);
//   }
//  
//  }
  while(1)
  {
      display1= adc_once(ADC1_SE8, ADC_16bit);
      display2= adc_once(ADC1_SE9, ADC_16bit);
      display3= adc_once(ADC1_SE10, ADC_16bit);
      display4= adc_once(ADC1_SE11, ADC_16bit);
      display5= adc_once(ADC1_SE12, ADC_16bit);
      display6= adc_once(ADC1_SE13, ADC_16bit);
      
      if(Stop){
      adc_maxmin_update();                      //���µ�ŵ������Сֵ
      } 
      
           
      
      
      
      
      Check_BottonPress();
      if(new_img)                               //�˶β�����0.5ms 200ms��Ƶ  6ms ��8ms ִ��һ��
      { 
        
        get_edge();
        Search();                       //roadturncal() ��Search(); ���               
       // roadturncal();
        Direction_Control();
        new_img=0;
        
        //Variable_update();                    
        //sendimg();
        //uart0_putValue();                     //���ͱ�������λ��
        
        if(OLED_Refresh)                        //��ʾ
        {  
          img_extract(img,imgbuff_process,CAMERA_SIZE);
          OLED_Draw_UI();
          DELAY_MS(30);
        }

      }

   }
}


void sendimg()                          // ����ͼ����λ��
{ 
  uint16 m,n;
  uint8 colour[2] = {0, 240};           //0 �� 1 �ֱ��Ӧ����ɫ
  //uint8 cmostmp[60];
  for(m=0;m<60;m++)                     //��imgbuff_process�е�0��1(�ںͰ�)ת����0��240���ŵ�cmos[60][80]��
  { for(n=0;n<10;n++)  
    { 
      cmos[m][8*n]   = colour[(imgbuff_process[m*10+n] >> 7 ) & 0x01 ];
      cmos[m][8*n+1] = colour[(imgbuff_process[m*10+n] >> 6 ) & 0x01 ];
      cmos[m][8*n+2] = colour[(imgbuff_process[m*10+n] >> 5 ) & 0x01 ];
      cmos[m][8*n+3] = colour[(imgbuff_process[m*10+n] >> 4 ) & 0x01 ];
      cmos[m][8*n+4] = colour[(imgbuff_process[m*10+n] >> 3 ) & 0x01 ];
      cmos[m][8*n+5] = colour[(imgbuff_process[m*10+n] >> 2 ) & 0x01 ];
      cmos[m][8*n+6] = colour[(imgbuff_process[m*10+n] >> 1 ) & 0x01 ];
      cmos[m][8*n+7] = colour[(imgbuff_process[m*10+n] >> 0 ) & 0x01 ];
    }
  }
  for(m=0;m<60;m++){
    for(n=0;n<80;n++){
      my_putchar(cmos[m][n]);    
    }
  }
  my_putchar(0xff);

}

void PIT_IRQHandler()  //2msһ���ж�
{
   static uint8 flag_100ms,cnt=0;
   // static uint8 flag_obstacle;
   PIT_Flag_Clear(PIT0);                         //���жϱ�־λ    
   if(!Stop)                                    //stop=1��ʾֹͣ
   { 
     RunTime=RunTime+0.002;
     AverageSpeed=Distance/RunTime;
   }
   flag_100ms++;                                //1~51
   // flag_100ms+=20;                            //s 8.5 Ӧ�����󣬸Ļ�����һ�еļ���                             
   // flag_obstacle++;
   // uart_putchar(UART0,'O');
   if(flag_100ms>Speed_Filter_Times)
   {
     flag_100ms=0;  
     Speed_Control();                           //100ms����һ���ٶȿ��������㣨SpeedControlOutOld ��SpeedControlOutNew��
     LED_RED_TURN;
     SpeedCount=0;
     
   }
  /* if(flag_obstacle>1500 && flag==0 && RoadType==50)
   {
    // uart_putchar(UART0,'K');
   //  RoadType=0;
     flag_jump=1;
     flag_obstacle=0;
   }*/
   if(Starting)                                 //Starting=1��ʾ����ǰ׼��״̬
   {
      
      Read_Switch();                            //�����뿪�ص�ֵ
      Start_Cnt--; 
      LED_GREEN_TURN;
      BEEP_ON;
      if(Start_Cnt==0)
      {
        Starting=0;
        Stop=0;                                 //Stop=1��������
        //RoadType=200;                           //s ��ʱ��������Ϊ200������
        RoadType=100;                           //s ���Ե��
        LED_BLUE_OFF;
        LED_GREEN_OFF;
        BEEP_OFF;                               //��������һ��ʱ��ص�
      }
    }
   
   cnt++;
   if(cnt==1)                                   //4ms����һ��
   {
     Get_Speed();                               //��ȡ����
   }
   if(cnt>=2)
   {
     cnt=0;
   }
   SpeedCount++;//1~50  Speed_Control_Output()���ô���   100ms����һ���ٶȿ��������㣨SpeedControlOutOld ��SpeedControlOutNew��������
   Speed_Control_Output();//2msһ�� �����ٶȿ�����PID_SPEED.OUT
   Moto_Out();//2msһ��
}


 