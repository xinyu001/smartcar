#include "common.h"
#include "include.h"

        //�����жϸ�λ����
int16 leftspeed,rightspeed;
//void PORTC_IRQHandler();
void PORTD_IRQHandler();
void DMA0_IRQHandler();
void PIT_IRQHandler();
void FTM2_IN_IRQHandler();
void FTM_IN_Init();
void AD_Init();

extern float Control_Para[15];
extern uint8 Style;                             //7��7�ռ�

//s  ��Ų���
extern int   AD_val_1;
extern int   AD_val_2;
extern int   AD_val_3;
extern int   AD_val_4;
int  adtmp1,adtmp2,adtmp3,adtmp4;
extern int   dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4  ;
extern int   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4;
extern int   AD_val_1_min;
extern int   AD_val_2_min;
extern int   AD_val_3_min;
extern int   AD_val_4_min;
extern int   AD_val_1_max;
extern int   AD_val_2_max;
extern int   AD_val_3_max; 
extern int   AD_val_4_max; 

void init()
{  
   AD_Init();
   OLED_Init();
   button_init(); 
   switch_init();
   BEEP_ON;
   DELAY_MS(200);
   BEEP_OFF;
   //OLED_Draw_Logo();
   led_init(); 
   led_flash(); 
   I2C_Init();
   adc_init (ADC1_SE6a);                // ��ص�ѹ�����ӿ� װ������Ϊ3.3/65535*3.128   (4.7k + 10k )/4.7k
   pit_init_ms(PIT0,2);                 //2ms��ʱ�ж�
   set_vector_handler(PIT0_VECTORn ,PIT_IRQHandler);

   
   //ͳ��ִ�д�����PIT1_IRQHandler()
   //pit_init_ms(PIT1,2); //����ms��ʱ�ж�
   //set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);
   
   Para_Init();                                          //ֱ�ӳ�ʼ������
   EnableInterrupts;                                     //���ж�  
   //EEPROM_init();     
                                                         //�����PWM Ƶ��Ϊ10khz ռ�ձȾ���Ϊ10000 
   FTM_PWM_init(FTM1,FTM_CH0,100,sever_middle);          //��� PWM  PTA8�����Ƶ��Ϊ100hz  ռ�ձ�Ϊserve_middle/ĳֵ 
    
   FTM_PWM_init(FTM0,FTM_CH0,10000,0);                   //��� PWM Ƶ��10000Hz
   FTM_PWM_init(FTM0,FTM_CH1,10000,0);                   //��� PWM Ƶ��10000Hz
   //FTM_QUAD_Init(FTM2);                                //���������ʼ�� 
   FTM_IN_Init();                                        //��ʼ��FTM2Ϊ����
   OLED_CLS(); 
   camera_init();
   set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);    
   set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler); 
   set_vector_handler(FTM2_VECTORn ,FTM2_IN_IRQHandler); 
   enable_irq(FTM2_IRQn);                               //FTM2�ж�
   enable_irq(PORTD_IRQn); 
   OLED_CLS();
   set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_IRQHandler);
   uart_rx_irq_en(UART0);
   NVIC_SetPriority(UART0_RX_TX_IRQn,0);
   NVIC_SetPriority(DMA0_VECTORn,1);
   NVIC_SetPriority(PORTD_VECTORn ,2); 
   enable_irq (PIT0_IRQn); 
   //enable_irq (PIT1_IRQn);                            //���Լ��ӵģ�PIT timer channel 1 interrupt
}

void Para_Init()
{
  Speed_H=0.75;                                 //0.75
  Speed_M=0.65;                                 //0.65
  Speed_L=0.52;                                 //0.52
  if(Style==0)
    SetSpeed=0.5;                              //0.9  ԭ��0.6
  if(Style==1)
    SetSpeed=0.4;                               //0.6//s 0.5
  Fuzzy_Kp=0.0065;                              //0.014
  Fuzzy_Kd=0.005;                               //0
  PID_SPEED.P=0.13;                             //0.13  
  PID_SPEED.I=0.02;                             //0.02
  PID_TURN.P=0.006;                             //0.006
  PID_TURN.D=0.0028;                            //0.0028
  Set_Angle=50;
  
  Control_Para[0]= SetSpeed;                    //Set_SPEED
  Control_Para[1]= Fuzzy_Kp;                    //FUZZY.P
  Control_Para[2]= Fuzzy_Kd;                    //FUZZY.D
  Control_Para[3]= PID_SPEED.P;                 //PID_SPEED.P
  Control_Para[4]= PID_SPEED.I;                 //PID_SPEED.I
  Control_Para[5]= PID_TURN.P;                  //PID_DIREC.P
  Control_Para[6]= PID_TURN.D;                  //PID_DIREC.D

}

void FTM_IN_Init()
{  SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;
   SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
   PORTA_PCR10 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;                           //PA10
   FTM2_C0SC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);                      //�����ش���
   FTM2_C0SC &= ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);
   
//   PORTA_PCR11 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;                         //PA11
//   FTM2_C1SC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);                    //�����ش���
//   FTM2_C1SC &= ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);
   gpio_init(PTA11,GPI,0);                                                      //����
   
   FTM2_SC = FTM_SC_CLKS(1);
   FTM2_MODE |= FTM_MODE_WPDIS_MASK;
   FTM2_COMBINE = 0;
   FTM2_MODE &= ~FTM_MODE_FTMEN_MASK;
   FTM2_CNTIN = 0;
   FTM2_STATUS = 0x00;                                          //���жϱ�־λ
   //enable_irq(64);                                            //FTM2�ж�
}


float Slope_Calculate(uint8 begin,uint8 end,float *p)           //��С���˷����б��
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum)                       //�жϳ����Ƿ�Ϊ�� 
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

void FTM2_IN_IRQHandler()
{  unsigned char curstatus = FTM2_STATUS;
   FTM2_STATUS = 0x00;                                  //���ж�
   if(curstatus&(1<<0))                                 //CH0�ж�,�����
   {  if(gpio_get(PTA11)==1) leftspeed++;               //ԭ����PTA11   ԭ����==0
      else   leftspeed--;
   }
//   if(curstatus&(1<<1))                               //CH1�жϣ��Ҳ���
//   { rightspeed++;
//   }
}

 void AD_Init()
{
    adc_init(ADC1_SE7a);                                //������  PTE3
    //���������һ����ɺ���ʵ��ȷ��
    adc_init(ADC1_SE8);                                 //���  PTB0   
    adc_init(ADC1_SE9);                                 //���  PTB1   
    adc_init(ADC1_SE10);                                //���  PTB4  
    adc_init(ADC1_SE11);                                //���  PTB5  
    adc_init(ADC1_SE12);                                //���  PTB6  
    adc_init(ADC1_SE13);                                //���  PTB7 

}

float adc_ave(ADCn_Ch_e adcn_ch, ADC_nbit bit, int N) //��ֵ�˲�
{
    float tmp = 0;
    int  i;
    for(i = 0; i < N; i++)
        tmp += adc_once(adcn_ch, bit);
    tmp = tmp / N;
    return (float)tmp;
}

void adc_maxmin_update(){  
//  //��ʾ��ǰ��й�һ��ֵ���鿴��һ��ֵ�Ƿ���ȷ���ȴ����IPE5���º�С������
      adtmp1=0;
      adtmp2=0;
      adtmp3=0;
      adtmp4=0;
      int i;
      for(i=0;i<10;i++)
      {
        AD_val_1 = adc_ave(ADC1_SE8, ADC_16bit,8);
        AD_val_2 = adc_ave(ADC1_SE13, ADC_16bit,8); 
        AD_val_3 = adc_ave(ADC1_SE10, ADC_16bit,8);
        AD_val_4 = adc_ave(ADC1_SE11, ADC_16bit,8);
        adtmp1= adtmp1+ AD_val_1;
        adtmp2= adtmp2+ AD_val_2;
        adtmp3= adtmp3+ AD_val_3;
        adtmp4= adtmp4+ AD_val_4;
      } 
      AD_val_1 = adtmp1/10;
      AD_val_2 = adtmp2/10;
      AD_val_3 = adtmp3/10;
      AD_val_4 = adtmp4/10;
      //���������Сֵ
      if(AD_val_1>AD_val_1_max)		AD_val_1_max=AD_val_1;
      if(AD_val_2>AD_val_2_max)		AD_val_2_max=AD_val_2;
      if(AD_val_3>AD_val_3_max)		AD_val_3_max=AD_val_3;
      if(AD_val_4>AD_val_4_max)		AD_val_4_max=AD_val_4;
			
      if(AD_val_1<AD_val_1_min)		AD_val_1_min=AD_val_1;
      if(AD_val_2<AD_val_2_min)		AD_val_2_min=AD_val_2;			  
      if(AD_val_3<AD_val_3_min)		AD_val_3_min=AD_val_3;
      if(AD_val_4<AD_val_4_min)		AD_val_4_min=AD_val_4;	
       //��һ��
//      AD_val_1=100*(AD_val_1 - AD_val_1_min)/(AD_val_1_max-AD_val_1_min);
//      AD_val_2=100*(AD_val_2 - AD_val_2_min)/(AD_val_2_max-AD_val_2_min);
//      AD_val_3=100*(AD_val_3 - AD_val_3_min)/(AD_val_3_max-AD_val_3_min);
//      AD_val_4=100*(AD_val_4 - AD_val_4_min)/(AD_val_4_max-AD_val_4_min);
//      disgy_AD_val_1 = AD_val_1;
//      disgy_AD_val_2 = AD_val_2;
//      disgy_AD_val_3 = AD_val_3;
//      disgy_AD_val_4 = AD_val_4;
}


