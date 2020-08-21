/*
��С�˿�PID_TURN.OUT����Сֵ

*/
#include "include.h"

//�ٶ������
uint8 Style=1;                                  //1�����٣�0����
float SpeedControlOutNew;
float SpeedControlOutOld;
float SpeedControlIntegral=0,Hill_Slow_Ratio;
uint8  Set_Angle;                               //����ǰ��Ƕ�
int   SpeedCount;
int   Speed_Filter_Times=50;                    //�ٶ�ƽ�����
float CarSpeed=0,ControlSpeed=0,AverageSpeed,SetSpeed=0,Distance=0;
float Speed_H=0,Speed_M=0,Speed_L=0;
int   Stop_Brake=0;                               //ɲ��
int   wycnt=0;
float acceleration;
int   NitroBooster=0;
int   NitroBoostercount=0;
//���������
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed=0;
int   DirectionCount;
float Delt_error,Middle_Err;
float Turn_Out;
float Turn_Angle_Integral;

/**������**/
int sever_middle=140;                   //ֵԽ��Խƫ��  
int sever_range=35;                     //19(ʵ�ʷ�Χ)//25(ԭ)//28  //s 

//ģ����ϵ��
float  Delta_P;
float  Delta_D;
float  Fuzzy_Kp;
float  Fuzzy_Kd;
//PID���������
PID PID_SPEED,PID_TURN;
float Kp,Ki;
float Last_SpeedError2;
float Last_SpeedError1;



float  MotorOut;                                //��������          
uint8   Starting,Stop;                          //stop=1ʹ����������Starting=1����׼��״̬
uint8 Encoder_Disable=0;

extern int16 GYRO_OFFSET_Z;
extern int16 leftspeed,rightspeed;
//extern int16 disspeed;
extern uint8 lost_line;
extern int flag,flag_obstacle;

float Distance1=10000;
uint8 flag2=0,flag_100;

// ����жϻ�������
float   AD_Value[2][5],AD_Value1[2],AD_V[2][5];
float AD_val_1,AD_val_2,AD_val_3,AD_val_4,AD_val_5,AD_val_6;
float dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4,dis_AD_val_5,dis_AD_val_6;
float disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4,disgy_AD_val_5,disgy_AD_val_6;
float AD_val_1_max=20000;
float AD_val_2_max=20000;
float AD_val_3_max=20000;
float AD_val_4_max=20000;         //���������ȷ��м��һ����40��
float AD_val_5_max=20000;
float AD_val_6_max=20000;
float AD_val_1_min=100000;
float AD_val_2_min=100000;
float AD_val_3_min=100000;
float AD_val_4_min=100000;
float AD_val_5_min=100000;
float AD_val_6_min=100000;

int flag_cricle_right=0,flag_cricle_left=0;         //��Ÿ�Ӧ����
int Go_Out_Circle=0;
int circle_Flag;
float Distance70=1000;
int turn_Flag;
int turn_Flag2=0;
int turn_right_Flag=0;
int turn_left_Flag=0;

float adc_ave();


void Get_Speed()      //������4msִ��һ��     //5msִ��һ��
{  
  int qd1_result;
  //int Car;                    //      ��δ�õ�
  //qd1_result =- FTM_QUAD_get(FTM2); 
  //FTM_QUAD_clean(FTM2);
  //disspeed = leftspeed; //+ rightspeed;
  qd1_result = leftspeed;// + rightspeed;
  
  leftspeed=0;
  //rightspeed=0;
  
  Distance+=qd1_result/2500.0;                                  //ת��Ϊ4ms���ܹ��ľ��� 500�ߣ�һȦ20cm
  //CarSpeed=CarSpeed*0.1+0.9*qd1_result*250.0/3100;            //�������ת��ΪM/S 4msһ��
  CarSpeed=CarSpeed*0.1+0.9*qd1_result*0.01; //*250/2500;       //�������ת��ΪM/S
  //disspeed= CarSpeed;
  if(CarSpeed>=4)CarSpeed=4; 
  
}
//�ٶȿ���������  100msһ��
void Speed_Control(void)                        //����SpeedControlOutOld  ����SpeedControlOutNew
{  
  static float PreError[20]={0};
  float  SpeedError;
  uint8 i;
  
  //SpeedError=0.42-CarSpeed;//����
  //������С���б��ٹ���
  if(Style==0)//�����٣�1
    SpeedError=SetSpeed-CarSpeed;               //����ƫС ���Ϊ��
  else{//����  
    if(RoadType==6 || RoadType==7 || RoadType==16 || RoadType==32 || RoadType==33  || RoadType==36 || RoadType==37) //50
    {
      
      if(CarSpeed>0.7)                          //�趨ֵΪ0.58ʱ������ͨ��С��0.4  0.385
      {  
        SpeedError=0.2-CarSpeed;                //����

      }
      else
      {
        SpeedError=Speed_L-CarSpeed;                //����

      }
    }
//    else if(RoadType==40 || RoadType==41 || RoadType==18)//0�ǿ���
//    {                                           //�ϰ���  ����
//      if(CarSpeed>0.5)                          //�趨ֵΪ0.7ʱ������ͨ��С��0.54
//      {  
//        SpeedError=0.2-CarSpeed;                //����ʱ����
//
//      }
//      else
//      {
//        SpeedError=Speed_M-CarSpeed;            //����
//
//      }
//    }
    else if(RoadType==3 || RoadType==4 || RoadType==5 || RoadType==13 || RoadType==14  || RoadType==15) //18
    {
      SpeedError=Speed_M-CarSpeed;                 //����
     // SpeedError=0.15-CarSpeed; 
     // SpeedError=0; 
     // SetSpeed=0;
    }
//    else if(RoadType==100)
//    {
//    SpeedError=Speed_M-CarSpeed;
//    //SpeedError=0.60-CarSpeed;                 //����
//    
//    }
//    else if(RoadType==200)                               //
//    {
//      SpeedError=0.4-CarSpeed;
//    }

    
    else{                                       //RoadType==1,2,7,12,17,18
      SpeedError=Speed_H-CarSpeed; //����
    }
  }

 acceleration=SpeedError;                      //��debug�в鿴
 


 // ������20��ƫ����ܺ���Ϊ������
  SpeedControlIntegral=0;
  for(i=0;i<19;i++)
  {
     PreError[i]=PreError[i+1]; 
     SpeedControlIntegral+=PreError[i];
  }
  PreError[19]=SpeedError;
  SpeedControlIntegral+=PreError[19];
  //�ٶȸ���
  SpeedControlOutOld=SpeedControlOutNew;
  if(RoadType==200){
    SpeedControlOutNew=0.2*SpeedError+0.02*SpeedControlIntegral;
  }
  else{
    SpeedControlOutNew=PID_SPEED.P*SpeedError+PID_SPEED.I*SpeedControlIntegral;   //PI���� ��CarSpeed��������SetSpeed�������˱���Ϊ���ұ�С  ��CarSpeed��С����SetSpeed�������˱���Ϊ���Ҿ���ֵ��С
  }

//  SpeedControlOutOld=SpeedControlOutNew;
//  SpeedControlOutNew=Kp*(SpeedError-Last_SpeedError1)+Ki*SpeedError+SpeedControlOutOld;   //
//  if(SpeedControlOutNew>0.4){
//    SpeedControlOutNew=0.4;
//  }
//  if(SpeedControlOutNew<-0.3){
//    SpeedControlOutNew<-0.3;
//  }
//  Last_SpeedError2=Last_SpeedError1;
//  Last_SpeedError1=SpeedError;


 
}
//�����ٶȿ�����PID_SPEED.OUT
void Speed_Control_Output(void)                         //2msһ�� ÿ�ε���ǰSpeedCount��1 SpeedCount����[1,50]
{ 
  
  float fValue; 
  fValue = SpeedControlOutNew - SpeedControlOutOld; 
  PID_SPEED.OUT = fValue * (SpeedCount+1)/Speed_Filter_Times+SpeedControlOutOld; 
  //PID_SPEED.OUT = SpeedControlOutNew; 
 // ���������Ϊ�㣬PID_SPEED.OUT=0��ʹ��ռ�ձ�Ϊ��
}
/********************�������������***************/
void Direction_Control(void)
{
  //static int Calculate_Length=0;        //��δ�õ�
  Turn_Speed= -0.01*(Get_Z_Gyro() - GYRO_OFFSET_Z);     //0.01//GYRO_OFFSET_Z��̬Z����ٶ�ֵ����I2C�ж�Ϊ17
  if(Turn_Speed<10&&Turn_Speed>-10)
  {
    Turn_Speed=0;
  }
  
  
    Fuzzy(Middle_Err,Delt_error);                       //�õ�ģ����ϵ��float  Delta_P;float  Delta_D;
    Delta_P=Delta_P*Fuzzy_Kp;                           //Fuzzy_Kp Fuzzy_Kd��founction.c�еĲ�����ʼ�������趨ֵ
    Delta_D=Delta_D*Fuzzy_Kd;

  PID_TURN.pout=(PID_TURN.P+Delta_P)*Middle_Err;        //Middle_Err
  PID_TURN.dout=(PID_TURN.D+Delta_D)*Turn_Speed*0.1;
  Turn_Out= PID_TURN.pout - PID_TURN.dout;
  
  Turn_Out=Turn_Out_Filter(Turn_Out);                   //ת������˲� 
  
  PID_TURN.OUT=Turn_Out*100;

  if( PID_TURN.OUT>sever_range+2) {
    PID_TURN.OUT=sever_range+2;
    //my_putchar('R');
  }
  if( PID_TURN.OUT<-sever_range) {
    PID_TURN.OUT=-sever_range;
    //my_putchar('L');
  }
  if(lost_line==0)
  {
    FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle + PID_TURN.OUT);
   // uart_putchar(UART0,'X');
  }
//  else
//  {
//    if(lost_line==1)
//    {
//       FTM_PWM_Duty(FTM1,FTM_CH0,270);    //right  139
//       uart_putchar(UART0,'Y');
//    }
//    else if(lost_line==2)
//    {
//       FTM_PWM_Duty(FTM1,FTM_CH0,190);    //left  85
//       uart_putchar(UART0,'Z');
//    }
//  }
 
}




//���pwmֵ���
void Moto_Out() //2msһ��
{
 //�ٶȿ�������޷�
 if(PID_SPEED.OUT>0.5)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
 PID_SPEED.OUT=0.5;
 if(PID_SPEED.OUT<-0.2)
 PID_SPEED.OUT=-0.2;
//if(PID_SPEED.OUT>0.4)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
// PID_SPEED.OUT=0.4;
//if(PID_SPEED.OUT<-0.4)
// PID_SPEED.OUT=-0.4;
 MotorOut=PID_SPEED.OUT;
// 
 
 //��ֵ�޷�����ֹ���ٹ���
 
  
  if(MotorOut>0.99)MotorOut=0.99;                     
  if(MotorOut<-0.99)MotorOut=-0.99; 
 

 if(Stop)                                //���ֹͣ�����������ռ�ձ�Ϊ��
 {
    MotorOut=0;
    LED_BLUE_ON;
 }
 if(Stop_Brake==1 && wycnt<=350){                //�յ�ɲ�����ֵ�תʱ��
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);
     
   FTM_PWM_Duty(FTM0,FTM_CH1,1200);
  
   wycnt++;                      
  
 }
  if(wycnt>350){ 
    
   FTM_PWM_Duty(FTM0,FTM_CH0,0);
   FTM_PWM_Duty(FTM0,FTM_CH1,0);
   Stop_Brake=0;
   wycnt=0;
                        
 }

 
 if(!Stop_Brake){
//int   NitroBooster=0;
//int   NitroBoostercount=0;
if(NitroBooster==1 && NitroBoostercount<=250 ){
  FTM_PWM_Duty(FTM0,FTM_CH0,3500);
  FTM_PWM_Duty(FTM0,FTM_CH1,0);
  NitroBoostercount++;
  
}
if(NitroBoostercount>250){
   FTM_PWM_Duty(FTM0,FTM_CH0,0);
   FTM_PWM_Duty(FTM0,FTM_CH1,0);
   NitroBooster=0;
   NitroBoostercount=0;
}
if(!NitroBooster){
   if(MotorOut>=0) //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,MotorOut*10000);//ռ�ձȾ���Ϊ10000  ����������Ϊռ�ձȷ���
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
     //if(MotorOut>0) my_putchar('A');
     ///else my_putchar('a');
  }
  else   //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH1,-MotorOut*10000);
     //my_putchar('B');
  }
}
 }
//   FTM_PWM_Duty(FTM0,FTM_CH0,700);
//    FTM_PWM_Duty(FTM0,FTM_CH1,0);
}
 

float  Turn_Out_Filter(float turn_out)    //ת���������˲�      
{
  float Turn_Out_Filtered; 
  static float Pre1_Error[4]; 
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.3+Pre1_Error[1]*0.2+Pre1_Error[2]*0.2+Pre1_Error[3]*0.2;
  return Turn_Out_Filtered;
}
float  Middle_Err_Filter(float middle_err)    //����ƫ���˲�    ��δʹ��
{
  float Middle_Err_Fltered; 
  static float Pre3_Error[4]; 
  Pre3_Error[3]=Pre3_Error[2];
  Pre3_Error[2]=Pre3_Error[1];
  Pre3_Error[1]=Pre3_Error[0];
  Pre3_Error[0]=middle_err;
  Middle_Err_Fltered=Pre3_Error[0]*0.4+Pre3_Error[1]*0.3+Pre3_Error[2]*0.2+Pre3_Error[3]*0.1;
  return Middle_Err_Fltered;
}

//��Ų���
void roadturncal()                              //��ż�⻷��  
{ //static uint8 turn_Flag2;                     //��δ�õ�
  uint16 i,j,k; 
  float temp,ad_sum[2],AD_sum[2]; 
  float Inductor_ADC[6];
  float err;
// ���ADֵ�˲����򣬱Ƚ��� ���ײ�ɼ������Զ����зǸ�����
  for(i=0;i<5;i++)
  {
     AD_Value[0][i] = adc_ave(ADC1_SE12, ADC_16bit,2);  //��� 
     AD_Value[1][i] = adc_ave(ADC1_SE9, ADC_16bit,2);  //�Ҷ� 
  }

  
    AD_val_3 = adc_ave(ADC1_SE13, ADC_16bit,2);  //���� 
    AD_val_4 = adc_ave(ADC1_SE8, ADC_16bit,2); //���� 
    AD_val_5 = adc_ave(ADC1_SE11, ADC_16bit,2);  //����
    AD_val_6 = adc_ave(ADC1_SE10, ADC_16bit,2); //����
 
  for(i=0;i<2;i++)     //���ҵ��  ��С��������  
  {
       for(j=0;j<4;j++)  
       {
          for(k=0;k<4-j;k++)
          {
             if(AD_Value[i][k] > AD_Value[i][k+1])  //ǰ��ıȺ���Ĵ�  ����н���
             {
                temp = AD_Value[i][k+1];
                AD_Value[i][k+1] = AD_Value[i][k];
                AD_Value[i][k] = temp;
             }
          }
       }
  }
  for(i=0;i<2;i++)    //���м�����ĺ�
    {
       ad_sum[i] = AD_Value[i][1] + AD_Value[i][2] + AD_Value[i][3];       
       AD_Value1[i] =ad_sum[i] / 3;
    }
  
  for(i = 0;i < 4;i++)
    {
        AD_V[0][i] = AD_V[0][i + 1];
        AD_V[1][i] = AD_V[1][i + 1];
    }
    for(i=0;i<2;i++)
    {
        AD_V[i][4] =  AD_Value1[i];
    }
    AD_sum[0]=0;
    AD_sum[1]=0;
    for(i = 0;i < 5;i ++)
    {
        AD_sum[0] += AD_V[0][i];
        AD_sum[1] += AD_V[1][i];
    }
  AD_val_1=AD_sum[0]/5;  //����
  AD_val_2=AD_sum[1]/5;  //�ҵ��
  
  dis_AD_val_1=AD_val_1;  //���
  dis_AD_val_2=AD_val_2;  //�Ҷ�
  dis_AD_val_3=AD_val_3;  //����
  dis_AD_val_4=AD_val_4;  //���� 
  dis_AD_val_5=AD_val_5;  //����
  dis_AD_val_6=AD_val_6;  // ����
  
  
  //�޷�
//  if(AD_val_1>AD_val_1_max)		AD_val_1=AD_val_1_max;
//  if(AD_val_2>AD_val_2_max)		AD_val_2=AD_val_2_max;
//  if(AD_val_3>AD_val_3_max)		AD_val_3=AD_val_3_max;
//  if(AD_val_4>AD_val_4_max)		AD_val_4=AD_val_4_max;
//  if(AD_val_5>AD_val_5_max)		AD_val_5=AD_val_5_max;
//  if(AD_val_6>AD_val_6_max)		AD_val_6=AD_val_6_max;
//  
//  if(AD_val_1<AD_val_1_min)		AD_val_1=AD_val_1_min;
//  if(AD_val_2<AD_val_2_min)		AD_val_2=AD_val_2_min;
//  if(AD_val_3<AD_val_3_min)		AD_val_3=AD_val_3_min;
//  if(AD_val_4<AD_val_4_min)		AD_val_4=AD_val_4_min;
//  if(AD_val_5<AD_val_6_min)		AD_val_5=AD_val_5_min;
//  if(AD_val_5<AD_val_6_min)		AD_val_6=AD_val_6_min;
//  
//  
  
  //��һ��
  AD_val_1=(100*(AD_val_1 -AD_val_1_min))/(AD_val_1_max-AD_val_1_min);
  AD_val_2=(100*(AD_val_2 -AD_val_2_min))/(AD_val_2_max-AD_val_2_min);
  AD_val_3=(100*(AD_val_3 -AD_val_3_min))/(AD_val_3_max-AD_val_3_min);
  AD_val_4=(100*(AD_val_4 -AD_val_4_min))/(AD_val_4_max-AD_val_4_min);
  AD_val_5=(100*(AD_val_5 -AD_val_5_min))/(AD_val_5_max-AD_val_5_min);
  AD_val_6=(100*(AD_val_6 -AD_val_6_min))/(AD_val_6_max-AD_val_6_min);
 
  disgy_AD_val_1 = AD_val_1;
  disgy_AD_val_2 = AD_val_2;
  disgy_AD_val_3 = AD_val_3;
  disgy_AD_val_4 = AD_val_4;
  disgy_AD_val_5 = AD_val_5;
  disgy_AD_val_6 = AD_val_6;
    
 

//     �ĵ�з��ж�Բ��   ����� 
  Inductor_ADC[0]= disgy_AD_val_1;              //���
  Inductor_ADC[1]= disgy_AD_val_2;              //�Ҷ�
  Inductor_ADC[2]= disgy_AD_val_3;              //����
  Inductor_ADC[3]= disgy_AD_val_4;              //����
  Inductor_ADC[4]= disgy_AD_val_5;              //����
  Inductor_ADC[5]= disgy_AD_val_6;              //����
//  if((Inductor_ADC[0]+Inductor_ADC[1])>20)                      //�е���źţ�û�ж���  

  
  if(Go_Out_Circle==0 && circle_Flag==0 &&
     ((Inductor_ADC[1]- Inductor_ADC[0]>45 && Inductor_ADC[3] - Inductor_ADC[2]>45 && Inductor_ADC[1]>55 && Inductor_ADC[3]>55) ||
      (Inductor_ADC[0]- Inductor_ADC[1]>45 && Inductor_ADC[2] - Inductor_ADC[3]>45 && Inductor_ADC[0]>55 && Inductor_ADC[2]>55)))
  {
     circle_Flag=1;  //ǰ�Կ�����Բ��
     if(Inductor_ADC[2]>Inductor_ADC[3])  //��Բ��         //Inductor_ADC[0]>Inductor_ADC[1]&&
     {
        turn_left_Flag=1;
        //flag_cricle_left=1;                           //���ݸ�search��ʶ�𵽻���
     }
     if(Inductor_ADC[2]<Inductor_ADC[3])  //��Բ��         //Inductor_ADC[1]>Inductor_ADC[0]&&
     {
        turn_right_Flag=1;
        //flag_cricle_right=1;
     }
     if(turn_right_Flag==0&&turn_left_Flag==0) //����
     {
        circle_Flag=0;
     }
     Distance70=Distance;
  }
  if(Distance-Distance70 <1 && Inductor_ADC[4]+Inductor_ADC[5]>145 ){
 
    if(turn_left_Flag==1){
      flag_cricle_left=1;                           //���ݸ�search��ʶ�𵽻���
    }
    if(turn_right_Flag==1){
      flag_cricle_right=1;
    }
  }
  
  if(Distance-Distance70>1 &&(flag_cricle_left!=1 || flag_cricle_right!=1)){
      circle_Flag=0;
      turn_left_Flag=0;
      turn_right_Flag=0;
      Distance70=1000;
  }
  
  
}
// err=Inductor_ADC[3]-Inductor_ADC[2]; //���ֵ�о�����Բ��
//  if(ABS(err)<70&&circle_Flag==1&&turn_Flag==0 && Inductor_ADC[0]>20 && Inductor_ADC[1]>20)
//  {
//     turn_Flag=1;                               //����Բ��
//     if(turn_right_Flag==1)
//     {
//
//     }
//     if(turn_left_Flag==1){
// 
//     }
//  }
  
//    Go_Out_Circle=1;
//    
//
//  
//  
//
//  if(Go_Out_Circle==1) //���һ��Բ������
//  {
//     turn_Flag=0; 
//     turn_Flag2=0;
//     turn_right_Flag=0;
//     turn_left_Flag=0;
//     circle_Flag=0;
//     Go_Out_Circle=0;
//  }

  
    
 // }