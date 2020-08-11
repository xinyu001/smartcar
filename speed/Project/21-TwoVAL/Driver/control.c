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
//���������
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed=0;
int   DirectionCount;
float Delt_error,Middle_Err;
float Turn_Out;
float Turn_Angle_Integral;

/**������**/
int sever_middle=225;                   //ֵԽ��Խƫ�� 225ganghao 
int sever_range=30;                     //19(ʵ�ʷ�Χ)//25(ԭ)//28  //s 55

//ģ����ϵ��
float  Delta_P;
float  Delta_D;
float  Fuzzy_Kp;
float  Fuzzy_Kd;
//PID���������
PID PID_SPEED,PID_TURN;

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

void Get_Speed()      //������4msִ��һ��     //5msִ��һ��
{  
  int qd1_result;
  int Car;
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
   /*  if(flag==1) //18
    {
      uart_putchar(UART0,'J'); 
      //uart_putchar(UART0,'Z');
       //SpeedError=0.2-CarSpeed; //����
       SpeedError=0.2-CarSpeed;
    }*/
    if(RoadType==6 || RoadType==7 || RoadType==16 || RoadType==32 || RoadType==33  || RoadType==36 || RoadType==37) //50
    {
      
      if(CarSpeed>0.7)                          //�趨ֵΪ0.58ʱ������ͨ��С��0.4  0.385
      {  
        SpeedError=0.2-CarSpeed;                //����

      }
      else
      {
        SpeedError=0.7-CarSpeed;                //����

      }
    }
    else if(RoadType==40 || RoadType==41 || RoadType==18)//0�ǿ���
    {                                           //�ϰ���  ����
      if(CarSpeed>0.5)                          //�趨ֵΪ0.7ʱ������ͨ��С��0.54
      {  
        SpeedError=0.2-CarSpeed;                //����ʱ����

      }
      else
      {
        SpeedError=Speed_M-CarSpeed;            //����

      }
    }
    else if(RoadType==3 || RoadType==4 || RoadType==5 || RoadType==13 || RoadType==14  || RoadType==15) //18
    {
      SpeedError=0.65-CarSpeed;                 //����
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
//    else if(RoadType==50 && flag==0)            //18
//    {
//    
//     SpeedError=0.45-CarSpeed;                  //0.35
//     }
//    else if(RoadType==50 && flag==1)            //18
//    {
//    
//     SpeedError=0.60-CarSpeed;                  //SetSpeed
//     }
//    else if(RoadType==200)
//    {
//      
//    SpeedError=Speed_L-CarSpeed;                //�� ����
//    
//    }

    
    else{                                       //RoadType==1,2,7,12,17,18
      SpeedError=Speed_M-CarSpeed; //��
    }
  }

 
  

  //������20��ƫ����ܺ���Ϊ������
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

  SpeedControlOutNew=PID_SPEED.P*SpeedError+PID_SPEED.I*SpeedControlIntegral;   //PI���� ��CarSpeed��������SetSpeed�������˱���Ϊ���ұ�С  ��CarSpeed��С����SetSpeed�������˱���Ϊ���Ҿ���ֵ��С
}
//�����ٶȿ�����PID_SPEED.OUT
void Speed_Control_Output(void)                         //2msһ�� ÿ�ε���ǰSpeedCount��1 SpeedCount����[1,50]
{ 
  float fValue; 
  fValue = SpeedControlOutNew - SpeedControlOutOld; 
  PID_SPEED.OUT = fValue * (SpeedCount+1)/Speed_Filter_Times+SpeedControlOutOld; 
  //���������Ϊ�㣬PID_SPEED.OUT=0��ʹ��ռ�ձ�Ϊ��
}
/********************�������������***************/
void Direction_Control(void)
{
  static int Calculate_Length=0;
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
  Turn_Out= PID_TURN.pout + PID_TURN.dout;
  
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
  else
  {
    if(lost_line==1)
    {
       FTM_PWM_Duty(FTM1,FTM_CH0,270);    //right  139
       uart_putchar(UART0,'Y');
    }
    else if(lost_line==2)
    {
       FTM_PWM_Duty(FTM1,FTM_CH0,190);    //left  85
       uart_putchar(UART0,'Z');
    }
  }
 /* if(RoadType==100)
  {
    if(Display2<45000 || Display1>Display2)
      {
          FTM_PWM_Duty(FTM1,FTM_CH0,190);
      }
    else if(Display1<57200)
      {
          if(Display2-Display1>500)
            {
                FTM_PWM_Duty(FTM1,FTM_CH0,250);
            }
      }
 else if(Display1>57300)
     {
         if(Display2-Display1<300)
          {
            FTM_PWM_Duty(FTM1,FTM_CH0,190);
          } 
      }
  }*/
  
/*if(RoadType==100)
{
 // FTM_PWM_Duty(FTM1,FTM_CH0,250);
  if(ABS(Display1-Display2)<5000)
 {FTM_PWM_Duty(FTM1,FTM_CH0,230);}
  else if(Display2-Display1>8000)
  {
    FTM_PWM_Duty(FTM1,FTM_CH0,250);
  if(Display1<10000)
  {SetSpeed=0.30;}
  }
  else if(Display1-Display2>10000)
  {FTM_PWM_Duty(FTM1,FTM_CH0,190);}
}*/
  
    if(RoadType==50 && flag==1)
  //if(flag==1)
     {
     //  flag2=1;
    //   SetSpeed=0.3;
      if(vol0>15000)
      {

       FTM_PWM_Duty(FTM1,FTM_CH0,190);   //190��270����
     //  RoadType=50;
       if(vol0>16000 && vol0<18000) //25500
       {Distance1=Distance;
       //flag_100=1;
       uart_putchar(UART0,'X');
       }
      }
     }
       if(RoadType==50 && flag==1)
     // if(flag==1)
     {
       if((Distance-Distance1)<=0.9 && (Distance-Distance1)>0)  //1.8
        { FTM_PWM_Duty(FTM1,FTM_CH0,190);   
        uart_putchar(UART0,'A');
        }
        else if((Distance-Distance1)>0.9 && (Distance-Distance1)<=1.7)
        { FTM_PWM_Duty(FTM1,FTM_CH0,250); 
        uart_putchar(UART0,'B');
        }
        else if ((Distance-Distance1)>1.7 && (Distance-Distance1)<=2.5)
        { FTM_PWM_Duty(FTM1,FTM_CH0,230);
       uart_putchar(UART0,'C'); 
        }
        else if ((Distance-Distance1)>2.5 && (Distance-Distance1)<=3.3)
        { FTM_PWM_Duty(FTM1,FTM_CH0,250);
        uart_putchar(UART0,'D'); 
        }
        else if ((Distance-Distance1)>3.3 && (Distance-Distance1)<=3.9)
        { FTM_PWM_Duty(FTM1,FTM_CH0,190); 
       uart_putchar(UART0,'E'); 
        if((Distance-Distance1)>3.8)
        {RoadType=0;
       // flag=0;
        flag_obstacle=0;}
        }
    /*    else if((Distance-Distance1)>4.2)
        { RoadType=0;
        flag=0;
        Style=1;
        uart_putchar(UART0,'F'); 
       // flag2=0;
        }*/
       /* else
        {
          RoadType=0;
          flag=0;
          uart_putchar(UART0,'G'); 
          FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle + PID_TURN.OUT);   //�����ֵ��0-180 �����仯
        } */
  //   }
    }
     /* if(RoadType==50 && flag==1)
      { 
       if((Distance-Distance1)>5)
        {
          uart_putchar(UART0,'G'); 
          RoadType=0;
          flag=0;
        }
      }*/
if(RoadType==100)
{
 // FTM_PWM_Duty(FTM1,FTM_CH0,250);
  if(ABS(Display1-Display2)<2000)
 {FTM_PWM_Duty(FTM1,FTM_CH0,230);}
 else if(((Display1-Display2)>-5000) && ((Display1-Display2)<-1000))
 {FTM_PWM_Duty(FTM1,FTM_CH0,235);}
  else if(Display2-Display1>8000)
  {
    FTM_PWM_Duty(FTM1,FTM_CH0,250);
  if(Display1<10000)
  {SetSpeed=0.30;}
  }
  else if((Display1-Display2>5500) && (Display1-Display2>10000))
  {FTM_PWM_Duty(FTM1,FTM_CH0,200);}
  else if(Display1-Display2>10000)
  {FTM_PWM_Duty(FTM1,FTM_CH0,190);}
}

//   FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle);
}




//���pwmֵ���
void Moto_Out() //2msһ��
{
 //�ٶȿ�������޷�
 if(PID_SPEED.OUT>0.8)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
 PID_SPEED.OUT=0.8;
 if(PID_SPEED.OUT<-0.8)
 PID_SPEED.OUT=-0.8;
 MotorOut=PID_SPEED.OUT;
 
 
 //��ֵ�޷�����ֹ���ٹ���
 
  
  if(MotorOut>0.99)MotorOut=0.99;                     
  if(MotorOut<-0.99)MotorOut=-0.99; 
 

 if(Stop)                                //���ֹͣ�����������ռ�ձ�Ϊ��
 {
    MotorOut=0;
    LED_BLUE_ON;
 }
 if(Stop_Brake==1 && wycnt<=250 ){                //�յ�ɲ�����ֵ�ת
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);     
   FTM_PWM_Duty(FTM0,FTM_CH1,2220); 
 }
  if(wycnt>250 ){                //�յ�ɲ�����ֵ�ת
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);     
   FTM_PWM_Duty(FTM0,FTM_CH1,0); 
 }
 


 if(!Stop_Brake){
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