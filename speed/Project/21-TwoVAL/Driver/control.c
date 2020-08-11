/*
减小了框定PID_TURN.OUT的最小值

*/
#include "include.h"

//速度类变量
uint8 Style=1;                                  //1不加速，0加速
float SpeedControlOutNew;
float SpeedControlOutOld;
float SpeedControlIntegral=0,Hill_Slow_Ratio;
uint8  Set_Angle;                               //加速前倾角度
int   SpeedCount;
int   Speed_Filter_Times=50;                    //速度平滑输出
float CarSpeed=0,ControlSpeed=0,AverageSpeed,SetSpeed=0,Distance=0;
float Speed_H=0,Speed_M=0,Speed_L=0;
int   Stop_Brake=0;                               //刹车
int   wycnt=0;
//方向类变量
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed=0;
int   DirectionCount;
float Delt_error,Middle_Err;
float Turn_Out;
float Turn_Angle_Integral;

/**舵机相关**/
int sever_middle=225;                   //值越大越偏右 225ganghao 
int sever_range=30;                     //19(实际范围)//25(原)//28  //s 55

//模糊化系数
float  Delta_P;
float  Delta_D;
float  Fuzzy_Kp;
float  Fuzzy_Kd;
//PID控制类变量
PID PID_SPEED,PID_TURN;

float  MotorOut;                                //电机输出量          
uint8   Starting,Stop;                          //stop=1使电机不输出，Starting=1起跑准备状态
uint8 Encoder_Disable=0;

extern int16 GYRO_OFFSET_Z;
extern int16 leftspeed,rightspeed;
//extern int16 disspeed;
extern uint8 lost_line;
extern int flag,flag_obstacle;

float Distance1=10000;
uint8 flag2=0,flag_100;

void Get_Speed()      //可能是4ms执行一次     //5ms执行一次
{  
  int qd1_result;
  int Car;
  //qd1_result =- FTM_QUAD_get(FTM2); 
  //FTM_QUAD_clean(FTM2);
  //disspeed = leftspeed; //+ rightspeed;
  qd1_result = leftspeed;// + rightspeed;
  
  leftspeed=0;
  //rightspeed=0;
  
  Distance+=qd1_result/2500.0;                                  //转化为4ms内跑过的距离 500线，一圈20cm
  //CarSpeed=CarSpeed*0.1+0.9*qd1_result*250.0/3100;            //求出车速转换为M/S 4ms一次
  CarSpeed=CarSpeed*0.1+0.9*qd1_result*0.01; //*250/2500;       //求出车速转换为M/S
  //disspeed= CarSpeed;
  if(CarSpeed>=4)CarSpeed=4; 
  
}
//速度控制量计算  100ms一次
void Speed_Control(void)                        //更新SpeedControlOutOld  计算SpeedControlOutNew
{  
  static float PreError[20]={0};
  float  SpeedError;
  uint8 i;
  
  
  //SpeedError=0.42-CarSpeed;//测试
  //现在让小车有变速功能
  if(Style==0)//不加速，1
    SpeedError=SetSpeed-CarSpeed;               //车速偏小 误差为正
  else{//加速  
   /*  if(flag==1) //18
    {
      uart_putchar(UART0,'J'); 
      //uart_putchar(UART0,'Z');
       //SpeedError=0.2-CarSpeed; //减速
       SpeedError=0.2-CarSpeed;
    }*/
    if(RoadType==6 || RoadType==7 || RoadType==16 || RoadType==32 || RoadType==33  || RoadType==36 || RoadType==37) //50
    {
      
      if(CarSpeed>0.7)                          //设定值为0.58时，车速通常小于0.4  0.385
      {  
        SpeedError=0.2-CarSpeed;                //减速

      }
      else
      {
        SpeedError=0.7-CarSpeed;                //慢速

      }
    }
    else if(RoadType==40 || RoadType==41 || RoadType==18)//0是快速
    {                                           //障碍物  中速
      if(CarSpeed>0.5)                          //设定值为0.7时，车速通常小于0.54
      {  
        SpeedError=0.2-CarSpeed;                //过弯时减速

      }
      else
      {
        SpeedError=Speed_M-CarSpeed;            //中速

      }
    }
    else if(RoadType==3 || RoadType==4 || RoadType==5 || RoadType==13 || RoadType==14  || RoadType==15) //18
    {
      SpeedError=0.65-CarSpeed;                 //减速
     // SpeedError=0.15-CarSpeed; 
     // SpeedError=0; 
     // SetSpeed=0;
    }
//    else if(RoadType==100)
//    {
//    SpeedError=Speed_M-CarSpeed;
//    //SpeedError=0.60-CarSpeed;                 //减速
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
//    SpeedError=Speed_L-CarSpeed;                //出 慢速
//    
//    }

    
    else{                                       //RoadType==1,2,7,12,17,18
      SpeedError=Speed_M-CarSpeed; //速
    }
  }

 
  

  //求出最近20个偏差的总和作为积分项
  SpeedControlIntegral=0;
  for(i=0;i<19;i++)
  {
     PreError[i]=PreError[i+1]; 
     SpeedControlIntegral+=PreError[i];
  }
  PreError[19]=SpeedError;
  SpeedControlIntegral+=PreError[19];
  //速度更新
  SpeedControlOutOld=SpeedControlOutNew;

  SpeedControlOutNew=PID_SPEED.P*SpeedError+PID_SPEED.I*SpeedControlIntegral;   //PI控制 当CarSpeed增大且向SetSpeed靠近，此变量为正且变小  当CarSpeed减小且向SetSpeed靠近，此变量为负且绝对值变小
}
//计算速度控制量PID_SPEED.OUT
void Speed_Control_Output(void)                         //2ms一次 每次调用前SpeedCount加1 SpeedCount属于[1,50]
{ 
  float fValue; 
  fValue = SpeedControlOutNew - SpeedControlOutOld; 
  PID_SPEED.OUT = fValue * (SpeedCount+1)/Speed_Filter_Times+SpeedControlOutOld; 
  //当车速误差为零，PID_SPEED.OUT=0，使得占空比为零
}
/********************方向控制量计算***************/
void Direction_Control(void)
{
  static int Calculate_Length=0;
  Turn_Speed= -0.01*(Get_Z_Gyro() - GYRO_OFFSET_Z);     //0.01//GYRO_OFFSET_Z静态Z轴角速度值，在I2C中定为17
  if(Turn_Speed<10&&Turn_Speed>-10)
  {
    Turn_Speed=0;
  }
  
  
    Fuzzy(Middle_Err,Delt_error);                       //得到模糊化系数float  Delta_P;float  Delta_D;
    Delta_P=Delta_P*Fuzzy_Kp;                           //Fuzzy_Kp Fuzzy_Kd在founction.c中的参数初始化函数设定值
    Delta_D=Delta_D*Fuzzy_Kd;

  PID_TURN.pout=(PID_TURN.P+Delta_P)*Middle_Err;        //Middle_Err
  PID_TURN.dout=(PID_TURN.D+Delta_D)*Turn_Speed*0.1;
  Turn_Out= PID_TURN.pout + PID_TURN.dout;
  
  Turn_Out=Turn_Out_Filter(Turn_Out);                   //转动输出滤波 
  
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

       FTM_PWM_Duty(FTM1,FTM_CH0,190);   //190，270往左
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
          FTM_PWM_Duty(FTM1,FTM_CH0,sever_middle + PID_TURN.OUT);   //舵机的值在0-180 反复变化
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




//电机pwm值输出
void Moto_Out() //2ms一次
{
 //速度控制输出限幅
 if(PID_SPEED.OUT>0.8)//如果车子前倾，则车模的速度控制输出为正，反之为负
 PID_SPEED.OUT=0.8;
 if(PID_SPEED.OUT<-0.8)
 PID_SPEED.OUT=-0.8;
 MotorOut=PID_SPEED.OUT;
 
 
 //正值限幅，防止减速过大
 
  
  if(MotorOut>0.99)MotorOut=0.99;                     
  if(MotorOut<-0.99)MotorOut=-0.99; 
 

 if(Stop)                                //如果停止则电机不输出，占空比为零
 {
    MotorOut=0;
    LED_BLUE_ON;
 }
 if(Stop_Brake==1 && wycnt<=250 ){                //终点刹车车轮倒转
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);     
   FTM_PWM_Duty(FTM0,FTM_CH1,2220); 
 }
  if(wycnt>250 ){                //终点刹车车轮倒转
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);     
   FTM_PWM_Duty(FTM0,FTM_CH1,0); 
 }
 


 if(!Stop_Brake){
   if(MotorOut>=0) //正转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,MotorOut*10000);//占空比精度为10000  第三个参数为占空比分子
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
     //if(MotorOut>0) my_putchar('A');
     ///else my_putchar('a');
  }
  else   //反转
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH1,-MotorOut*10000);
     //my_putchar('B');
  }
 }
 
}
 

float  Turn_Out_Filter(float turn_out)    //转向控制输出滤波      
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
float  Middle_Err_Filter(float middle_err)    //中心偏差滤波    从未使用
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