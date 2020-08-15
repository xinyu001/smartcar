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
int sever_middle=124;                   //值越大越偏右  
int sever_range=35;                     //19(实际范围)//25(原)//28  //s 

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
extern uint8 RoadType;
float Distance1=10000;
uint8 flag2=0,flag_100;

// 电磁转向AD控制参数
float   AD_Value[2][5],AD_Value1[2],AD_V[2][5];
float AD_val_1;
float AD_val_2;
float AD_val_3;
float AD_val_4;
float dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4;
float disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4;
float AD_val_1_max=20000;
float AD_val_2_max=20000;
float AD_val_3_max=20000;
float AD_val_4_max=20000;         //大概两边相等放中间归一化是40几
float AD_val_1_min=2200;
float AD_val_2_min=2200;
float AD_val_3_min=2200;
float AD_val_4_min=2200;


int circle_Flag;
int turn_Flag;
int turn_Flag2=0;
int turn_right_Flag=0;
int turn_left_Flag=0;
int Go_Out_Circle=0;
float Distance101,Distance102,Distance103,Distance104,Distance105;
float Distance111,Distance112,Distance113,Distance114,Distance115;
void roadturncal();
float adc_ave();

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
  if(Style==0)                                  //不加速
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
        SpeedError=Speed_L-CarSpeed;                //慢速

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
      SpeedError=Speed_M-CarSpeed;                 //减速
     // SpeedError=0.15-CarSpeed; 
     // SpeedError=0; 
     // SetSpeed=0;
    }
    else if(RoadType==100)
    {
    SpeedError=Speed_M-CarSpeed;
    //SpeedError=0.60-CarSpeed;                 //减速
    
    }
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
      SpeedError=Speed_H-CarSpeed; //快速
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
  Turn_Out= PID_TURN.pout - PID_TURN.dout;
  
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
       //uart_putchar(UART0,'Y');
    }
    else if(lost_line==2)
    {
       FTM_PWM_Duty(FTM1,FTM_CH0,190);    //left  85
       //uart_putchar(UART0,'Z');
    }
  }
  
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
 
 
  if(MotorOut>0.99)MotorOut=0.99;               //正值限幅，防止减速过大       
  if(MotorOut<-0.99)MotorOut=-0.99; 
 

 if(Stop)                                //如果停止则电机不输出，占空比为零
 {
    MotorOut=0;
    LED_BLUE_ON;
 }
 if(Stop_Brake && wycnt<=250){
   
   FTM_PWM_Duty(FTM0,FTM_CH0,0);
   FTM_PWM_Duty(FTM0,FTM_CH1,3000);
   wycnt++;
 }
 if(wycnt>250){
   FTM_PWM_Duty(FTM0,FTM_CH0,0);
   FTM_PWM_Duty(FTM0,FTM_CH1,0);
 }
 
if(Stop_Brake!=1)
 {if(MotorOut>=0) //正转
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
float  Middle_Err_Filter(float middle_err)    //中心偏差滤波    
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

//电磁部分
void roadturncal()  //转向控制程序  
{ static uint8 turn_Flag2;
  uint16 i,j,k; 
  float temp,ad_sum[2],AD_sum[2]; 
  float Inductor_ADC[4];
  float err;
// 电感AD值滤波程序，比较慢 ，底层采集数据自动进行非负处理
  for(i=0;i<5;i++)
  {


     AD_Value[0][i] = adc_ave(ADC1_SE10, ADC_16bit,3);  //左电感  
     AD_Value[1][i] = adc_ave(ADC1_SE11, ADC_16bit,3);  //右电感 
  }

// Voltage = adc_ave(ADC_CHANNEL_AD15,ADC_12BIT,2)*3.81;
  
    AD_val_3 = adc_ave(ADC1_SE8, ADC_16bit,2);  //左边八字电感  
    AD_val_4 = adc_ave(ADC1_SE13, ADC_16bit,2); //右边八字电感 
 
  for(i=0;i<2;i++)     //左右电感  从小到大排序  
  {
       for(j=0;j<4;j++)  
       {
          for(k=0;k<4-j;k++)
          {
             if(AD_Value[i][k] > AD_Value[i][k+1])  //前面的比后面的大  则进行交换
             {
                temp = AD_Value[i][k+1];
                AD_Value[i][k+1] = AD_Value[i][k];
                AD_Value[i][k] = temp;
             }
          }
       }
  }
  for(i=0;i<2;i++)    //求中间三项的和
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
  AD_val_1=AD_sum[0]/5;  //左电感
  AD_val_2=AD_sum[1]/5;  //右电感
  
  dis_AD_val_1=AD_val_1;  //左
  dis_AD_val_2=AD_val_2;  //右
  dis_AD_val_3=AD_val_3;  //左八
  dis_AD_val_4=AD_val_4;  //右八 
  
  //限幅
  if(AD_val_1>AD_val_1_max)		AD_val_1=AD_val_1_max;
  if(AD_val_2>AD_val_2_max)		AD_val_2=AD_val_2_max;
  if(AD_val_3>AD_val_3_max)		AD_val_3=AD_val_3_max;
  if(AD_val_4>AD_val_4_max)		AD_val_4=AD_val_4_max;
  
  if(AD_val_1<AD_val_1_min)		AD_val_1=AD_val_1_min;
  if(AD_val_2<AD_val_2_min)		AD_val_2=AD_val_2_min;
  if(AD_val_3<AD_val_3_min)		AD_val_3=AD_val_3_min;
  if(AD_val_4<AD_val_4_min)		AD_val_4=AD_val_4_min;
//  
  
  //归一化
  AD_val_1=(100*(AD_val_1 -AD_val_1_min))/(AD_val_1_max-AD_val_1_min);
  AD_val_2=(100*(AD_val_2 -AD_val_2_min))/(AD_val_2_max-AD_val_2_min);
  AD_val_3=(100*(AD_val_3 -AD_val_3_min))/(AD_val_3_max-AD_val_3_min);
  AD_val_4=(100*(AD_val_4 -AD_val_4_min))/(AD_val_4_max-AD_val_4_min);
 
  disgy_AD_val_1 = AD_val_1;
  disgy_AD_val_2 = AD_val_2;
  disgy_AD_val_3 = AD_val_3;
  disgy_AD_val_4 = AD_val_4;
 


////////////     四电感法,带圆环    /////////////////////////////
  Inductor_ADC[0]= disgy_AD_val_1; //左
  Inductor_ADC[1]= disgy_AD_val_2; //右
  Inductor_ADC[2]= disgy_AD_val_3; //左八
  Inductor_ADC[3]= disgy_AD_val_4; //右八
  if((Inductor_ADC[0]+Inductor_ADC[1])>20) //有电磁信号，没有丢线  
    Middle_Err=(float)100*(AD_val_2-AD_val_1)/(AD_val_2+AD_val_1) ;  //归一
    //Middle_Err=disgy_AD_val_1-disgy_AD_val_2;
  
   //下面的各个判定参数需要实际测量来修正
  if(Go_Out_Circle==0&&circle_Flag==0&&((Inductor_ADC[0]>90 && Inductor_ADC[1]>75 && Inductor_ADC[3]>60&& Inductor_ADC[2]<40) || (Inductor_ADC[0]>90 && Inductor_ADC[1]>75 && Inductor_ADC[2]>70&& Inductor_ADC[3]<40)))//(Inductor_ADC[0]>3000||Inductor_ADC[1]>3000)&&
  {
     circle_Flag=1;  //前言可能是圆环
     if(Inductor_ADC[2]<Inductor_ADC[3])  //左圆环         //Inductor_ADC[0]>Inductor_ADC[1]&&
     {
        turn_left_Flag=1;
     }
     if(Inductor_ADC[2]>Inductor_ADC[3])  //右圆环         //Inductor_ADC[1]>Inductor_ADC[0]&&
     {
        turn_right_Flag=1;
     }
     if(turn_right_Flag==0&&turn_left_Flag==0) //误判
     {
        circle_Flag=0;
     }
  }
  err=Inductor_ADC[3]-Inductor_ADC[2]; //八字电感决定进圆环
  if(ABS(err)<5&&circle_Flag==1&&turn_Flag==0 && Inductor_ADC[0]>70 && Inductor_ADC[1]>70)
  {
     turn_Flag=1;                               //进入圆环
     if(turn_right_Flag==1)
     {RoadType=101;//
     //BEEP_ON;
     Distance101=Distance;              //记录下检测到右环时的距离
     }
     if(turn_left_Flag==1){
     RoadType=111;
     Distance111=Distance;              //记录下检测到左环时的距离
     }
  }
  //左环
  if(RoadType==111 && Distance-Distance111>1.5){                
    RoadType=112;                              //进入左环，往左打固定方向距离
    Distance112=Distance;                       //记录进入左环时的距离
  }
  
  if(RoadType==112 && Distance-Distance112>1){
  
    RoadType=113;              //左环内行驶
    
  }
  if(RoadType==113 && Distance-Distance112>5.5){
  
    RoadType=114;              //出环左拐
    turn_Flag2=1;
    Distance114=Distance;  
    
  }
  if(RoadType==114 && Distance-Distance114>1){
  
    RoadType=115;              //出环
    Distance115=Distance;  
    
  }
 if(RoadType==115 && Distance-Distance114>1.5){
  
    RoadType=100;              //出环一段距离后变回普通赛道 
    Go_Out_Circle=1;
    
    
  }
  
  //右环
  if(RoadType==101 && Distance-Distance101>1.5){                
    RoadType=102;                              //进入右环，往右打固定方向距离
    Distance102=Distance;                       //记录进入右环时的距离
  }
  
  if(RoadType==102 && Distance-Distance102>1){
  
    RoadType=103;              //kaishi环内行驶
    
  }
  if(RoadType==103 && Distance-Distance102>5.5){
  
    RoadType=104;              //出环右拐
    turn_Flag2=1;
    Distance104=Distance;  
    
  }
  if(RoadType==104 && Distance-Distance104>1){
  
    RoadType=105;              //出右环
    Distance105=Distance;  
    
  }
 if(RoadType==105 && Distance-Distance104>1.5){
  
    RoadType=100;              //出环一段距离后变回普通赛道 
    Go_Out_Circle=1;
    
    
  }
  
  
  if(RoadType==103&&turn_Flag==1 && turn_Flag2==0) //圆环内部
  {
       
      if((Inductor_ADC[1]+Inductor_ADC[2])>10)
      Middle_Err=(float)100*(AD_val_2-AD_val_3)/(AD_val_2+AD_val_3);
        
  }
  if(RoadType==113&&turn_Flag==1 && turn_Flag2==0) //圆环内部
  {
       
      if((Inductor_ADC[0]+Inductor_ADC[1])>10)
      Middle_Err=(float)100*(AD_val_4-AD_val_1)/(AD_val_4+AD_val_1);
        
  }
  if(RoadType==102){            //进右环
    Middle_Err=-90;
  }
  if(RoadType==104){            //出右环
    Middle_Err=-90;
  }
  if(RoadType==112){            //进左环
    Middle_Err=90;
  }
  if(RoadType==114){            //出左环
    Middle_Err=90;
  }
  if(RoadType==200){            //出库
    Middle_Err=-90;
  }
  if(RoadType==205){            //入库
    Middle_Err=-90;
  }
  if(Go_Out_Circle==1) //完成一次圆环动作//s 2？不是1吗？？？
  {
     turn_Flag=0; 
     turn_Flag2=0;
     turn_right_Flag=0;
     turn_left_Flag=0;
     circle_Flag=0;
     Go_Out_Circle=0;
     //BEEP_OFF;
  }
  Middle_Err=Middle_Err_Filter(Middle_Err);
  Middle_Err=-Middle_Err*(Middle_Err*Middle_Err/1250+2)/30;
}
//  if(turn_left_Flag==1&&(Inductor_ADC[0]>Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[3]<5000)
//  {
//     turn_Flag2=1;  //准备出环
//  }
//  if(turn_right_Flag==1&&(Inductor_ADC[0]>Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[1]>70&&Inductor_ADC[2]>70&&Inductor_ADC[0]>70)
//  {
//     turn_Flag2=1;  //准备出环
//  }
//  if(turn_Flag2==1 && ((turn_left_Flag==1 && Inductor_ADC[0]>80 && Inductor_ADC[1]>80) || (turn_right_Flag==1 && Inductor_ADC[0]>60 && Inductor_ADC[1]>60&& Inductor_ADC[2]>60 && Inductor_ADC[3]>60)))
//  {
//    Go_Out_Circle=1;  //出环
//  }

  
//  if(Inductor_ADC[0]<20 || Inductor_ADC[1]<20){
//  RoadType=100;
//  }
//  if(RoadType==100&&turn_Flag==1 && turn_Flag2==0) //圆环内部
//  {
//        if(turn_left_Flag==1)
//        {
//           if((Inductor_ADC[0]+Inductor_ADC[3])>10)
//            Middle_Err=-(float)100*(AD_val_4-AD_val_1)/(AD_val_4+AD_val_1);
//        }
//        if(turn_right_Flag==1)
//        {
//          if((Inductor_ADC[1]+Inductor_ADC[2])>10)
//            Middle_Err=-(float)100*(AD_val_2-AD_val_3)/(AD_val_2+AD_val_3);
//        }
//  }
//  if(turn_left_Flag==1&&(Inductor_ADC[0]>Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[3]<5000)
//  {
//     turn_Flag2=1;  //准备出环
//  }
//  if(turn_right_Flag==1&&(Inductor_ADC[0]>Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[1]>70&&Inductor_ADC[2]>70&&Inductor_ADC[0]>70)
//  {
//     turn_Flag2=1;  //准备出环
//  }
//  if(turn_Flag2==1 && ((turn_left_Flag==1 && Inductor_ADC[0]>80 && Inductor_ADC[1]>80) || (turn_right_Flag==1 && Inductor_ADC[0]>60 && Inductor_ADC[1]>60&& Inductor_ADC[2]>60 && Inductor_ADC[3]>60)))
//  {
//    Go_Out_Circle=1;  //出环
//  }
//  if(Go_Out_Circle==1) //完成一次圆环动作//s 2？不是1吗？？？
//  {
//     turn_Flag=0; 
//     turn_Flag2=0;
//     turn_right_Flag=0;
//     turn_left_Flag=0;
//     circle_Flag=0;
//   //  Go_Out_Circle=0;
//     //BEEP_OFF;
//  }
  ///偏离很大时
//  if( (Inductor_ADC[0]<4) && ((Inductor_ADC[1] - Inductor_ADC[0]) < 450) && ((Inductor_ADC[1] - Inductor_ADC[0]) > 350) )  Middle_Err = 80;
//  if( (Inductor_ADC[1]<4) && ((Inductor_ADC[0] - Inductor_ADC[1]) < 450) && ((Inductor_ADC[0] - Inductor_ADC[1]) > 350) )  Middle_Err = -80; 
  

    

  
  /*
  //下面的各个判定参数需要实际测量来修正
  if(circle_Flag==0&&((Inductor_ADC[0]>2500 && Inductor_ADC[1]>1300 && Inductor_ADC[3]>800) || (Inductor_ADC[1]>2500 && Inductor_ADC[0]>1300 && Inductor_ADC[2]>800)))//(Inductor_ADC[0]>3000||Inductor_ADC[1]>3000)&&
  {
     circle_Flag=1;  //前言可能是圆环
     if(Inductor_ADC[0]>Inductor_ADC[1]&&Inductor_ADC[2]<Inductor_ADC[3])  //左圆环
     {
        turn_left_Flag=1;
     }
     if(Inductor_ADC[1]>Inductor_ADC[0]&&Inductor_ADC[2]>Inductor_ADC[3])  //右圆环
     {
        turn_right_Flag=1;
     }
     if(turn_right_Flag==0&&turn_left_Flag==0) //误判
     {
        circle_Flag=0;
     }
  }
  err=Inductor_ADC[3]-Inductor_ADC[2]; //八字电感决定进圆环
  if(ABS(err)<150&&circle_Flag==1&&turn_Flag==0 && Inductor_ADC[2]>1800 && Inductor_ADC[3]>1800)
  {
     turn_Flag=1; //进入圆环
     //BEEP_ON;
  }
  if(turn_Flag==1 && turn_Flag2==0) //圆环内部
  {
        if(turn_left_Flag==1)
        {
           if((Inductor_ADC[0]+Inductor_ADC[3])>100)
            Middle_Err=(float)100*(AD_val_4-AD_val_1)/(AD_val_4+AD_val_1) ;
        }
        if(turn_right_Flag==1)
        {
          if((Inductor_ADC[1]+Inductor_ADC[2])>100)
            Middle_Err=(float)100*(AD_val_2-AD_val_3)/(AD_val_2+AD_val_3) ;
        }
  }
  if(turn_left_Flag==1&&(Inductor_ADC[0]>Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[3]<360)
  {
     turn_Flag2=1;  //准备出环
  }
  if(turn_right_Flag==1&&(Inductor_ADC[0]<Inductor_ADC[1])&&turn_Flag==1&&Inductor_ADC[2]<360)
  {
     turn_Flag2=1;  //准备出环
  }
  if(turn_Flag2==1 && ((turn_left_Flag==1 && Inductor_ADC[0]>2500 && Inductor_ADC[1]>1000) || (turn_right_Flag==1 && Inductor_ADC[1]>2500 && Inductor_ADC[0]>1000)))
  {
    Go_Out_Circle=1;  //出环
  }
  if(Go_Out_Circle==2) //完成一次圆环动作//s 2？不是1吗？？？
  {
     turn_Flag=0; 
     turn_Flag2=0;
     turn_right_Flag=0;
     turn_left_Flag=0;
     circle_Flag=0;
     Go_Out_Circle=0;
     //BEEP_OFF;
  }
  ///偏离很大时
//  if( (Inductor_ADC[0]<4) && ((Inductor_ADC[1] - Inductor_ADC[0]) < 450) && ((Inductor_ADC[1] - Inductor_ADC[0]) > 350) )  Middle_Err = 80;
//  if( (Inductor_ADC[1]<4) && ((Inductor_ADC[0] - Inductor_ADC[1]) < 450) && ((Inductor_ADC[0] - Inductor_ADC[1]) > 350) )  Middle_Err = -80; 
  */
  
  
  // Middle_Err=Middle_Err*(Middle_Err*Middle_Err/1250.0+2)/10;
   

// //////////////  四电感法结束   ///////////////////////////




