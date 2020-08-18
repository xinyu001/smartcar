#include "common.h"
#include "include.h"

float sVariable[20]; //传感器变量
float Variable[20];  //
float Control_Para[15];
float Voltage;
float RunTime;
int   Start_Cnt=0;
uint8 Para_Index_Limit=7;       //一页最多有7个变量序号
uint8 Page_Index=0,Para_Index=1,Light_Tower_Index=0,Para_Checked=0,OLED_Refresh=0;
uint8 SendPara,stop_contorl,send_data_contorl=0,beep;
float Step[6]={0.0001,0.001,0.01,0.1,1.0,10.0};   //默认调节步长为0.01
unsigned char Step_Index=2;
//对应不同的页面

char Para_Name[7][12]={"Set_SPEED\0","FUZZY.P\0","FUZZY.D\0","PID_SPEED.P\0",
"PID_SPEED.I\0","PID_DIREC.P\0","PID_DIREC.D\0"};


char Para_Name1[7][12]={"Set_Speed\0","Set_Angle\0","Acc_Offset\0","Fuzzy_kp",
"Fuzzy_kd","STurnSpeed\0","BTurnAngle\0"};

//int16 disspeed;

extern int16 GYRO_OFFSET_Z;
extern uint8  LMR[3][CAMERA_H];
extern uint8 Style;


extern int   dis_AD_val_1,dis_AD_val_2,dis_AD_val_3 ;
extern int   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3 ;
extern int   AD_val_1_min,AD_val_2_min,AD_val_3_min;
extern int   AD_val_1_max,AD_val_2_max,AD_val_3_max;

void my_putchar(char temp)
{
      uart_putchar(UART0,temp); //根据实际的串口号来修改
      
}


void ShiBoQi()    //每个通道2字节，高字节在前，低字节在后
{
  my_putchar(0xff);
  my_putchar(0xff);
//通道1  
  my_putchar(0x10);//0001 0000=16
  my_putchar(0x00);
//通道2  
  my_putchar(0x00);
  my_putchar(0x01);
//通道3
  my_putchar(0x07);
  my_putchar(0xd0);

//通道4
  my_putchar(0x00);
  my_putchar(0x00);

}





/*用来通知上位机新的一组数据开始，要保存数据必须发送它*/
void Send_Begin()                 //从未调用
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}

void Variable_update()//主函数的while(1)中被调用
{           //这个数组赋值后仅在Send_Variable()中被发送给上位机，然而Send_Variable()从未调用
  Variable[3]=Middle_Err;
  Variable[4]=Delt_error;
  Variable[5]=Turn_Speed;
  Variable[6]=Delta_P;
  Variable[7]=CarSpeed;
  Variable[8]= MotorOut ;
  Variable[9]= PID_SPEED.OUT ;
  Variable[10]= PID_TURN.OUT;
  Variable[14]=Distance;
  Variable[15]=RunTime;
}
void Para_Update()//  function.c中Para_Init()中这7个变量被反向赋值//在多处被调用 通过按钮修改参数后调用
{ 
  SetSpeed=Control_Para[0];
  Fuzzy_Kp=Control_Para[1];
  Fuzzy_Kd=Control_Para[2];
  PID_SPEED.P=Control_Para[3];
  PID_SPEED.I=Control_Para[4];
  PID_TURN.P=Control_Para[5];
  PID_TURN.D=Control_Para[6]; 
}


void OLED_Draw_UI()  //画出界面
{ 
   uint8 i;
   if(Page_Index<=1) 
   {
     Voltage=adc_once(ADC1_SE6a,ADC_12bit);
     Voltage=Voltage*Vol_ratio; //转换为实际电压
     OLED_P6x8Str(0,0,"Voltage=");                          //显示电池电压
     OLED_PrintValueF(48, 0,Voltage,2);                     
     OLED_PrintValueF(72, 0,Step[Step_Index],5);            //显示调节步长值
     if(Para_Index==7)
     {
        reverse=1; 
         OLED_P6x8Str(116,0,"EE"); 
        reverse=0;
     }
     else  
     {
        OLED_P6x8Str(116,0,"EE"); 
     }
     OLED_Set_Pos(122,7);
     OLED_P6x8Char(Page_Index+48);                         //写出页面序号
   }
  /////////////////////////////////////////////////////////第1页  PID调节
  if(Page_Index==0)                
  {
    for(i=0;i<7;i++)
    {
      if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name[i]);   //将参量名反转显示
       reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Name[i]);

      if(i==Para_Index&&Para_Checked)
      {
        reverse=1;
        OLED_PrintValueF(72, i+1,Control_Para[i],5);
        reverse=0;
      }
      else  OLED_PrintValueF(72, i+1,Control_Para[i],5);
      
      OLED_Set_Pos(116,i+1);

    }
  }
    /////////////////////////////////////////////////////////第2页 状态显示
  else if(Page_Index==1)
  {

/*   
    OLED_P6x8Str(0,2,"Distance");
    OLED_PrintValueF(72, 2,Distance,4);
    OLED_P6x8Str(0,3,"RunTime");
    OLED_PrintValueF(72, 3,RunTime,4);
    OLED_P6x8Str(0,4,"Average_Spd");
    OLED_PrintValueF(72, 4,AverageSpeed,4);  
    OLED_P6x8Str(0,5,"GYRO_OFF_Z");
    OLED_PrintValueF(72, 5,Middle_Err,4);  
    OLED_P6x8Str(0,6,"NowSPEED");
    OLED_PrintValueF(72, 6,CarSpeed,4);  //Middle_Err CarSpeed
    reverse=0;
 */
    OLED_P6x8Str(0,1,"RoadType");
    OLED_PrintValueF(72, 1,RoadType,4);
    OLED_P6x8Str(0,2,"Right[1]");
    OLED_PrintValueF(72, 2,SlopeRight[1],4);
    OLED_P6x8Str(0,3,"Right[2]");
    OLED_PrintValueF(72, 3,SlopeRight[2],4);
    OLED_P6x8Str(0,4,"Right[3]");
    OLED_PrintValueF(72, 4,SlopeRight[3],4);
    OLED_P6x8Str(0,5,"LMR[1][27]");
    OLED_PrintValueF(72, 5,LMR[1][27],4);
    OLED_P6x8Str(0,6,"LMR[1][30]");
    OLED_PrintValueF(72, 6,LMR[1][30],4);
    OLED_P6x8Str(0,7,"Distance");
    OLED_PrintValueF(72, 7,Distance,4);

  } 
  ////////////////////////////////////////////////////////////第3页 显示数据
  else if(Page_Index==2){

    OLED_P6x8Str(0,1,"RoadType");
    OLED_PrintValueF(72, 1,RoadType,4);
    
    OLED_P6x8Str(0,2,"val_1:");
    OLED_PrintValueF(72, 2,dis_AD_val_1,4);
    OLED_P6x8Str(0,3,"val_2:");
    OLED_PrintValueF(72, 3,dis_AD_val_2,4);
    OLED_P6x8Str(0,4,"val_3:");
    OLED_PrintValueF(72, 4,dis_AD_val_3,4);
    OLED_P6x8Str(0,5,"val_1:");
    OLED_PrintValueF(72, 5,disgy_AD_val_1,4);
    OLED_P6x8Str(0,6,"val_2:");
    OLED_PrintValueF(72, 6,disgy_AD_val_2,4);
    OLED_P6x8Str(0,7,"val_3:");
    OLED_PrintValueF(72, 7,disgy_AD_val_3,4);
    
    OLED_Set_Pos(122,7);
    OLED_P6x8Char(Page_Index+48); 
    
    
  }
  ////////////////////////////////////////////////////////////第4页 传感器显示
  else if(Page_Index==3) 
  {
    OLED_Draw_camera();
    
    OLED_P6x8Str(90,6,"RT");
    OLED_PrintValueF(110, 6,RoadType,4);
   
    OLED_Set_Pos(122,7);
    OLED_P6x8Char(Page_Index+48);
  }
  ///////////////////////////////第五页 显示数据
    else if(Page_Index==4){

    OLED_P6x8Str(0,1,"RoadType");
    OLED_PrintValueF(72, 1,RoadType,4);
    
    OLED_P6x8Str(0,2,"v1_min:");
    OLED_PrintValueF(72, 2,AD_val_1_min,4);
    OLED_P6x8Str(0,3,"v2_min:");
    OLED_PrintValueF(72, 3,AD_val_2_min,4);
    OLED_P6x8Str(0,4,"v3_min:");
    OLED_PrintValueF(72, 4,AD_val_3_min,4);
    OLED_P6x8Str(0,5,"v1_max:");
    OLED_PrintValueF(72, 5,AD_val_1_max,4);
    OLED_P6x8Str(0,6,"v2_max:");
    OLED_PrintValueF(72, 6,AD_val_2_max,4);
    OLED_P6x8Str(0,7,"v3_max:");
    OLED_PrintValueF(72, 7,AD_val_3_max,4);
    
    OLED_Set_Pos(122,7);
    OLED_P6x8Char(Page_Index+48); 
  
}
///第六页数据
    else if(Page_Index==5){
    
     OLED_PrintValueF(0, 1,LMR[0][5],4);
    OLED_PrintValueF(0, 2,LMR[0][6],4);
    OLED_PrintValueF(0, 3,LMR[0][7],4);
    OLED_PrintValueF(0, 4,LMR[0][8],4);
    OLED_PrintValueF(0, 5,LMR[0][9],4);
    OLED_PrintValueF(0, 6,LMR[0][10],4);
    OLED_PrintValueF(0, 7,LMR[0][11],4);
    
    OLED_PrintValueF(36, 1,LMR[0][12],4);
    OLED_PrintValueF(36, 2,LMR[0][13],4);
    OLED_PrintValueF(36, 3,LMR[0][14],4);
    OLED_PrintValueF(36, 4,LMR[0][15],4);
    OLED_PrintValueF(36, 5,LMR[0][16],4);
    OLED_PrintValueF(36, 6,LMR[0][17],4);
    OLED_PrintValueF(36, 7,LMR[0][18],4);
    
    
    OLED_PrintValueF(72, 1,LMR[0][20],4);
    OLED_PrintValueF(72, 2,LMR[0][21],4);
    OLED_PrintValueF(72, 3,LMR[0][22],4);
    OLED_PrintValueF(72, 4,LMR[0][23],4);
    OLED_PrintValueF(72, 5,LMR[0][24],4);
    OLED_PrintValueF(72, 6,LMR[0][25],4);
    OLED_PrintValueF(72, 7,RoadType,4);  
      
      
    OLED_Set_Pos(122,7);
    OLED_P6x8Char(Page_Index+48); 
    }
   
}

/*
 * 读拨码开关的值
 */
void Read_Switch() //0表示ON(左) 1表示OFF(右)
{
  if(gpio_get(SW1)==0)  //拨码开关1功能   
  {
    Style=1; ///////////////////7月7日注释
  }
  else           
  {
    
  }

  if(gpio_get(SW2)==0)   //拨码开关2功能
  {
    //SetSpeed=0;
  } 
  else 
  {
    //SetSpeed=0.72;
  }

  if(gpio_get(SW3)==0)   //拨码开关3功能
  {
    //SetSpeed=0;
  } 
  else 
  {
    //SetSpeed=0.2;
  }

} 

/*
 * 检测按键是否按下
 */
void Check_BottonPress()                        //主函数while(1)中被调用
{
      //显示按键
  
  if(OLED_Refresh==0&&BT_YES_IN==0)             //当显示屏关闭且按键按下时
  {
   if(BT_YES_IN==0)
   {
      //去抖
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_YES_IN==0)
      {    
        if(OLED_Refresh==false)                 //false=0；true=1
        {
         OLED_Init();
         OLED_Refresh=true;                     //令主函数while(1)中每次循环都显示OLED
        }
        //else
        //{
        //  OLED_Refresh=false;
        //  OLED_Fill(0x00);       
        //}
      }
      while(BT_YES_IN==0);                      //直到按键松开再运行
      DELAY_MS(30);
   } 
  }
   //按键1 yes
   if(BT_YES_IN==0&&OLED_Refresh)               //当按键按下且显示屏打开时
   {
     //去抖
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_YES_IN==0&&OLED_Refresh)
     {    
       if(Para_Index==7) 
       { //参数号为7时，按下按钮，则
         EEPROM_Save();
         Para_Index=0; 
       }
       else
       {//Para_Index不为7且页号是0或1 且参数未被选中，则选中参数
         if(Para_Checked==false&&((Page_Index==1)||(Page_Index==0))) Para_Checked=true;
         else Para_Checked=false;       

       }
      }
      if(Page_Index==1)
      {
         Page_Index=0;
         OLED_Refresh=false;                    //OLED屏打开且页号为1时按下按钮关闭OLED
         OLED_Fill(0x00);                       //清屏
      }
      
      while(BT_YES_IN==0);                      //直到按键松开再运行
   }
   //按键2 Left_L
   if(BT_LEFT_IN==0)
   {
      //去抖
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_LEFT_IN==0)
      {
        if(Para_Checked) 
        {//参数被选中时按下左键，修改步长号
          if(Step_Index==5) 
          Step_Index=5;                         //调节步长号最大为5 对应最大的步长10
          else Step_Index++;
        }
        else 
        { 
          Para_Index=0;
          if(Page_Index==0) Page_Index=5;       //当参数没被选中的时候，按左右键翻页
          else Page_Index--;
          OLED_Fill(0);//清屏 
        }
      }
      while(BT_LEFT_IN==0);                     //直到按键松开再运行
   } 
   //按键6 Right_L
   if(BT_RIGHT_IN==0&&OLED_Refresh)
   {
      //去抖
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_RIGHT_IN==0)
      {
        if(Para_Checked) 
        {
          if(Step_Index==0) 
           Step_Index=0;                        //步长号最小是0，对应最小的步长0.0001
          else
          {
            Step_Index--;
          }
        }
        else 
        { 
          Para_Index=0;
          if(Page_Index==5) Page_Index=0;       //当参数没被选中的时候，按左右键翻页
          else Page_Index++;
         OLED_Fill(0);//清屏 
        }
      }
      while(BT_RIGHT_IN==0);                    //直到按键松开再运行
   }
   //按键3 up
    if(BT_UP_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_UP_IN==0)
      {
   
          if(Para_Checked==false)
          {//参数未被选中时按上键，修改参数号
           if(Para_Index==0) Para_Index=Para_Index_Limit;
           else Para_Index-=1;
          }
          else
          {//参数被选中时按上键，修改参数
              if(Page_Index==0&&Para_Index<=6)                    //修改第0页的参数
            {
              Control_Para[Para_Index]+=Step[Step_Index];
            }
            
            if(Page_Index==1&&Para_Index<=6)                    //修改第1页的参数，但第一页参数不必修改也无法修改
            {
              Control_Para[Para_Index+7]+=Step[Step_Index];
            } 
            Para_Update();
          }
      }  
      while(BT_UP_IN==0);                       //直到按键松开再运行  
   }
   //按键4 down
   if(BT_DOWN_IN==0)
   {
     if(OLED_Refresh)
     {
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_DOWN_IN==0)
      {
          if(Para_Checked==false)
          {//显示屏打开且参数未被选中时按下键，修改参数号
            if(Para_Index==Para_Index_Limit)Para_Index=0;          //防止序号超出范围
            else  Para_Index+=1; 
          }
           else 
           {//显示屏打开且参数被选中时按下键，修改参数
              if(Page_Index==0&&Para_Index<=6)                    //修改第0页的参数
            {
              Control_Para[Para_Index]-=Step[Step_Index];
            }
             
             if(Page_Index==1&&Para_Index<=6)                    //修改第1页的参数，但第一页参数不必修改也无法修改
            {
              Control_Para[Para_Index+7]-=Step[Step_Index];
            }
            Para_Update();
          }
      }
     }
     else
     {//显示屏没开时按下键      
        if(Stop)  //如果当前停止就准备起跑  stop=1表示停止
       { 
         Start_Cnt=1000;
         Starting=true;
         //把所有速度控制变量清零
         SpeedControlOutOld=0;
         SpeedControlOutNew=0;
         SpeedControlIntegral=0;  
         PID_SPEED.OUT=0;
         RunTime=0;
         Distance=0;
         ControlSpeed=0;
       }
     }
     while(BT_DOWN_IN==0);                              //直到按键松开再运行
   }
   
}


void Send_Variable()                                    //从未调用  向上位机发送变量
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Variable_num=16;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0x01);
  my_putchar(Variable_num);
 for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
  my_putchar(0x01);
}




void Modify_Parameter(uint8 *buff)                      //接受上位机数据UART0_RX_IRQHandler()中被调用
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num=14; //14个可改参数
  /*          修改参数数组         */
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE1(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE2(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE3(temp)=*(uint8*)(buff+addr);
       addr++;
       Control_Para[i]=temp;
   }
    Para_Update();
}



void Send_Parameter()//从未调用
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=14;  //14个可改参数
  
 
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0x02);
  my_putchar(Parameter_num);
  for(i=0;i<Parameter_num;i++)
  { 
     temp=Control_Para[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
    my_putchar(0X02);//帧尾
}

void UART0_RX_IRQHandler()
{
  static uint8 recv;
  static uint8 data_cnt=0;
  static uint8 predata[10];
  static uint8 Recv_Buff[100];
  static uint8 Data_Receiving=false;
  
  if(uart_query(UART0)==1)  uart_getchar (UART0,(char*)(&recv));  //根据实际的串口来修改
  /**********代表正在接收来自上位机的参数数据*********/
  if(Data_Receiving)
  {
      if(data_cnt<56)
      {
       Recv_Buff[data_cnt]= recv;
       data_cnt++;
      }
      else
      {
        data_cnt=0;    //达到帧长
        Data_Receiving=false;
        if(recv==2)  //帧尾
        {
           Modify_Parameter(Recv_Buff);
           SendPara=1;      //参数回传，确认参数修改完成
            beep=1; //开启蜂鸣器
        }
      }
  }

  
  
    if( predata[1]==0x55&&predata[0]==0xAA)
    {
      
        switch(recv)         //判断功能字
         { 
            case 1:           //读取参数 
               if(SendPara==0) SendPara=1;
                beep=1; //开启蜂鸣器
              break;
            
            case 2:           //修改参数
              data_cnt=0;
              Data_Receiving=true;
            case 3:           //保存参数
              EEPROM_Save();
              beep=1; //开启蜂鸣器
            break;        
           
            case 4:           //功能开关1
             
            break;    
            
            case 5:           //功能开关2
             
            break;     
            
            case 6:           //功能开关3
             
            break; 
            
            case 7:           //功能开关4
             
            break;        
            
            default:           //
             break;
          }
    }
  predata[1]=predata[0];
  predata[0]=recv;
}


