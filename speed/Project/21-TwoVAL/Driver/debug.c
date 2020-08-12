#include "common.h"
#include "include.h"

float sVariable[20]; //����������
float Variable[20];  //
float Control_Para[15];
float Voltage;
float RunTime;
int   Start_Cnt=0;
uint8 Para_Index_Limit=7;       //һҳ�����7���������
uint8 Page_Index=0,Para_Index=1,Light_Tower_Index=0,Para_Checked=0,OLED_Refresh=0;
uint8 SendPara,stop_contorl,send_data_contorl=0,beep;
float Step[6]={0.0001,0.001,0.01,0.1,1.0,10.0};   //Ĭ�ϵ��ڲ���Ϊ0.01
unsigned char Step_Index=2;
//��Ӧ��ͬ��ҳ��

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
      uart_putchar(UART0,temp); //����ʵ�ʵĴ��ں����޸�
      
}


void ShiBoQi()    //ÿ��ͨ��2�ֽڣ����ֽ���ǰ�����ֽ��ں�
{
  my_putchar(0xff);
  my_putchar(0xff);
//ͨ��1  
  my_putchar(0x10);//0001 0000=16
  my_putchar(0x00);
//ͨ��2  
  my_putchar(0x00);
  my_putchar(0x01);
//ͨ��3
  my_putchar(0x07);
  my_putchar(0xd0);

//ͨ��4
  my_putchar(0x00);
  my_putchar(0x00);

}





/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()                 //��δ����
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}

void Variable_update()//��������while(1)�б�����
{           //������鸳ֵ�����Send_Variable()�б����͸���λ����Ȼ��Send_Variable()��δ����
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
void Para_Update()//  function.c��Para_Init()����7������������ֵ//�ڶദ������ ͨ����ť�޸Ĳ��������
{ 
  SetSpeed=Control_Para[0];
  Fuzzy_Kp=Control_Para[1];
  Fuzzy_Kd=Control_Para[2];
  PID_SPEED.P=Control_Para[3];
  PID_SPEED.I=Control_Para[4];
  PID_TURN.P=Control_Para[5];
  PID_TURN.D=Control_Para[6]; 
}


void OLED_Draw_UI()  //��������
{ 
   uint8 i;
   if(Page_Index<=1) 
   {
     Voltage=adc_once(ADC1_SE6a,ADC_12bit);
     Voltage=Voltage*Vol_ratio; //ת��Ϊʵ�ʵ�ѹ
     OLED_P6x8Str(0,0,"Voltage=");                          //��ʾ��ص�ѹ
     OLED_PrintValueF(48, 0,Voltage,2);                     
     OLED_PrintValueF(72, 0,Step[Step_Index],5);            //��ʾ���ڲ���ֵ
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
     OLED_P6x8Char(Page_Index+48);                         //д��ҳ�����
   }
  /////////////////////////////////////////////////////////��1ҳ  PID����
  if(Page_Index==0)                
  {
    for(i=0;i<7;i++)
    {
      if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name[i]);   //����������ת��ʾ
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
    /////////////////////////////////////////////////////////��2ҳ ״̬��ʾ
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
  ////////////////////////////////////////////////////////////��3ҳ ��ʾ����
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
  ////////////////////////////////////////////////////////////��4ҳ ��������ʾ
  else if(Page_Index==3) 
  {
    OLED_Draw_camera();
    
    OLED_P6x8Str(90,6,"RT");
    OLED_PrintValueF(110, 6,RoadType,4);
   
    OLED_Set_Pos(122,7);
    OLED_P6x8Char(Page_Index+48);
  }
  ///////////////////////////////����ҳ ��ʾ����
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
///����ҳ����
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
 * �����뿪�ص�ֵ
 */
void Read_Switch() //0��ʾON(��) 1��ʾOFF(��)
{
  if(gpio_get(SW1)==0)  //���뿪��1����   
  {
    Style=1; ///////////////////7��7��ע��
  }
  else           
  {
    
  }

  if(gpio_get(SW2)==0)   //���뿪��2����
  {
    //SetSpeed=0;
  } 
  else 
  {
    //SetSpeed=0.72;
  }

  if(gpio_get(SW3)==0)   //���뿪��3����
  {
    //SetSpeed=0;
  } 
  else 
  {
    //SetSpeed=0.2;
  }

} 

/*
 * ��ⰴ���Ƿ���
 */
void Check_BottonPress()                        //������while(1)�б�����
{
      //��ʾ����
  
  if(OLED_Refresh==0&&BT_YES_IN==0)             //����ʾ���ر��Ұ�������ʱ
  {
   if(BT_YES_IN==0)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_YES_IN==0)
      {    
        if(OLED_Refresh==false)                 //false=0��true=1
        {
         OLED_Init();
         OLED_Refresh=true;                     //��������while(1)��ÿ��ѭ������ʾOLED
        }
        //else
        //{
        //  OLED_Refresh=false;
        //  OLED_Fill(0x00);       
        //}
      }
      while(BT_YES_IN==0);                      //ֱ�������ɿ�������
      DELAY_MS(30);
   } 
  }
   //����1 yes
   if(BT_YES_IN==0&&OLED_Refresh)               //��������������ʾ����ʱ
   {
     //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_YES_IN==0&&OLED_Refresh)
     {    
       if(Para_Index==7) 
       { //������Ϊ7ʱ�����°�ť����
         EEPROM_Save();
         Para_Index=0; 
       }
       else
       {//Para_Index��Ϊ7��ҳ����0��1 �Ҳ���δ��ѡ�У���ѡ�в���
         if(Para_Checked==false&&((Page_Index==1)||(Page_Index==0))) Para_Checked=true;
         else Para_Checked=false;       

       }
      }
      if(Page_Index==1)
      {
         Page_Index=0;
         OLED_Refresh=false;                    //OLED������ҳ��Ϊ1ʱ���°�ť�ر�OLED
         OLED_Fill(0x00);                       //����
      }
      
      while(BT_YES_IN==0);                      //ֱ�������ɿ�������
   }
   //����2 Left_L
   if(BT_LEFT_IN==0)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_LEFT_IN==0)
      {
        if(Para_Checked) 
        {//������ѡ��ʱ����������޸Ĳ�����
          if(Step_Index==5) 
          Step_Index=5;                         //���ڲ��������Ϊ5 ��Ӧ���Ĳ���10
          else Step_Index++;
        }
        else 
        { 
          Para_Index=0;
          if(Page_Index==0) Page_Index=5;       //������û��ѡ�е�ʱ�򣬰����Ҽ���ҳ
          else Page_Index--;
          OLED_Fill(0);//���� 
        }
      }
      while(BT_LEFT_IN==0);                     //ֱ�������ɿ�������
   } 
   //����6 Right_L
   if(BT_RIGHT_IN==0&&OLED_Refresh)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_RIGHT_IN==0)
      {
        if(Para_Checked) 
        {
          if(Step_Index==0) 
           Step_Index=0;                        //��������С��0����Ӧ��С�Ĳ���0.0001
          else
          {
            Step_Index--;
          }
        }
        else 
        { 
          Para_Index=0;
          if(Page_Index==5) Page_Index=0;       //������û��ѡ�е�ʱ�򣬰����Ҽ���ҳ
          else Page_Index++;
         OLED_Fill(0);//���� 
        }
      }
      while(BT_RIGHT_IN==0);                    //ֱ�������ɿ�������
   }
   //����3 up
    if(BT_UP_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_UP_IN==0)
      {
   
          if(Para_Checked==false)
          {//����δ��ѡ��ʱ���ϼ����޸Ĳ�����
           if(Para_Index==0) Para_Index=Para_Index_Limit;
           else Para_Index-=1;
          }
          else
          {//������ѡ��ʱ���ϼ����޸Ĳ���
              if(Page_Index==0&&Para_Index<=6)                    //�޸ĵ�0ҳ�Ĳ���
            {
              Control_Para[Para_Index]+=Step[Step_Index];
            }
            
            if(Page_Index==1&&Para_Index<=6)                    //�޸ĵ�1ҳ�Ĳ���������һҳ���������޸�Ҳ�޷��޸�
            {
              Control_Para[Para_Index+7]+=Step[Step_Index];
            } 
            Para_Update();
          }
      }  
      while(BT_UP_IN==0);                       //ֱ�������ɿ�������  
   }
   //����4 down
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
          {//��ʾ�����Ҳ���δ��ѡ��ʱ���¼����޸Ĳ�����
            if(Para_Index==Para_Index_Limit)Para_Index=0;          //��ֹ��ų�����Χ
            else  Para_Index+=1; 
          }
           else 
           {//��ʾ�����Ҳ�����ѡ��ʱ���¼����޸Ĳ���
              if(Page_Index==0&&Para_Index<=6)                    //�޸ĵ�0ҳ�Ĳ���
            {
              Control_Para[Para_Index]-=Step[Step_Index];
            }
             
             if(Page_Index==1&&Para_Index<=6)                    //�޸ĵ�1ҳ�Ĳ���������һҳ���������޸�Ҳ�޷��޸�
            {
              Control_Para[Para_Index+7]-=Step[Step_Index];
            }
            Para_Update();
          }
      }
     }
     else
     {//��ʾ��û��ʱ���¼�      
        if(Stop)  //�����ǰֹͣ��׼������  stop=1��ʾֹͣ
       { 
         Start_Cnt=1000;
         Starting=true;
         //�������ٶȿ��Ʊ�������
         SpeedControlOutOld=0;
         SpeedControlOutNew=0;
         SpeedControlIntegral=0;  
         PID_SPEED.OUT=0;
         RunTime=0;
         Distance=0;
         ControlSpeed=0;
       }
     }
     while(BT_DOWN_IN==0);                              //ֱ�������ɿ�������
   }
   
}


void Send_Variable()                                    //��δ����  ����λ�����ͱ���
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




void Modify_Parameter(uint8 *buff)                      //������λ������UART0_RX_IRQHandler()�б�����
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num=14; //14���ɸĲ���
  /*          �޸Ĳ�������         */
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



void Send_Parameter()//��δ����
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=14;  //14���ɸĲ���
  
 
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
    my_putchar(0X02);//֡β
}

void UART0_RX_IRQHandler()
{
  static uint8 recv;
  static uint8 data_cnt=0;
  static uint8 predata[10];
  static uint8 Recv_Buff[100];
  static uint8 Data_Receiving=false;
  
  if(uart_query(UART0)==1)  uart_getchar (UART0,(char*)(&recv));  //����ʵ�ʵĴ������޸�
  /**********�������ڽ���������λ���Ĳ�������*********/
  if(Data_Receiving)
  {
      if(data_cnt<56)
      {
       Recv_Buff[data_cnt]= recv;
       data_cnt++;
      }
      else
      {
        data_cnt=0;    //�ﵽ֡��
        Data_Receiving=false;
        if(recv==2)  //֡β
        {
           Modify_Parameter(Recv_Buff);
           SendPara=1;      //�����ش���ȷ�ϲ����޸����
            beep=1; //����������
        }
      }
  }

  
  
    if( predata[1]==0x55&&predata[0]==0xAA)
    {
      
        switch(recv)         //�жϹ�����
         { 
            case 1:           //��ȡ���� 
               if(SendPara==0) SendPara=1;
                beep=1; //����������
              break;
            
            case 2:           //�޸Ĳ���
              data_cnt=0;
              Data_Receiving=true;
            case 3:           //�������
              EEPROM_Save();
              beep=1; //����������
            break;        
           
            case 4:           //���ܿ���1
             
            break;    
            
            case 5:           //���ܿ���2
             
            break;     
            
            case 6:           //���ܿ���3
             
            break; 
            
            case 7:           //���ܿ���4
             
            break;        
            
            default:           //
             break;
          }
    }
  predata[1]=predata[0];
  predata[0]=recv;
}


