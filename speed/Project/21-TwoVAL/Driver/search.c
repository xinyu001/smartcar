/*�߼������޸�*/
/*

*/
#include "include.h"
#define SlopeLeftDiff 0.3               //0.4
#define SlopeRightDiff 0.3
extern  PID PID_SPEED,PID_TURN;
extern uint8 flag2;
float SlopeRight[6]={0},SlopeLeft[6]={0};
float Distance_0,Distance_1,Distance2,Distance4,Distance5,Distance6,Distance7;
float Distance14,Distance15,Distance16,Distance17;
float Distance150=0;
float Distance13=1000;
float Distance200;                                      //s ���ʱ�����ж�
float Distance3=1000,Distance8=1000;
float Distance18=1000;
float distance_test;
uint8  RoadType=0;                                      //·����־��Ĭ��0��1ʮ��·��
float Previous_Error[10];
int vol0=0,Display1=0,Display2=0,Display3=0;
int Display4=0,Display5=0,Display6=0;

int vol1;
int flag,flag_obstacle;
extern int flag_cricle_right,flag_cricle_left;         //��Ÿ�Ӧ����
extern int Go_Out_Circle,circle_Flag;
int flag_right,flag_left,flag_3,flag_4,flag_13,flag_14,flag_15,flag_7,flag_17;
int flag_16,flag_6,flag_5;
extern uint8 Style;                             
extern uint8 flag_jump;
extern int Stop_Brake;
extern int NitroBooster;
extern float CarSpeed,AverageSpeed;

uint16 edgposition[CAMERA_H];
uint8 mid_line=35,left_line=0,right_line=70;            //mid_lineԽС��С��Խ����
uint8 lost_line;
uint16 cont;

int jishu1,jishu2,jishu,jishu3;
int count;
void roadturncal();

void Push_And_Pull(float *buff,int len,float newdata)
{//�µ����ݷ����һ��Ԫ�أ�����Ԫ���������
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}
   

void get_edge()   //�������ٳ˷�����
{

  static int16 i=0,j=0,last=0,x=0,y=0,n=0;
  uint8 temp=0,find=0;
  cont=0;
  for(i=0;i<60;i++)
  {
    last=0; 
    x=i*10;                                     //����
    find=0;                                     //ÿһ�еı�����ֻҪ����һ������Ͱ�find�ĳ�1    
    edgposition[i]=0;                           //16λ�޷����������飬����Ϊͼ��ĸ�
    for(j=0;j<10;j++)                           //�ֽ�����; �ò�ѭ���൱����һ���ֽ�һ���ֽڵر���
    {
      if(imgbuff_process[x+j]==0xff)            //8λ�޷�������һά���飬����Ϊ60��*��80/8����
      {                                         //������ֽ�ȫ1���ף�
        if(last==0)
        {                                       //������Ǹ��е�һ���ֽڻ��߸�������һ����0���ڣ�
          y=j<<3;                               //���ֽ�����*8=��¼������һ����������
          if(find==0)
          {                                     //����Ǹ��е�һ������
            edgposition[i]=cont;
          }
          img_edg[cont++]=y;                    //���������λ�ã�������һ��������������¼�ڸ�����
          find=1;
        }
        last=1;                                 //�����һ�����˴�����1
        continue;                               //������һ���ֽ�
      }
      if(imgbuff_process[x+j]==0)
      {                                         //������ֽ�ȫ0���ڣ�
        if(last==1)
        {                                       //���������һ����1���ף�
          y=j<<3;
          if(find==0)
          {                                     //����Ǹ��е�һ������
            edgposition[i]=cont;
          }
          img_edg[cont++]=y;                    //���������λ�ã�������һ��������������¼�ڸ�����  
          find=1;
        }
        last=0;                                 //�����һ�����˴�����0
        continue;                               //������һ���ֽ�
      }      
      for(n=7;n>=0;n--)
      {                                         //������ֽڲ���ȫ0��ȫ1�Ļ����ʹ�ǰ����һλһλ��������ֽ�
        temp=(imgbuff_process[x+j]>>n)&1;       // ��ȡ�õ�����ֵ ��0��1��     
        if(temp!=last)                          //����һ��ֵ����ͬ ������������            
        {
           y=j<<3;  
           if(find==0)
           {                                    //����Ǹ��е�һ������
            edgposition[i]=cont;
           }
           img_edg[cont++]=y+7-n;               //���������λ�ã�������һ��������������¼�ڸ�����
           find=1;
        }
        last=temp;                              //�洢�õ��ֵ
      } 
    } 
    img_edg[cont++]=0xff;                       //����λ�õ����������ܴﵽ11111111 

  }
}
/*img_edg��һ��һά���� ��¼������ͷÿ�е������ص�����ֵ  ÿ�������� �������ؿ�ʼ���ɺڱ�ף�Ȼ����½��أ��ɰױ�ڣ� ����������ֵ����������������������Ԫ��
   0xff����ָʾ���е����������ˣ���ʼ��¼��һ��
   ���ÿ��ͼ���԰�ɫ���ֿ�ʼ����ô������������ʼλ��Ϊ0
   �������ȫ�� ��ô���м�¼Ϊ0xff
   �������Ϊȫ�� ��¼Ϊ 0 0xff
 
   
  oxff����������ֵ������������һ��
 //edgposition[i]�����i�е� ��һ�������� �� img_edg ���������

 */
void judgeblack()
{
  static int16 n=0,m=0;
  int16 sum[5];  //15-19��
  int16 sum1[5];  //0-4��
  int16 sum2[5];  //55-59��
  int16 whitenum;
  for(n=0;n<5;n++)  sum[n]=0;
  
  for(n=15;n<20;n++)
  {
    for(m=0;m<10;m++)
    {
      whitenum=((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
      sum[n-15] = sum[n-15] + whitenum;
    } 
  }
  
  if(flag==0)
  {
  if( (sum[0]<4) || (sum[1]<4) || (sum[2]<4) || (sum[3]<4) || (sum[4]<4) )  //5��ȫ�ڣ�ÿ�����4���׵㣩
  { RoadType=100;  //�л������
  uart_putchar(UART0,'D');}
  }
  
  for(n=0;n<5;n++)  sum1[n]=0;
    for(n=0;n<5;n++)
  {
    for(m=0;m<10;m++)
    {
      whitenum=((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
      sum1[n] = sum1[n] + whitenum;
    } 
  }
  
  for(n=0;n<5;n++)  sum2[n]=0;
  
  for(n=55;n<60;n++)
  {
    for(m=0;m<10;m++)
    {
      whitenum=((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
      sum2[n-55] = sum2[n-55] + whitenum;
    } 
  }
  
  
 /* if(RoadType==50)
  {
    if( (sum1[0]<4) || (sum1[1]<4) || (sum1[2]<4) || (sum1[3]<4) || (sum1[4]<4) ||  (sum[0]<4) || (sum[1]<4) || (sum[2]<4) || (sum[3]<4) || (sum[4]<4))  //5��ȫ�ڣ�ÿ�����4���׵㣩
  { flag=1; //ʶ���ϰ�
 // uart_putchar(UART0,'D');
  }
  }*/
  
  
  if(RoadType==100)                             // �ӵ���л�����ͨ��������
  {
      if((sum[0]<60) && (sum[1]<60) && (sum[2]<60) && (sum[3]<60) && (sum[4]<60) && (sum[0]>30) && (sum[1]>30) && (sum[2]>30) && (sum[3]>30) && (sum[4]>30))
      {
          if((sum2[0]>65) && (sum2[1]>65) && (sum2[2]>65) && (sum2[3]>65) && (sum2[4]>65))
          {
            RoadType=0;
          }
      }
  }

}






void Search()
{
  //�ӵײ���������
      
      //judgeblack();                           //�л����

//      Display1=adc_once(ADC1_SE8, ADC_16bit);
//      Display2=adc_once(ADC1_SE9, ADC_16bit);
//      Display3=adc_once(ADC1_SE10, ADC_16bit);
//      Display4=adc_once(ADC1_SE11, ADC_16bit);
//      Display5=adc_once(ADC1_SE12, ADC_16bit);
//      Display6=adc_once(ADC1_SE13, ADC_16bit);
      

  
  float Middle_Err_Sum=0,slope;                         //Middle_Err_Sum��һ�е����
  static int i,j,find;                                  //find��ͨ���־��1��ʾ������ͨ��
  uint8 left_cont=0,right_cont=0;
  
  uint8 Stop_Change=0,Stop_White=0,Stop_Black=0;        //���������
  static int k;
  
  int leftfind=0,rightfind=0;                         
  int AllWhileStartLine=0,AllWhileEndLine=0;            //�ֱ���ͼ���д������ϵ�һ�������һ��ȫ���У�����ƴд����
  
  static int break_line_left=0,break_line_right=0,continue_line_left=0,continue_line_right=0;
              //���ұ��߶���ǰ�����һ��            //���ұ��������еĵ�һ��
  uint8 line_i,line_j,line_con,line_con3,line_58,line_59;
  static int last_err;
  
  int search_end_line=0;
  for(i=0;i<60;i++)                             //�������(�����鶨����camera.c 8λ�޷�������)
  {  
        LMR[0][i]=0;                            //���������
        LMR[1][i]=0;                            //��������
        LMR[2][i]=80;                           //�ұ�������
  }
  leftfind=0;
  rightfind=0;
  break_line_left=0;
  break_line_right=0;
  continue_line_left=0;
  continue_line_right=0;
  
  for(i=59;i>0;i--)                             //�ӵ�59�п�ʼ���ߣ�Ŀ�����ܽ����ͼ��
  {
 
   
    //������
   if(i==30 &&  Distance>50 && (RoadType==0 || RoadType==2 || RoadType==12 ))  //Distance>50����֤�����ڵڶ��ξ���������ʱ��ִ��  ԭi==55
    {

        for(j=1;j<8;j++)
        {
          for(k=7;k>=0;k--)
          {
             if(Stop_Change==0)
             {
               if(((imgbuff_process[i*10+j]>>k)&0x01)==1)
               {
                 Stop_Change=1;
                 Stop_White++;
               }
             }
             if(Stop_Change==1)
             {
               if(((imgbuff_process[i*10+j]>>k)&0x01)==0)
               {
                 Stop_Change=0;
                 Stop_Black++;
               }
             }
          }
        }
        
        if((Stop_Black+Stop_White)>6)                   //s ԭ8����Ϊ����ͷ�߶ȴ�20cm��Ϊ10cm������������������
        {
          
          RoadType=205;                                 //s 8.1 ���������ߣ�׼�����
          Distance200=Distance;                         //��¼��ǰ����,�Ӷ��ж�ɲ��
          SetSpeed=0.2;
//          Speed_H=0;
//          Speed_M=0;
//          Speed_L=0;
        }
        Stop_Black=0;
        Stop_White=0;

        
    }    
    

    if(edgposition[i]==0&&(i!=0))                       //��ʾ�Ӹ��п�ʼ����ͼ��ȫ�� ��Ϊ����  lost_line=0��������ת��
    {
      if(i>24)
      {                                                 //����������µ�25�вſ�ʼ����
        if(last_err>0)
        {
           lost_line=1;                                 //ʹcontrol.c���ö������
        }
        if(last_err<0)
        {
           lost_line=2;                                 //ʹcontrol.c���ö������
        }
      }
//      if(i<23)
//      {
//         lost_line=0;
//      }
      
      
      break;                                            //��������ѭ��for,��������ȫ�ڣ�����Ҫ����
    }
    j=edgposition[i];                                   //jΪ��i�е�һ��������������img_edg[]�е��±� j����ڱ�� ������� j+1������ ���ұ��� 
    if(i==59)                                           //�ײ���ʼ��
    {                                                   //���while��������ѡ�����һ���бȽ�׼ȷ������������ֵ���ұ��ߣ����ų�����������
        while(img_edg[j]!=255)                          //img_edg[j]�����i�и�������(�ڱ��)�����꣨��������0~79��
       {  //255=0xff ����������ֵ������������һ��
          if((img_edg[j]<55)&&(img_edg[j+1]>25))        //�����С��55 �ұ��ش���25(����̫�������»��߸���ȫ�ף�
          {
            if((img_edg[j+1]-img_edg[j])>25)            //�ұ���-����ش���20
            {
              LMR[0][i]=img_edg[j];                     //��ֵ��������һ��Ϊ��Ӧ������
              if(img_edg[j+1]==255)
              {                                         //������������һ�����䣨�ױ�ڣ�
                 LMR[2][i]=80;                          //��ֵ�ұ������һ��  
              }
              else
              {                                         //������������һ�����䣨�ױ�ڣ�
                 LMR[2][i]=img_edg[j+1];                //��ֵ�ұ������һ�� 
              }
              break;                                    //����whileѭ����Ҳ�ͽ�����if��i==59)��������һ������forѭ��
            }
          } 
          if(img_edg[j+1]==255)
          {
            break;                                      //����whileѭ����Ҳ������if��i==59)��������һ������forѭ��
          }
          j=j+2;                                        //����jȻ�������һ��whileѭ�������������ײ��е�����������е�����˵���п���������
       }
       
    //  if(LMR[0][i]==0)  break_line_left=59;
     // if(LMR[2][i]==80) break_line_right=59;
    }
    else                                                //���ǵײ���ʼ�У�������59��
    { 
        find=0;                                         //��ͨ���־����Ϊ0 ��ʾû��������ͨ��
        while(img_edg[j]!=255)                          //img_edg[j]�����i�и�������(�ڱ��)�����꣨��������0~79�У�ѭ������������Ϊ���˸��еĽ�����־0xff��
       {
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j])>8)     
        {//������������С�ڵ�����һ�е��ұ��� &&�����ұ��ش��ڵ�����һ�е������ && ����ͨ��
          find=1;                                       //��ͨ���־��1��ʾ������ͨ��
          if(LMR[0][i]==0&&(img_edg[j]!=0))
          {                                             //�����������߻�δ��ֵ���������˸��������أ��ڱ��)
            if(break_line_left!=0&&(continue_line_left==0))//��ʾ����ѭ��������ʮ��·ȫ������ǰ�����У���Ҫ��һ���ж�
            {                                           //ʮ��·�ڵ�����£�����߶���ǰ�����һ���м�¼���������������λ��û�м�¼
              if(img_edg[j]>LMR[0][break_line_left]&&img_edg[j]<55)
              {                                         //�жϸ������أ��ڱ��)��û�п���������ߵ����ߣ�����ԶС��
                LMR[0][i]=img_edg[j];                   //��ֵ��������ߵ�����
                continue_line_left=i;                   //��������������У�ÿһ��ͼƬ����ִ��һ�Σ�
                leftfind=1;
              }
            }
            else
            {//�����������߻�δ��ֵ���������˸��е�һ�������أ��ڱ��)���ң��������δ���߼�û��ʮ��·��������ߵĶ��ߴ������ߴ����ҵ��ˣ���
                if(LMR[2][i]==80)
                {                                       //��������ұ��߻�δ��ֵ
                 LMR[0][i]=img_edg[j];                  //��ֵ�����Ϊ���е�һ��������(�ڱ��)�����꣨������
                 leftfind=1;
                }
            }
          } 

          if(img_edg[j+1]!=255&&(LMR[2][i]==80))
          {                                             //��������ұ��߻�δ��ֵ�ҿ��õ����еڶ��������أ��ױ�ڣ�
            if(break_line_right!=0&&(continue_line_right==0)&&(img_edg[j+1]>20))////��ʾ����ѭ��������ʮ��·ȫ������ǰ�����У���Ҫ��һ���ж�
            {//ʮ��·�ڵ�����£��ұ��߶���ǰ�����һ���м�¼�������ұ�������λ��û�м�¼
              if(img_edg[j+1]<LMR[2][break_line_right])
              {                                         //�жϸ������أ��ڱ��)��û�п������ұ��ߵ����ߣ�����ԶС��
                LMR[2][i]=img_edg[j+1];                 //��ֵ�����ұ��ߵ�����
                continue_line_right=i;                  //��¼�ұ��������У�ÿһ��ͼƬ����ִ��һ�Σ�
                rightfind=1;
              }
            }
            else
            {
                if((RoadType==1))                       //ʮ��·�ڱ�־����ֵΪ0��ʾδ����
                 {
                   if(img_edg[j+1]>30)
                   {
                    LMR[2][i]=img_edg[j+1];
                    rightfind=1;
                   }
                 }
                else
                {
                   LMR[2][i]=img_edg[j+1];              //��ֵ�ұ���Ϊ���еڶ���������(�ױ��)�����꣨������
                   rightfind=1;
                }
            }
          }
        }                                               //��ͨ���жϽ���
        if(img_edg[j+1]==255)                           //���е������ؽ�����
        {
          if(img_edg[j]==0)                             //˵������Ϊȫ�� 
          { 
            if(AllWhileStartLine==0)
            {                                           //˵���ǵ�һ������ȫ����
              AllWhileStartLine=i;                      //��¼ȫ���п�ʼ����������Ϊ�ǵ�����������ʵ��������ײ���ȫ���У�
            }
            AllWhileEndLine=i;                          //��ʱ�����м�¼Ϊȫ���н���������

            if(/*(rightfind&&leftfind&&(RoadType==0)&&i>20)||*/(AllWhileStartLine-AllWhileEndLine)>40)
              //�ң������߱��߶��ҵ��� ���ų�����20�еĸ��ţ�������������12�е�ȫ���У���  10
            {
                if(RoadType==0) RoadType=1;             //����ʮ����
            }
          }
          break;                                        //����while����Ϊ���������ؽ�����
        }
        
        
        j=j+2;                                          //ÿһ�п��ܻ��кܶ������ֵ
       }
                                                        //������������ͨ����㷨
       
       
       
       if(RoadType==1)                                  //�����ǵײ���ʼ�е�ǰ���£���ʮ���ڶ����߽������⴦����¼���ұ��߶���ǰ�����һ�У�
       { 
         if(left_cont>=4)
         {                                              //���������Ĵ��ҵ������
           if(((LMR[0][i]<(LMR[0][i+1]-1))||(LMR[0][i]==0))&&i>20)
           {                                            //�����������չ���ų���ѭ����ʮ��·��ǰ�����е���������߶���
                LMR[0][i]=0;
                if(break_line_left==0)break_line_left=i+1;         //��¼����߶���ǰ�����һ�У�ǰ����ô��if��Ҫ��Ϊ�˱������ʮ��·�ں������ظ�ִ�У����������Ƕ��ߺ�ĵ�һ�в�ִ�е����
           }
         }
         if(LMR[0][i]!=0)                               //�ҵ�����߱��߾͸����������1
         {
           left_cont++;       
         }
         else
         {
           left_cont=0;               //������ʮ��·�ڣ���ѭ����������ʮ��·���ڣ���ִ�У�Ϊ�˱��������һ��if
         }
         
         if(right_cont>=4)
         {
           if(((LMR[2][i]>(LMR[2][i+1]+1))||(LMR[2][i]==80))&&i>20)
           {                                            //�ұ���������չ���߶���
                LMR[2][i]=80;
                 if(break_line_right==0)break_line_right=i+1;       //��¼�ұ��߶���ǰ�����һ�У�ǰ����ô��if��Ҫ��Ϊ�˱������ʮ��·�ں������ظ�ִ�У�
           }
         }
         if(LMR[2][i]!=80)                              //�ҵ����ұ���
         {
           right_cont++;
         }
         else
         {
           right_cont=0;
         }
       }
     
       
       if(find==0)                                      //�����ǵײ���ʼ�е�ǰ���£�û���ҵ���ͨ����
       {
         search_end_line=i;                             //��¼���Ǵӵ����ڶ��п�ʼ���������ĵ�һ��û����ͨ�������
         break;         //��������ѭ��for�����ַ�ʽ����ѭ���Ļ���iΪ�����������һ������ͨ������������һ��
       }
  }
  
 }                                                      //����forѭ������

 

//s ���⣬��ʼ������ʻ
if((RoadType==200)&&(Distance>1)){
        
        RoadType=0;
//  Speed_H=0.5;                                 
//  Speed_M=0.5;                                 
//  Speed_L=0.4;                                
  //      SetSpeed=0.52;

}
//if(RoadType==0 && Distance>3 && Distance150==0 &&
//   LMR[0][1]<15 && LMR[0][2]<15 && LMR[0][3]<15 && LMR[0][4]<15 && LMR[0][5]<10 && LMR[0][6]<10 && LMR[0][7]<10 && LMR[0][8]<10 && LMR[0][9]<10 && LMR[0][10]<10 && LMR[0][11]<5 && LMR[0][30]<5 && LMR[0][40]<5 && LMR[0][50]<5 &&         
//   LMR[2][1]>65 && LMR[2][2]>65 && LMR[2][3]>65 && LMR[2][4]>65 &&LMR[2][5]>70 && LMR[2][6]>70 && LMR[2][7]>70 && LMR[2][8]>70 && LMR[2][9]>70 && LMR[2][10]>70 && LMR[2][11]>75 && LMR[2][30]>75 && LMR[2][40]>75 && LMR[2][50]>75){               //�ж��µ�
//  RoadType=150;
//  Distance150=Distance; 
// // NitroBooster=1;
////  Speed_H=0.7;                                 
////  Speed_M=0.6;                                 
////  Speed_L=0.4;                                
////  SetSpeed=0.7;
//}
//if((Distance-Distance150>4) && RoadType==150){                     //�µ�����
//  RoadType=0;
//  Distance150=1000;
//  Stop_Brake=1;
//  
////  SetSpeed=0.5;
////  Speed_H=0.5;                  //�ָ������ٶ�                                
////  Speed_M=0.5;                                 
////  Speed_L=0.4;                                
////  SetSpeed=0.5;
//  
//}

//s ��������ͣ��
if((RoadType==205)&&(Distance-Distance200>2.5)){
       Distance200=1000;
       RoadType=206;
       Stop_Brake=1;                    //ɲ��
       SetSpeed=0;
       Speed_H=0;
       Speed_M=0;
       Speed_L=0;
}


 
//���¶�ʮ��·�ڽ��в��߲����жϲ�ͬ��·��
 if(RoadType==1)                //��һ����׼���������ߵ������е�����Ȼ���ߣ�Ϊɶ�������жϷ�ʽ���в�ͬ������
 {//ʮ��·��
   if(AllWhileEndLine==0||AllWhileEndLine<20)RoadType=0;//��ʾ���һ��ȫ���л�Զ
   if(continue_line_left!=0)
   {
       for(i=continue_line_left-1;(i>continue_line_left-10)&&(i>0);i--) //�����п��������� ���һ�� 
      {
        if((LMR[0][i]!=0)&&(LMR[0][i-1]!=0))
        {
          if(ABS(LMR[0][i]-LMR[0][i-1])<2)
          {
              if(ABS(LMR[0][i-1]-LMR[0][i-2])<2)
            {//�����������������Ϸ����������������е�����ߵ�֮��С��2�����������������������ж�����
              continue_line_left=i;//������������и�Ϊ��С������
              break;
            }
          }
        }
      }
      
      slope=(LMR[0][break_line_left]- LMR[0][continue_line_left])*1.0/(break_line_left-continue_line_left);//���������б�� Ӧ���Ǹ��ģ�x/y��(����߶����ж�Ӧ������-����������ж�Ӧ��������/��������-�����У�
      for(i=break_line_left;i>=continue_line_left;i--)                  //��ʼ����
      {//б�ʼ�������Ż�
        LMR[0][i]= LMR[0][break_line_left]-(int)((break_line_left-i)*slope);//����߶����ж�Ӧ����������һ������
      }
   }
   
   if(break_line_right!=0&&continue_line_right!=0)
   {
       for(i=continue_line_right;(i>continue_line_right-10)&&(i>0);i--) //�����п��������� ���һ�� 
      {
        if((LMR[2][i]!=0)&&(LMR[2][i-1]!=0))
        {
          if(ABS(LMR[2][i]-LMR[2][i-1])<2)
          {
            continue_line_right=i;//���ұ��������и�Ϊ��С������
            break;
          }
        }
      }
      
      slope=(LMR[2][break_line_right]- LMR[2][continue_line_right])*1.0/(break_line_right-continue_line_right);//�����ұ���б�� Ӧ�������ģ�x/y��(�ұ��߶����ж�Ӧ������-�ұ��������ж�Ӧ��������/��������-�����У�
      for(i=break_line_right;i>=continue_line_right;i--)                //��ʼ����
      { 
         LMR[2][i]= LMR[2][break_line_right]-(int)((break_line_right-i)*slope);//�ұ��߶����ж�Ӧ��������ȥһ������
      }
      
   }   
 }


 //roadturncal();                          //��ż�⻷��
 SlopeLeft[0]=(LMR[0][15]-LMR[0][18])/3.0;
 SlopeLeft[1]=(LMR[0][20]-LMR[0][23])/3.0;
 SlopeLeft[2]=(LMR[0][25]-LMR[0][28])/3.0;
 SlopeLeft[3]=(LMR[0][30]-LMR[0][33])/3.0;
 SlopeLeft[4]=(LMR[0][35]-LMR[0][38])/3.0;
 
 
 

 if(RoadType==0)                                        //�ж�������Ƿ�Ϊֱ��
 {
   if(SlopeLeft[0]!=0 && SlopeLeft[1]!=0 && SlopeLeft[2]!=0 )
   {
     if(SlopeLeft[0]<4 && SlopeLeft[1]<4 && SlopeLeft[2]<4)
     {       
       if(ABS(SlopeLeft[0]-SlopeLeft[1])<1.4*SlopeLeftDiff && ABS(SlopeLeft[1]-SlopeLeft[2])<1.4*SlopeLeftDiff)
       {
         RoadType=2;                                    //�����Ϊֱ��
        // uart_putchar(UART0,'L');
       }
     }
   }
 }
 if((RoadType==2 || RoadType==12 || RoadType==0)&& Distance>500 && flag_right==0)//|| RoadType==0)  
 {   
   roadturncal();                                     
   if(ABS(SlopeLeft[0]-SlopeLeft[1])>1.4*SlopeLeftDiff || ABS(SlopeLeft[1]-SlopeLeft[2])>1.4*SlopeLeftDiff )
      RoadType=0;  
   
   else if (Go_Out_Circle==0 && flag_cricle_right==1 )
   { RoadType=3;                                        //������һ�����Ҷ���·��  //32   34
     // flag_3=1;                                         //�һ���һ��·�ڱ�־

    Distance3=Distance;   
   }  
 }
 
 if(RoadType==3)   
 {
  
   RoadType=4;
 }
 if(RoadType==4)  
 {
                                        
  if(LMR[0][10] == 0 && LMR[0][15] == 0 && LMR[0][20] == 0 && LMR[2][20] == 80 && LMR[2][15] == 80 && LMR[2][10] == 80)
   {
     RoadType=0;
     
   }
  else{
    flag_5=1;
  }
 }

 if(flag_5=1&&(Distance-Distance3>1.1)){
    RoadType=5; 
    flag_right=1;                        //����
    Distance3=1000;
    Distance5=Distance;
 }
if(RoadType==5)
{
  if(Distance-Distance5>1.5)
  {
    RoadType=6;
    flag_6=1;
   // flag_3=0;
   // flag_4=0;
    flag_5=0;
    Distance5=1000;
    Distance6=Distance;//���»�����ʻ�ľ��룬�ж�RoadType7
  }
}
 //���һ�
 //������
if(flag_6==1)
{
  RoadType=6;
  if(flag_right==1 && Distance-Distance6>4.5 )
  { 
    flag_6=0;  
    RoadType=7;
    flag_7=1;
    Distance6=1000;
    Distance7=Distance;     //��¼����ʱ���룬һ��ʱ���ָ�ֱ����ʻ��
    
   }                      
}

//���󻷹���
if(flag_7==1)
{
  RoadType=7;
  if(Distance-Distance7>1.7)
  {
    RoadType=8;
    Distance8=Distance;
    Distance7=1000;
//    flag_right=0;
    flag_7=0;
    Go_Out_Circle=1;
  //  flag_right=0;                     //ע�͵��һ�����ֹ����
//    flag_cricle_right=0; 
//    circle_Flag=0;
    
  }
}
if(RoadType==8 && Distance-Distance8>2  ){
    RoadType=0;
    Go_Out_Circle=0;
    Distance8=1000;
    flag_cricle_right=0;
 //   flag_right=0;

}

 

 
 SlopeRight[0]=-(LMR[2][15]-LMR[2][18])/3.0;
 SlopeRight[1]=-(LMR[2][20]-LMR[2][23])/3.0;
 SlopeRight[2]=-(LMR[2][25]-LMR[2][28])/3.0;
 SlopeRight[3]=-(LMR[2][30]-LMR[2][33])/3.0;
 SlopeRight[4]=-(LMR[2][35]-LMR[2][38])/3.0;  
 

 
 if(RoadType==0)
 {
   if(SlopeRight[0]!=0 && SlopeRight[1]!=0 && SlopeRight[2]!=0 )
   {
     if(SlopeRight[0]<4 && SlopeRight[1]<4 && SlopeRight[2]<4 )
     {
       if(ABS(SlopeRight[0]-SlopeRight[1])<1.4*SlopeRightDiff && ABS(SlopeRight[1]-SlopeRight[2])<1.4*SlopeLeftDiff )
       {
         RoadType=12;                                   //�ұ���Ϊֱ��
       }
     }
   }
 }
// flag_cricle_left=0;
 if((RoadType==12 || RoadType==2 || RoadType==0 )&& (Distance>10 && Distance<25 )&& flag_left==0)//|| RoadType==0)
 {  
   roadturncal();
 
   if(ABS(SlopeRight[0]-SlopeRight[1])>=1.4*SlopeRightDiff || ABS(SlopeRight[1]-SlopeRight[2])>=1.4*SlopeRightDiff )
      RoadType=0;
   
   if (Go_Out_Circle==0 && flag_cricle_left==1 )
   
   {
      RoadType=13;                                        //��ż�⵽��
 //     flag_13=1;                                           //�󻷵�һ��·�ڱ��
      Distance13=Distance;                                //s ���µ�Ž���⵽��ʱ�ľ��룬�ж�RoadType15  
   }
 }
 
if(RoadType==13)      
{
  RoadType=14;
 // flag_14=1;
}
 
 if(RoadType==14)   //s ���һ�ο�����ͨ����һ�ξ���
 {
   if(LMR[0][10] == 0 && LMR[0][15] == 0 && LMR[0][20] == 0 && LMR[2][20] == 80 && LMR[2][15] == 80 && LMR[2][10] == 80)
   {
     RoadType=0;
   }
   
   flag_15=1;
 }
 if(flag_15=1&&(Distance-Distance13>1)){
    RoadType=15; 
    flag_left=1;                        //����
    Distance13=1000;
    Distance15=Distance;
 }
 //�������RoadType16
if(RoadType==15)
{
  if(Distance-Distance15>1.5)
  {
    RoadType=16;
    flag_16=1;
   // flag_13=0;
   // flag_14=0;
    flag_15=0;
    Distance15=1000;
    Distance16=Distance;//���»�����ʻ�ľ��룬�ж�RoadType17
  }
}
 
 //������
if(flag_16==1)
{
  RoadType=16;
  if(flag_left==1 && Distance-Distance16>4.6 )
  { 
    flag_16=0;  
    RoadType=17;
    flag_17=1;
    Distance16=1000;
    Distance17=Distance;     //��¼����ʱ���룬һ��ʱ���ָ�ֱ����ʻ��
    
   }                      
}

//���󻷹���
if(flag_17==1)
{
  RoadType=17;
  if(Distance-Distance17>1.8)
  {
    RoadType=18;
    Distance18=Distance;
    Distance17=1000;
//    flag_left=0;                      //ȡ��ע��ʱ��ʹ��ֻ��һ�λ���
    flag_17=0;
    Go_Out_Circle=1;
    flag_left=0;
//    flag_cricle_left=0; 
//    circle_Flag=0;
    
  }
}
if(RoadType==18 && Distance-Distance18>2  ){
    RoadType=0;
    Go_Out_Circle=0;
    Distance18=1000;
    flag_cricle_left=0;
    
 //   flag_left=0;

}

  
    //�Ҽ���
    if(SlopeLeft[0]>1.3 && RoadType==0){
      if(LMR[0][12]>35 && LMR[0][12]>LMR[0][15]){
        if(LMR[0][15]>25 && LMR[0][15]>LMR[0][20]){
          if(LMR[0][20]>19 && LMR[0][20]>LMR[0][25]){
            if(LMR[0][25]>15 && LMR[0][25]>LMR[0][28]){
              if(LMR[2][5]==80 && LMR[2][7]==80 && LMR[2][10]==80){//��S��
                RoadType=32;//��Ҫ�����Ҽ���
                Distance_0=Distance;
                
              }
            }
          }
        }
      }
    }
    if(RoadType==32 && Distance-Distance_0>1.5 ){
      RoadType=33;//�Ҽ�����
    }
    if(RoadType==33 && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff && SlopeLeft[2]<1.2 && SlopeLeft[3]<1.2){
      RoadType=0;
    }
    

    //����
    if(SlopeRight[0]>1.3 && RoadType==0){
      if(LMR[2][12]<45 && LMR[2][12]<LMR[2][15]){
        if(LMR[2][15]<55 && LMR[2][15]<LMR[2][20]){
          if(LMR[2][20]<61 && LMR[2][20]<LMR[2][25]){
            if(LMR[2][25]<65 && LMR[2][25]<LMR[2][28]){
              if(LMR[0][5]==0 && LMR[0][7]==0 /*&& LMR[0][10]==0*/){//��S��
                RoadType=36;//��Ҫ��������
                Distance_0=Distance;
                left_line=5;//�������ѹ��
              }
            }
          }
        }
      }
    }
    if(RoadType==36 && Distance-Distance_0>1.5 ){
      RoadType=37;//������
      left_line=0;
    }
    if(RoadType==37 && ABS(SlopeRight[2]-SlopeRight[3])<SlopeRightDiff && SlopeRight[2]<1.2 && SlopeRight[3]<1.2){
      RoadType=0;
    }
   
     
 
 Middle_Err_Sum=0;
  for(i=0;i<59;i++)
   {//�������ұ��߷�Χ���޶��ұ���Ϊ70
       if(LMR[2][i]>(right_line))
          LMR[2][i]=(right_line);
       if(LMR[0][i]<(left_line))
          LMR[0][i]=(left_line);

     if(RoadType==5)                                    //���ҵĵڶ������ֿ�
     {
       
          LMR[1][i]=50;
     }
     else if(RoadType==7)                               //���ҳ�����
     {
       
          LMR[1][i]=50;                                 //LMR[2][i]-11;
     }
     else if(RoadType==15)                              //����ʻ�뻷��
     {
       
          LMR[1][i]=20;
     }
     else if(RoadType==16)                              //��������ʻ
     {

       LMR[1][i]=(LMR[0][i]+LMR[2][i])/2;               //����  
          
     }
     else if(RoadType==17)                              //���󻷵�
     { 
       
          LMR[1][i]=24;                                 
     }

//     else if(RoadType==200)                     //s 8.1�������
//     {
//          LMR[1][i]=24;
//     }
//     else if(RoadType==205)                     //s 8.1������
//     {
//          LMR[1][i]=15;
//     }
     else if(RoadType==200)                     //�����ҹ�
     {
          LMR[1][i]=46;
     }
     else if(RoadType==205)                     //����ҹ�
     {
          LMR[1][i]=55;
     }
     else                                               //RoadType=0,6��16...
     {
       LMR[1][i]=(LMR[0][i]+LMR[2][i])/2-1;             //����  -3
     }
   }
  
 
  
  
  for(i=0;i<59;i++)
   {

    if(search_end_line<25)                      //search_end_line ���Ҳ�����ͨ����������ѭ��ʱ������
    {                                           //û������ͨ���л�������ͨ������25������
      if(Style==1)
      {                                         //������
        if(i>=21&i<=23)            //30  32����ͷ���ˣ����Լ������Ҫ��ǰ��ǰհ  '25,27'����ʵͦ��  '21,23'�� 
        {//my_putchar('_');
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
        }//20��21��22�����е����м���֮���ۼӣ��жϳ��ĵ�·�м���-����ͷ��Ӧ���м��ߣ�e.g.С��ƫ��ʱ���Ϊ��
      }
      else{//����
                  
        //if( (RoadType==32 || RoadType==33 || RoadType==36 || RoadType==37) && CarSpeed<0.7){//����
        if (RoadType==0)
        {//����S�����ѹ��

          if(i>=29&&i<=31)
          {
            Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
          }
        }
        else {
          if(i>=28&&i<=30)
          {
            Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
          }
        }
        

      }
    }
    else
    {//�����������ͨ������25�м�����ʱ
      if((i>=27)&&(i<=29))
      {
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;            //ȡ��30��31��32�е����
      }
    }
   }
  
    Middle_Err_Sum=0;
    for(i=22;i<24;i++)
      {
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;            //ȡ���е����
      }
   Middle_Err_Sum=Middle_Err_Sum/3;                                     //ȡƽ��
  
   if(RoadType==1)
   {                    //����ʮ��·�ڵ�����£�Middle_Err_Sumȡȫ����ǰ������λ���ڵ�10�����µ�ĳһ�е����
      for(i=AllWhileEndLine;i>10;i--)
     {                                          //i�����һ��ȫ���У�������ѭ����������11��
       if(LMR[0][i]!=0&&LMR[2][i]!=(mid_line*2))//����ǰ��Ŀ򶨣���ʱ���ұ�����0~70��Χ������ȫ���������������ö������ϣ�ֱ�ӽ�����һ��ѭ��
       {
           if(LMR[0][i-2]!=0&&LMR[2][i-2]!=(mid_line*2))
         {
              Middle_Err_Sum=(LMR[0][i-2]+LMR[2][i-2])/2-mid_line;//����ȡ�������һ��ȫ���е��Ϸ���3�е����
              break;
         }
       }
     }
   }
   
   for(line_i=0;line_i<10;line_i++)
   {
     for(line_j=0;line_j<8;line_j++)//��ԭʼͼ��25��27��58��59�е�ÿ���ֽڵ�ÿһλ����
     {
        if(((imgbuff_process[25*10+line_i]>>line_j)&1)==1)line_con++;
        if(((imgbuff_process[27*10+line_i]>>line_j)&1)==1)line_con3++;
        if(((imgbuff_process[58*10+line_i]>>line_j)&1)==1)line_58++;
        if(((imgbuff_process[59*10+line_i]>>line_j)&1)==1)line_59++;    
     }
   }//���ĸ������ֱ��Ƕ�Ӧ��4����1�ĸ���
   
   if(line_con==0&&line_con3==0)
   {                                                    //����м�λ�õ�������ȫ��
     if(last_err>0)                                     //e.g.С��ƫ��ʱ���Ϊ��
     {
        //lost_line=1;                                  //ʹcontrol.c���ö�����ң�
     }
     if(last_err<0)
     {
        //lost_line=2;                                  //ʹcontrol.c���ö������
     }
   }
   if(line_con>30&&line_con3>30)
   {
      lost_line=0;                                      //ʹcontrol.c�ж����������
   }
   
   
//if(search_end_line>55&&Distance>1) Stop=1;
   
   if(Middle_Err!=0)
   {
      last_err=Middle_Err;                              //e.g.С��ƫ��ʱ���Ϊ��
   }

  Middle_Err= Middle_Err_Sum;                           //������� e.g.С��ƫ��ʱ���Ϊ��
  Push_And_Pull(Previous_Error,10,Middle_Err);          //Middle_Err���볤��Ϊ12������Previous_Error����Ԫ
  Delt_error=-10*Slope_Calculate(0,10,Previous_Error);  //�ñ�����������������ļ�ʹ��
  
//roadturncal();
}
//search()����

