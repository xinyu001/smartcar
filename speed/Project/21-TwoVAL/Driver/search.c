/*
最后一部分if(line_58<15&&line_59<15) stop=1;注释掉了，否则跑到没有蓝布的赛道会停下来
没有无连通域行或者无连通域行在25行以上时，Middle_Err_Sum取25、26、27行改为29、30、31行
mid_line和right_line由35和70改为30和60
*/



/*逻辑错误修改*/
/*
get_edge()中edgposition[i]=0;重复赋值
删除空函数void fix_break_line() //修复断开的线
删除变量Middle_Temp，赋值后从未使用
float Previous_Error[12]改成float Previous_Error[10];search.h中该变量声明也改了，因为只用了10个元素
static int last_err[2]改成了单个变量static int last_err，这两个元素始终相等
删除从未使用的float Slope_Calculate_Uint8(uint8 begin,uint8 end,uint8 *p)    //最小二乘法拟合斜率  本文件末尾调用founction.c中同样功能的函数
删除float character_distance;//赛道特征点的位置，赋值后从未使用
删除搜线for循环中的if(find==0)语句块中的i++语句
左右两边线补线for循环中的斜率计算放到for循环之前

*/
#include "include.h"
#define SlopeLeftDiff 0.4
#define SlopeRightDiff 0.3
extern uint8 flag2;
float SlopeRight[6]={0},SlopeLeft[6]={0};
float Distance_0,Distance_1,Distance2,Distance4,Distance5,Distance6,Distance7;
float Distance200;//s 8.1
float Distance3=1000,Distance8=1000;
float distance_test;
uint8  RoadType=0;//路况标志，默认0，1十字路口
float Previous_Error[10];
int vol0=0,Display1=0,Display2=0,Display3=0;
int Display4=0,Display5=0,Display6=0;

int vol1;////////////////////////////////////7月4日
int flag,flag_obstacle;
int flag_right,flag_left,flag_3,flag_4,flag_13,flag_14,flag_15,flag_7,flag_17;
int flag_16;
extern uint8 Style;       ////////////7月7日加
extern uint8 flag_jump;
extern int Stop_Brake;
uint16 edgposition[CAMERA_H];
uint8 mid_line=35,left_line=0,right_line=70;//mid_line越小，小车越靠右
uint8 lost_line;
uint16 cont;
//float Distance6;//s 疑似重复
int jishu1,jishu2,jishu,jishu3;
int count;


void Push_And_Pull(float *buff,int len,float newdata)
{//新的数据放入第一个元素，其他元素整体后移
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}
   
/*
void sendimg()    
{   
   //my_putchar(img_edg[i]);
  
}
*/
void get_edge()   //尽量减少乘法运算
{

  static int16 i=0,j=0,last=0,x=0,y=0,n=0;
  uint8 temp=0,find=0;
  cont=0;
  for(i=0;i<60;i++)
  {
    last=0; 
    x=i*10;                                     //行数
    find=0;                                     //每一行的遍历中只要发生一次跳变就把find改成1    
    edgposition[i]=0;                           //16位无符号整型数组，长度为图像的高
    for(j=0;j<10;j++)                           //字节列数; 该层循环相当于在一个字节一个字节地遍历
    {
      if(imgbuff_process[x+j]==0xff)            //8位无符号整型一维数组，长度为60行*（80/8）列
      {                                         //如果该字节全1（白）
        if(last==0)
        {                                       //如果这是该行第一个字节或者该行中上一个是0（黑）
          y=j<<3;                               //该字节列数*8=记录跳变后第一列像素列数
          if(find==0)
          {                                     //如果是该行第一次跳变
            edgposition[i]=cont;
          }
          img_edg[cont++]=y;                    //该行跳变的位置（跳变后第一列像素列数）记录在该数组
          find=1;
        }
        last=1;                                 //标记上一个（此处）是1
        continue;                               //遍历下一个字节
      }
      if(imgbuff_process[x+j]==0)
      {                                         //如果该字节全0（黑）
        if(last==1)
        {                                       //如果该行上一个是1（白）
          y=j<<3;
          if(find==0)
          {                                     //如果是该行第一次跳变
            edgposition[i]=cont;
          }
          img_edg[cont++]=y;                    //该行跳变的位置（跳变后第一列像素列数）记录在该数组  
          find=1;
        }
        last=0;                                 //标记上一个（此处）是0
        continue;                               //遍历下一个字节
      }      
      for(n=7;n>=0;n--)
      {                                         //如果该字节不是全0或全1的话，就从前往后一位一位遍历这个字节
        temp=(imgbuff_process[x+j]>>n)&1;       // 获取该点像素值 （0或1）     
        if(temp!=last)                          //与上一个值不相同 出现了跳变沿            
        {
           y=j<<3;  
           if(find==0)
           {                                    //如果是该行第一次跳变
            edgposition[i]=cont;
           }
           img_edg[cont++]=y+7-n;               //该行跳变的位置（跳变后第一列像素列数）记录在该数组
           find=1;
        }
        last=temp;                              //存储该点的值
      } 
    } 
    img_edg[cont++]=0xff;                       //跳变位置的列数不可能达到11111111 

  }
}
/*img_edg是一个一维数组 记录了摄像头每行的跳变沿的坐标值  每行跳变沿 由上升沿开始（由黑变白）然后接下降沿（由白变黑） 这两个坐标值（列数）保存在相邻两个元素
   0xff用于指示该行的跳变沿完了，开始记录下一行
   如果每行图像以白色部分开始，那么该行跳变沿起始位置为0
   如果该行全黑 那么该行记录为0xff
   如果该行为全白 记录为 0 0xff
 
   
  oxff代表本行坐标值结束，进入下一行
 //edgposition[i]代表第i行的 第一个跳变沿 在 img_edg 中坐标起点

 */
void judgeblack()
{
  static int16 n=0,m=0;
  int16 sum[5];  //15-19行
  int16 sum1[5];  //0-4行
  int16 sum2[5];  //55-59行
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
  if( (sum[0]<4) || (sum[1]<4) || (sum[2]<4) || (sum[3]<4) || (sum[4]<4) )  //5行全黑（每行最多4个白点）
  { RoadType=100;  //切换到电磁
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
    if( (sum1[0]<4) || (sum1[1]<4) || (sum1[2]<4) || (sum1[3]<4) || (sum1[4]<4) ||  (sum[0]<4) || (sum[1]<4) || (sum[2]<4) || (sum[3]<4) || (sum[4]<4))  //5行全黑（每行最多4个白点）
  { flag=1; //识别到障碍
 // uart_putchar(UART0,'D');
  }
  }*/
  
  
  if(RoadType==100)                             // 从电磁切换回普通赛道类型
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
  //从底部往上搜线
      
      //judgeblack();                           //s 切换电磁
      vol0=adc_once(ADC1_SE7a, ADC_16bit);
      vol1=adc_once(ADC1_SE13, ADC_16bit);              //7月4日

      Display1=adc_once(ADC1_SE8, ADC_16bit);
      Display2=adc_once(ADC1_SE13, ADC_16bit);
      Display3=adc_once(ADC1_SE10, ADC_16bit);
      

  
  float Middle_Err_Sum=0,slope;                         //Middle_Err_Sum是一行的误差
  static int i,j,find;                                  //find连通域标志，1表示遇到连通域
  uint8 left_cont=0,right_cont=0;
  
  uint8 Stop_Change=0,Stop_White=0,Stop_Black=0;        //起跑线相关
  static int k;
  
  int leftfind=0,rightfind=0;
  int AllWhileStartLine=0,AllWhileEndLine=0;            //分别是图像中从下往上第一个和最后一个全白行，变量拼写有误
  
  static int break_line_left=0,break_line_right=0,continue_line_left=0,continue_line_right=0;
              //左右边线断线前的最后一行            //左右边线续线行的第一行
  uint8 line_i,line_j,line_con,line_con3,line_58,line_59;
  static int last_err;
  
  int search_end_line=0;
  for(i=0;i<60;i++)                             //清空数组(该数组定义在camera.c 8位无符号整型)
  {  
        LMR[0][i]=0;                            //左边线数列
        LMR[1][i]=0;                            //中线数列
        LMR[2][i]=80;                           //右边线数列
  }
  leftfind=0;
  rightfind=0;
  break_line_left=0;
  break_line_right=0;
  continue_line_left=0;
  continue_line_right=0;
  
  for(i=59;i>0;i--)                             //从第59行开始搜线，目的是总结归纳图像
  {
 
   
    //起跑线
   if(i==30 &&  Distance>50)  //Distance>60即保证车是在第二次经过起跑线时才执行  原i==55
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
        
        if((Stop_Black+Stop_White)>6)                   //s 原8，因为摄像头高度从20cm调为10cm，看到的条数减少了
        {
          
          RoadType=205;                                 //s 8.1 看到起跑线，准备入库
          Distance200=Distance;                         //记录当前距离
          SetSpeed=0;
          Speed_H=0;
          Speed_M=0;
          Speed_L=0;
        }
        Stop_Black=0;
        Stop_White=0;

        
    }    
    

    if(edgposition[i]==0&&(i!=0))                       //表示从该行开始往上图像全黑 置为丢线  lost_line=0则舵机正常转向
    {
      if(i>24)
      {                                                 //如果从上往下第25行才开始跳变
        if(last_err>0)
        {
           lost_line=1;                                 //使control.c中让舵机往右
        }
        if(last_err<0)
        {
           lost_line=2;                                 //使control.c中让舵机往左
        }
      }
//      if(i<23)
//      {
//         lost_line=0;
//      }
      
      
      break;                                            //跳出搜线循环for,该行往上全黑，不需要搜了
    }
    j=edgposition[i];                                   //j为第i行第一次跳变沿在数组img_edg[]中的下标 j代表黑变白 即左边线 j+1代表变黑 即右边线 
    if(i==59)                                           //底部开始行
    {                                                   //这个while就是在挑选出最后一行中比较准确的跳变沿来赋值左右边线，有排除噪声的作用
        while(img_edg[j]!=255)                          //img_edg[j]代表第i行该跳变沿(黑变白)的坐标（列数），0~79列
       {  //255=0xff 代表本行坐标值结束，进入下一行
          if((img_edg[j]<55)&&(img_edg[j+1]>25))        //左边沿小于55 右边沿大于25(即不太歪的情况下或者该行全白）
          {
            if((img_edg[j+1]-img_edg[j])>25)            //右边沿-左边沿大于20
            {
              LMR[0][i]=img_edg[j];                     //赋值左边线最后一行为相应的列数
              if(img_edg[j+1]==255)
              {                                         //看不到该行下一次跳变（白变黑）
                 LMR[2][i]=80;                          //赋值右边线最后一行  
              }
              else
              {                                         //看到该行有下一次跳变（白变黑）
                 LMR[2][i]=img_edg[j+1];                //赋值右边线最后一行 
              }
              break;                                    //跳出while循环，也就结束了if（i==59)，进入下一行搜线for循环
            }
          } 
          if(img_edg[j+1]==255)
          {
            break;                                      //跳出while循环，也就跳出if（i==59)，进入下一行搜线for循环
          }
          j=j+2;                                        //更新j然后进入下一次while循环（继续分析底部行的情况），运行到这里说明有可能是噪声
       }
       
    //  if(LMR[0][i]==0)  break_line_left=59;
     // if(LMR[2][i]==80) break_line_right=59;
    }
    else                                                //不是底部开始行，其他的59行
    { 
        find=0;                                         //连通域标志先设为0 表示没有遇到连通域
        while(img_edg[j]!=255)                          //img_edg[j]代表第i行该跳变沿(黑变白)的坐标（列数），0~79列，循环结束的条件为到了该行的结束标志0xff。
       {
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j])>8)     
        {//如果该行左边沿小于等于上一行的右边线 &&该行右边沿大于等于上一行的左边线 && 是连通域；
          find=1;                                       //连通域标志，1表示遇到连通域
          if(LMR[0][i]==0&&(img_edg[j]!=0))
          {                                             //如果该行左边线还未赋值，但看到了该行跳变沿（黑变白)
            if(break_line_left!=0&&(continue_line_left==0))//表示可能循环到的是十字路全白区域前方的行，需要进一步判断
            {                                           //十字路口的情况下（左边线断线前的最后一行有记录），且左边线续线位置没有记录
              if(img_edg[j]>LMR[0][break_line_left]&&img_edg[j]<55)
              {                                         //判断该跳变沿（黑变白)有没有可能是左边线的续线（近大远小）
                LMR[0][i]=img_edg[j];                   //赋值该行左边线的列数
                continue_line_left=i;                   //保存左边线续线行（每一张图片至多执行一次）
                leftfind=1;
              }
            }
            else
            {//如果该行左边线还未赋值，但看到了该行第一个跳变沿（黑变白)，且（（左边线未断线即没有十字路）或（左边线的断线处和续线处都找到了））
                if(LMR[2][i]==80)
                {                                       //如果该行右边线还未赋值
                 LMR[0][i]=img_edg[j];                  //则赋值左边线为该行第一个跳变沿(黑变白)的坐标（列数）
                 leftfind=1;
                }
            }
          } 

          if(img_edg[j+1]!=255&&(LMR[2][i]==80))
          {                                             //如果该行右边线还未赋值且看得到该行第二个跳变沿（白变黑）
            if(break_line_right!=0&&(continue_line_right==0)&&(img_edg[j+1]>20))////表示可能循环到的是十字路全白区域前方的行，需要进一步判断
            {//十字路口的情况下（右边线断线前的最后一行有记录），且右边线续线位置没有记录
              if(img_edg[j+1]<LMR[2][break_line_right])
              {                                         //判断该跳变沿（黑变白)有没有可能是右边线的续线（近大远小）
                LMR[2][i]=img_edg[j+1];                 //赋值该行右边线的列数
                continue_line_right=i;                  //记录右边线续线行（每一张图片至多执行一次）
                rightfind=1;
              }
            }
            else
            {
                if((RoadType==1))                       //十字路口标志，初值为0表示未进入
                 {
                   if(img_edg[j+1]>30)
                   {
                    LMR[2][i]=img_edg[j+1];
                    rightfind=1;
                   }
                 }
                else
                {
                   LMR[2][i]=img_edg[j+1];              //赋值右边线为该行第二个跳变沿(白变黑)的坐标（列数）
                   rightfind=1;
                }
            }
          }
        }                                               //连通域判断结束
        if(img_edg[j+1]==255)                           //该行的跳变沿结束了
        {
          if(img_edg[j]==0)                             //说明该行为全白 
          { 
            if(AllWhileStartLine==0)
            {                                           //说明是第一次遇到全白行
              AllWhileStartLine=i;                      //记录全白行开始的行数（因为是倒序搜线所以实际上是最底部的全白行）
            }
            AllWhileEndLine=i;                          //暂时将该行记录为全白行结束的行数

            if(/*(rightfind&&leftfind&&(RoadType==0)&&i>20)||*/(AllWhileStartLine-AllWhileEndLine)>40)
              //且（（两边边线都找到了 （排除顶部20行的干扰））或（至少有了12行的全白行））  10
            {
                if(RoadType==0) RoadType=1;             //进入十字了
            }
          }
          break;                                        //跳出while，因为该行跳变沿结束了
        }
        
        
        j=j+2;                                          //每一行可能会有很多个跳变值
       }
                                                        //以上是搜索连通域的算法
       
       
       
       if(RoadType==1)                                  //（不是底部开始行的前提下）在十字内对搜线进行特殊处理（记录左右边线断线前的最后一行）
       { 
         if(left_cont>=4)
         {                                              //至少连续四次找到左边线
           if(((LMR[0][i]<(LMR[0][i+1]-1))||(LMR[0][i]==0))&&i>20)
           {                                            //左边线往外扩展（排除了循环到十字路口前方的行的情况）或者断线
                LMR[0][i]=0;
                if(break_line_left==0)break_line_left=i+1;         //记录左边线断线前的最后一行（前后那么多if主要是为了避免进入十字路口后该语句重复执行，仅当该行是断线后的第一行才执行到这里）
           }
         }
         if(LMR[0][i]!=0)                               //找到了左边边线就给这个变量加1
         {
           left_cont++;       
         }
         else
         {
           left_cont=0;               //遇到了十字路口，且循环到的行是十字路口内，才执行，为了避免进入上一个if
         }
         
         if(right_cont>=4)
         {
           if(((LMR[2][i]>(LMR[2][i+1]+1))||(LMR[2][i]==80))&&i>20)
           {                                            //右边线往外扩展或者断线
                LMR[2][i]=80;
                 if(break_line_right==0)break_line_right=i+1;       //记录右边线断线前的最后一行（前后那么多if主要是为了避免进入十字路口后该语句重复执行）
           }
         }
         if(LMR[2][i]!=80)                              //找到了右边线
         {
           right_cont++;
         }
         else
         {
           right_cont=0;
         }
       }
     
       
       if(find==0)                                      //（不是底部开始行的前提下）没有找到连通区域
       {
         search_end_line=i;                             //记录的是从倒数第二行开始往上遇到的第一个没有连通域的行数
         break;         //跳出搜线循环for，这种方式跳出循环的话，i为从下往上最后一个有连通域的行数或最后一行
       }
  }
  
 }                                                      //搜线for循环结束

 

//s 出库，开始正常行驶
if((RoadType==200)&&(Distance>1)){
        
         RoadType=0;

}
//s 8.1 入库结束，停车
if((RoadType==205)&&(Distance-Distance200>1)){
         Distance200=1000;
         RoadType=206;
//       Stop_Brake=1;          //刹车
//       SetSpeed=0;
//       Speed_H=0;
//       Speed_M=0;
//       Speed_L=0;
}



if(vol0>16000 && vol1>16000 && vol0<26000 && vol1<26000) //   17900
 {
   if(ABS(vol0-vol1)<800)                               //1500,1000
   { count++; }
   if(count>5)                                          //5
   {
   RoadType=50;
  // flag_obstacle=1;
   count=0;
   }
  // else if(ABS(vol0-vol1)<3000 && ABS(vol0-vol1)>1000)
  // RoadType=51;
  // if(abs(vol0-vol1)<6000)////////////7月4日，区分坡道和横断
   //   flag=1;  //////////7月4日，提前判断横断路障，减速
   //  if(vol0<18200 && vol1<14200)
   //   RoadType=51;
 //  else
 //    flag=0;
 //  if(flag==1)
   if(RoadType==50)
   {
      if(i<5)
      {
            if(edgposition[i]==0)               //表示从该行开始往上图像全黑 置为丢线  lost_line=0则舵机正常转向
        {
            jishu++;
            if(jishu>3)
            {
              uart_putchar(UART0,'V');
              flag=1;
              //  RoadType=51;                    //7月4日，进入转向程序
             // Style=0;
              
              jishu=0;
             
            }
        }
      }
   }
}

 if(RoadType==50)
 {

    lost_line=3;
 }

 
 /*else
 {
    Style=1;
 }*/

 /*  if((i>20) && (i<45)) // 10,23  30,45
  {
      if(img_edg[j]==255)
      {
          jishu1++;
          if(jishu1>15)
        { 
          RoadType=100;
          jishu1=0;
        }
      }
   }*/





/*if((i>5) && (i<20)) // 10,23  30,45
{
  if(edgposition[i]==0)
  {
  jishu1++;
  if(jishu1>10)
  { 
    RoadType=100;
    uart_putchar(UART0,'D');
    lost_line=3;
    Distance2=Distance;
    jishu1=0;
  }
  }
}*/
 



////////////6月28日/////////
/*
if(i<10)
  {
    if(edgposition[i]==0&&(i!=0))
      {
        jishu1++;
        if(jishu>8)
          {
             if(vol0>24886)
              {RoadType=50;}
             RoadType=100;
          }
      }
  }

*/
///////////6月28日///////////




/*if(RoadType==100)
{
   //uart_putchar(UART0,'D');
   //uart_putchar(UART0,'\n');
    //  SetSpeed=0.3;   //6月29日
 //  if(Distance-Distance2>3.5)
 //  {
       if(i<30)
        {
           if((img_edg[j]!=0) &&(img_edg[j+1]!=255))
         // if((LMR[0][i]<10) && (LMR[2][i])>60)
               {
                  jishu2++;
                 if(jishu2>20)
                 { 
                      RoadType=0;
     //                 SetSpeed=0.6;   //6月29日
     //                 jishu2=0;
                  }
               }
        }
  // }
}*/
 
////避免使用红外判断坡道，会与横断混淆
/*if(i<30)
{
  if((img_edg[j]!=0) &&(img_edg[j+1]!=255))
  {
      if(ABS(vol0-vol1)>3500 && ABS(vol0-vol1)<5000)
      {
        jishu3++;
        if(jishu3>10)
        {
          RoadType=200;
          Distance8=Distance;
          jishu3=0;
        }
      
      
      }
  
  }


}

if(Distance-Distance8>1 && RoadType==200)
{
  RoadType=0;
}*/

/*if(flag_jump==1)
{
  RoadType=0;
  flag_jump=0;
}*/
 
 //以下对十字路口进行补线并且判断不同的路段
 if(RoadType==1)                //进一步找准左右两边线的续线行的行数然后补线，为啥两边线判断方式略有不同？？？
 {//十字路口
   if(AllWhileEndLine==0||AllWhileEndLine<20)RoadType=0;//表示最后一个全白行还远
   if(continue_line_left!=0)
   {
       for(i=continue_line_left-1;(i>continue_line_left-10)&&(i>0);i--) //续线行可能有问题 检测一下 
      {
        if((LMR[0][i]!=0)&&(LMR[0][i-1]!=0))
        {
          if(ABS(LMR[0][i]-LMR[0][i-1])<2)
          {
              if(ABS(LMR[0][i-1]-LMR[0][i-2])<2)
            {//如果在左边线续线行上方有连续两次相邻行的左边线的之差小于2？？？？？？？？看不懂判断条件
              continue_line_left=i;//将左边线续线行改为更小的行数
              break;
            }
          }
        }
      }
      
      slope=(LMR[0][break_line_left]- LMR[0][continue_line_left])*1.0/(break_line_left-continue_line_left);//计算左边线斜率 应该是负的（x/y）(左边线断线行对应的列数-左边线续线行对应的列数）/（断线行-续线行）
      for(i=break_line_left;i>=continue_line_left;i--)                  //开始补线
      {//斜率计算可以优化
        LMR[0][i]= LMR[0][break_line_left]-(int)((break_line_left-i)*slope);//左边线断线行对应的列数加上一个正数
      }
   }
   
   if(break_line_right!=0&&continue_line_right!=0)
   {
       for(i=continue_line_right;(i>continue_line_right-10)&&(i>0);i--) //续线行可能有问题 检测一下 
      {
        if((LMR[2][i]!=0)&&(LMR[2][i-1]!=0))
        {
          if(ABS(LMR[2][i]-LMR[2][i-1])<2)
          {
            continue_line_right=i;//将右边线续线行改为更小的行数
            break;
          }
        }
      }
      
      slope=(LMR[2][break_line_right]- LMR[2][continue_line_right])*1.0/(break_line_right-continue_line_right);//计算右边线斜率 应该是正的（x/y）(右边线断线行对应的列数-右边线续线行对应的列数）/（断线行-续线行）
      for(i=break_line_right;i>=continue_line_right;i--)                //开始补线
      { 
         LMR[2][i]= LMR[2][break_line_right]-(int)((break_line_right-i)*slope);//右边线断线行对应的列数减去一个正数
      }
      
   }   
 }

 SlopeLeft[0]=(LMR[0][15]-LMR[0][18])/3.0;
 SlopeLeft[1]=(LMR[0][20]-LMR[0][23])/3.0;
 SlopeLeft[2]=(LMR[0][25]-LMR[0][28])/3.0;
 SlopeLeft[3]=(LMR[0][30]-LMR[0][33])/3.0;
 SlopeLeft[4]=(LMR[0][35]-LMR[0][38])/3.0;
 
 
 

 if(RoadType==0)                                        //判断左边线是否为直线
 {
   if(SlopeLeft[0]!=0 && SlopeLeft[1]!=0 && SlopeLeft[2]!=0 && SlopeLeft[3]!=0)
   {
     if(SlopeLeft[0]<1.2 && SlopeLeft[1]<1.2 && SlopeLeft[2]<1.2)
     {       
       if(ABS(SlopeLeft[0]-SlopeLeft[1])<SlopeLeftDiff && ABS(SlopeLeft[1]-SlopeLeft[2])<SlopeLeftDiff && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff )
       {
         RoadType=2;                                    //左边线为直线
        // uart_putchar(UART0,'L');
       }
     }
   }
 }
 if(RoadType==2 || RoadType==12 || RoadType==0)  
 {                                      
  // if(ABS(SlopeLeft[0]-SlopeLeft[2])>=SlopeLeftDiff || ABS(SlopeLeft[1]-SlopeLeft[3])>=SlopeLeftDiff || ABS(SlopeLeft[2]-SlopeLeft[4])>=SlopeLeftDiff)
  //    RoadType=0;  //此处判断存在疑问？？？？？？？？？？？？？？
   if(LMR[2][5]!=80 && LMR[2][7]!=80 && LMR[2][10]!=80 && LMR[2][15]==80 && LMR[2][20]==80 && LMR[2][5]<LMR[2][7] && LMR[2][7]<LMR[2][10] && ABS(LMR[0][15]-LMR[0][22])<5 /*&& ABS(SlopeLeft[0]-SlopeLeft[1])<=SlopeLeftDiff  && ABS(SlopeLeft[1]-SlopeLeft[2])<=SlopeLeftDiff*/)
   { RoadType=3;                                        //看到第一个朝右丁字路口  //32   34
      flag_3=1;                                         //右环第一个路口标志
  // uart_putchar(UART0,'F');
   }  
 }
 
 if(RoadType==3)   
 {
  // if(ABS(SlopeLeft[1]-SlopeLeft[2])>=2.5*SlopeLeftDiff || ABS(SlopeLeft[1]-SlopeLeft[3])>=2.5*SlopeLeftDiff /*|| ABS(SlopeLeft[2]-SlopeLeft[4])>=SlopeLeftDiff*/)
  // RoadType=0;
  // RoadType=4;   if 3 then 4 ,那么连3都不会出现
   if(LMR[2][20]==80 && LMR[2][25]==80 && LMR[2][30]==80  && LMR[2][35]==80 && LMR[0][20]==0  && LMR[0][25]==0  && LMR[0][30]==0 && LMR[0][35]==0)////////7月5日改
     RoadType=0;                                        //避免路过十字路口变成5，因为出环岛时也是4
   if(LMR[2][5]!=80 && LMR[2][7]!=80 && LMR[2][10]!=80 && LMR[2][15]!=80 && LMR[2][20]!=80 && LMR[2][5]>LMR[2][10] && LMR[2][20]>LMR[2][10] && LMR[0][6]!=0  && LMR[0][7]!=0  && LMR[0][8]!=0 && ABS(SlopeLeft[0]-SlopeLeft[1])<=SlopeLeftDiff  && ABS(SlopeLeft[1]-SlopeLeft[2])<=SlopeLeftDiff/*&& LMR[2][5]>LMR[2][6] && LMR[2][20]>LMR[2][10]*/)  // 34  36    
   {  RoadType=4;                                       //看到环岛侧边
      flag_4=1;                                         //右环侧边标志
  // uart_putchar(UART0,'H');
   }
   
 }
 if(RoadType==4)  
 {
  //  if(ABS(SlopeLeft[0]-SlopeLeft[2])>=2.5*SlopeLeftDiff || ABS(SlopeLeft[1]-SlopeLeft[3])>=2.5*SlopeLeftDiff || ABS(SlopeLeft[2]-SlopeLeft[4])>=2.5*SlopeLeftDiff)
  //  RoadType=0;

   if(LMR[2][5]!=80 && LMR[2][7]!=80 && LMR[2][20]==80 && LMR[2][25]==80 && LMR[2][15]==80 && LMR[2][5]<LMR[2][7] )
   { 
     RoadType=5;                                        //看到第2个朝右丁字路口
     flag_right=1;                                      //进入右环标记
     Distance3=Distance;                                //记下进入右环时的距离，判断RoadType6
     Distance7=Distance;                                //记下进入右环时的距离，判断RoadType0
    // uart_putchar(UART0,'S');
   }
   if(LMR[2][20]==80 && LMR[2][25]==80 && LMR[2][30]==80  && LMR[2][35]==80 && LMR[0][20]==0  && LMR[0][25]==0  && LMR[0][30]==0 && LMR[0][35]==0)////////7月5日改
     RoadType=0;                                        //避免路过十字路口变成5，因为出环岛时也是4  ，斟酌
 }
if(i<10)
{
  if(img_edg[j]==255)
  {
  jishu1++;
  if(jishu1>5)
  {
    if(LMR[2][20]==80 && LMR[2][25]==80 && LMR[2][30]==80  && LMR[2][35]==80 && LMR[0][20]==0  && LMR[0][25]==0  && LMR[0][30]==0 && LMR[0][35]==0)
    {
      if(Distance-Distance3>3)                                 //出环判断，必须在环内行驶一定的距离
      {RoadType=6;
      jishu1=0;
      }
    }
  }
  }
}
//进环行驶一段距离后变回普通RoadType
 if((Distance-Distance7>1)&& RoadType==5)
 {RoadType=0;
 flag_3=0;
 flag_4=0;
 }
 //出右环
if(RoadType==6 && flag_right==1)
{
  RoadType=7;
  flag_7=1;
  Distance4=Distance;
}
//防止误判
if(RoadType==6 && flag_right==0)
{
  RoadType=0;
}
//出右环一段距离后变回普通RoadType
if(flag_7==1)
{
  RoadType==7;
if(Distance-Distance4>0.55)
{
  RoadType=0;
  flag_right=0;
}
}
 /*if(RoadType==5 )
 {

     if(SlopeLeft[3]>1.3 && SlopeLeft[2]>1.3 && LMR[0][31]!=1 && LMR[0][27]!=1)/* Distance-Distance_0>1.4
     { 
       RoadType=6;//环岛内
     }
      
 } 
 if(RoadType==6 && LMR[0][23]<LMR[0][27] && LMR[0][24]<LMR[0][27])
 {
   RoadType=7;  //出环岛
 } 
 if(RoadType==7 )
 {
   if(SlopeLeft[0]!=0 && SlopeLeft[1]!=0 && SlopeLeft[2]!=0 && SlopeLeft[3]!=0)
   {
     if(SlopeLeft[0]<1.2 && SlopeLeft[1]<1.2 && SlopeLeft[2]<1.2)
     {       
       if(ABS(SlopeLeft[0]-SlopeLeft[1])<SlopeLeftDiff && ABS(SlopeLeft[1]-SlopeLeft[2])<SlopeLeftDiff && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff )
       {
          RoadType=0;
          flag_right=0;
       }
     }
   }   
 } */

 
 SlopeRight[0]=-(LMR[2][15]-LMR[2][18])/3.0;
 SlopeRight[1]=-(LMR[2][20]-LMR[2][23])/3.0;
 SlopeRight[2]=-(LMR[2][25]-LMR[2][28])/3.0;
 SlopeRight[3]=-(LMR[2][30]-LMR[2][33])/3.0;
 SlopeRight[4]=-(LMR[2][35]-LMR[2][38])/3.0;  
 

 
 if(RoadType==0)
 {
   if(SlopeRight[0]!=0 && SlopeRight[1]!=0 && SlopeRight[2]!=0 && SlopeRight[3]!=0)
   {
     if(SlopeRight[0]<4 && SlopeRight[1]<4 && SlopeRight[2]<4 )
     {
       if(ABS(SlopeRight[0]-SlopeRight[1])<SlopeRightDiff && ABS(SlopeRight[1]-SlopeRight[2])<SlopeLeftDiff && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff )
       {
         RoadType=12;                                   //右边线为直线
       }
     }
   }
 }
 
 if(RoadType==12 || RoadType==0 || RoadType==2)   
 {  
   if(ABS(SlopeRight[0]-SlopeRight[1])>=1.4*SlopeRightDiff || ABS(SlopeRight[1]-SlopeRight[2])>=1.4*SlopeRightDiff )
      RoadType=0;
   
   else if(LMR[0][5]!=0 && LMR[0][7]!=0 && LMR[0][9]!=0 && 
           LMR[0][11]==0 && LMR[0][12]==0 && 
           LMR[0][18]!=0 && LMR[0][20]!=0 && LMR[0][22]!=0 && 
           LMR[0][5]>LMR[0][7] && LMR[0][7]>LMR[0][9]&& 
           LMR[2][4]<LMR[2][8] && LMR[2][6]<LMR[2][12] && LMR[2][10]<LMR[2][20] )
   
   {
      RoadType=13;                                        //看到第一个朝左丁字路口
      flag_13=1;                                           //左环第一个路口标记
      Distance3=Distance;                                //s 记下第一次看到左环时的距离，判断RoadType15
      
   }
 }
 
if(RoadType==13)      
{
   
   //SetSpeed=0.5;
   if(ABS(SlopeRight[0]-SlopeRight[1])>=4*SlopeRightDiff || ABS(SlopeRight[1]-SlopeRight[2])>=4*SlopeRightDiff) //
      RoadType=0;


   
   else if( LMR[0][2]!=0 && LMR[0][4]!=0 && LMR[0][6]!=0 && LMR[0][18]!=0 && LMR[0][20]!=0 &&
           LMR[0][12]<LMR[0][6] && LMR[0][12]<LMR[0][18] &&
           LMR[2][4]<LMR[2][8] &&LMR[2][6]<LMR[2][12] && LMR[2][10]<LMR[2][20] )
   {
     RoadType=14;                                         //看到环岛侧边
     flag_14=1;                                           //左环侧边标记
   }
}
 
 if(RoadType==14)   //s 离第一次看到左环通过了一段距离
 {
   if(LMR[0][10] == 0 && LMR[0][15] == 0 && LMR[0][20] == 0 && LMR[2][20] == 80 && LMR[2][15] == 80 && LMR[2][10] == 80)
   {
     RoadType=0;
   }
   
   if(LMR[0][4]!=0 && LMR[0][6]!=0 && LMR[0][15]==0 && LMR[0][20]==0 &&
      LMR[0][25]==0 && LMR[0][4]>LMR[0][6])     
   { 
     //RoadType=15;                                       //看到第2个朝左丁字路口
     //flag_left=1;                                       //进入左环标记
     flag_15=1;
    // Distance4=Distance;                                //记下进入左环时的距离，判断RoadType6     
   }             
 }
 if(flag_15=1&&(Distance-Distance3>3.2)){
   
    RoadType=15; 
    flag_left=1;
    Distance3=1000;
    Distance4=Distance;
 }
 
 //进环后变RoadType16
if(RoadType==15)//;flag_15==1)
{
  //RoadType==15;
  if(Distance-Distance4>1)
  {
    RoadType=16;
    flag_16=1;
    flag_13=0;
    flag_14=0;
    flag_15=0;
    Distance5=Distance;//记下环内行驶的距离，判断RoadType17
  }
}
 
 //出环岛
if(flag_16==1)
{
  RoadType==16;
  if(flag_left==1 && Distance-Distance5>4.5 )
  { 
    flag_16=0;  
    RoadType=17;
    flag_17=1;
    Distance6=Distance;     //记录出环时距离
   }                      
}

//出左环拐弯
if(flag_17==1)
{
  RoadType=17;
  if(Distance-Distance6>2)
  {
    RoadType=0;
    flag_left=0;
    flag_17=0;
  }
}
 /*if(RoadType==15 && SlopeRight[2]>1.3 && SlopeRight[3]>1.3)
 {
   RoadType=16;  //环岛内
 } 
 if(RoadType==16 && LMR[2][23]>LMR[2][27] && LMR[2][24]>LMR[2][27])
 {
   RoadType=17;  //出环岛
 } 
 
 if(RoadType==17 )
 {
   if(SlopeRight[0]!=0 && SlopeRight[1]!=0 && SlopeRight[2]!=0 && SlopeRight[3]!=0)
   {
     if(SlopeRight[0]<1.2 && SlopeRight[1]<1.2 && SlopeRight[2]<1.2 )
     {
       if(ABS(SlopeRight[0]-SlopeRight[1])<SlopeRightDiff && ABS(SlopeRight[1]-SlopeRight[2])<SlopeLeftDiff && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff )
       {
         RoadType=18;
         Distance_0=Distance;
       }
     }
   }
 }  
 if(RoadType==18 && Distance-Distance_0>1)
 {
   RoadType=0;    //为什么此处判断正常赛道的方法和左侧判断时不一样？？？？？？？？？？？
 }  */

 
 
 
    //躲避左侧障碍物
 /*   if(LMR[0][20]-LMR[0][24]>5 && LMR[0][20]-LMR[0][6]>3 && LMR[0][5]!=1 && LMR[0][7]!=1 && LMR[0][9]!=1 && LMR[0][25]!=1 && LMR[0][27]!=1 && LMR[0][29]!=1 && RoadType!=7 && SlopeLeft[2]<1.2 && SlopeLeft[3]<1.2 && LMR[2][25]!=80 && LMR[2][27]!=80 && LMR[2][25]!=80 && ABS(SlopeRight[0]-SlopeRight[1])<SlopeRightDiff && ABS(SlopeRight[1]-SlopeRight[2])<SlopeLeftDiff  )
    {
      
      Distance_1=Distance;
      RoadType=40;//左侧障碍物
    }
    if(RoadType==40 && Distance-Distance_1>2) RoadType=0;  //通过距离来实现对障碍的躲避

    //躲避右侧障碍物
    if(LMR[2][20]-LMR[2][24]<-5 && LMR[2][20]-LMR[2][6]<-3 && LMR[2][6]>LMR[2][4] && LMR[0][18]!=1 && LMR[0][22]!=1 && LMR[0][25]!=1 && LMR[0][27]!=1 && LMR[0][29]!=1 && LMR[0][32]!=1 && LMR[2][25]!=80 && LMR[2][27]!=80 && LMR[2][29]!=80 && RoadType!=16 && RoadType!=1 && SlopeRight[2]<1.2 && ABS(SlopeLeft[0]-SlopeLeft[1])<SlopeLeftDiff && ABS(SlopeLeft[1]-SlopeLeft[2])<SlopeLeftDiff && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff )
    {
      
      Distance_1=Distance;
      RoadType=41;//右侧障碍物
    }
    if(RoadType==41 && Distance-Distance_1>2) RoadType=0;*/


   
    //右急弯
    if(SlopeLeft[0]>1.3 && RoadType==0){
      if(LMR[0][12]>35 && LMR[0][12]>LMR[0][15]){
        if(LMR[0][15]>25 && LMR[0][15]>LMR[0][20]){
          if(LMR[0][20]>19 && LMR[0][20]>LMR[0][25]){
            if(LMR[0][25]>15 && LMR[0][25]>LMR[0][28]){
              if(LMR[2][5]==80 && LMR[2][7]==80 && LMR[2][10]==80){//非S弯
                RoadType=32;//将要进入右急弯
                Distance_0=Distance;
                
              }
            }
          }
        }
      }
    }
    if(RoadType==32 && Distance-Distance_0>1.5 ){
      RoadType=33;//右急弯内
    }
    if(RoadType==33 && ABS(SlopeLeft[2]-SlopeLeft[3])<SlopeLeftDiff && SlopeLeft[2]<1.2 && SlopeLeft[3]<1.2){
      RoadType=0;
    }
    

    //左急弯
    if(SlopeRight[0]>1.3 && RoadType==0){
      if(LMR[2][12]<45 && LMR[2][12]<LMR[2][15]){
        if(LMR[2][15]<55 && LMR[2][15]<LMR[2][20]){
          if(LMR[2][20]<61 && LMR[2][20]<LMR[2][25]){
            if(LMR[2][25]<65 && LMR[2][25]<LMR[2][28]){
              if(LMR[0][5]==0 && LMR[0][7]==0 /*&& LMR[0][10]==0*/){//非S弯
                RoadType=36;//将要进入左急弯
                Distance_0=Distance;
                left_line=5;//避免左边压线
              }
            }
          }
        }
      }
    }
    if(RoadType==36 && Distance-Distance_0>1.5 ){
      RoadType=37;//左急弯内
      left_line=0;
    }
    if(RoadType==37 && ABS(SlopeRight[2]-SlopeRight[3])<SlopeRightDiff && SlopeRight[2]<1.2 && SlopeRight[3]<1.2){
      RoadType=0;
    }
   
     
 
 Middle_Err_Sum=0;
  for(i=0;i<59;i++)
   {//框定了左右边线范围，限定右边线为70
       if(LMR[2][i]>(right_line))
          LMR[2][i]=(right_line);
       if(LMR[0][i]<(left_line))
          LMR[0][i]=(left_line);

     if(RoadType==5)                                    //朝右的第二个丁字口
     {
       
          LMR[1][i]=40;
     }
     else if(RoadType==7)                               //朝右出环岛
     {
       
          LMR[1][i]=50;                                 //LMR[2][i]-11;
     }
     else if(RoadType==15)                              //朝左驶入环岛
     {
       
          LMR[1][i]=20;
     }
     else if(RoadType==16)                              //环岛内行驶
     {

       LMR[1][i]=(LMR[0][i]+LMR[2][i])/2;             //中线  
          
     }
     else if(RoadType==17)                              //出左环岛
     { 
       
          LMR[1][i]=24;                                 
     }

     else if(RoadType==18 )
     {
       
          LMR[1][i]=LMR[2][i]-26;
     }
     else if(RoadType==40 )
     {
       
          LMR[1][i]=LMR[2][i]-17;
     }
     else if(RoadType==41 )
     {
       
          LMR[1][i]=LMR[0][i]+12;
     }
     else if(RoadType==200)                     //s 8.1出库左拐
     {
          LMR[1][i]=15;
     }
     else if(RoadType==205)                     //s 8.1入库左拐
     {
          LMR[1][i]=10;
     }
  /*   else if(RoadType==50 && flag==1)
     {
       lost_line=3;
       if(vol0>24886)
       LMR[1][i]=25;
       if(vol0>24886 && vol0<25000)
       {Distance6=Distance;
       }
       if((Distance-Distance6)<=0.8)
       {LMR[1][i]=25;}
       else if((Distance-Distance6)>0.8 && (Distance-Distance6)<=1.6)
       {LMR[1][i]=70;
       }
       else if((Distance-Distance6)>1.6 && (Distance-Distance6)<=2.4)
       {
       LMR[1][i]=35;
       }
       else if((Distance-Distance6)>2.4 && (Distance-Distance6)<=3.2)
       {
       LMR[1][i]=70;
       }
       else if((Distance-Distance6)>3.2 && (Distance-Distance6)<=3.8)
       {
       LMR[1][i]=25;
         if((Distance-Distance6)>3.7)
        RoadType=0;
      // LMR[1][i]=(LMR[0][i]+LMR[2][i]);  //////应该可以不要
       }
       
     
     
     }*/

     else                                               //RoadType=0,6，16...
     {
       LMR[1][i]=(LMR[0][i]+LMR[2][i])/2-3;             //中线  -3
     }
   }
  
 
  
  
  for(i=0;i<59;i++)
   {

    if(search_end_line<25)                      //search_end_line 因找不到连通域跳出搜线循环时的行数
    {                                           //没有无连通域行或者无连通域行在25行以上
      if(Style==1)
      {                                         //不加速
        if(i>=21&i<=23)            //30  32摄像头低了，所以计算误差要提前，前瞻  '25,27'慢其实挺好  '21,23'快 
        {//my_putchar('_');
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
        }//20、21、22这三行的两中间线之差累加（判断出的道路中间线-摄像头对应的中间线）e.g.小车偏左时误差为正
      }
      else{//加速
/*
          if(i>=28&&i<=30)
          {//my_putchar('_');
            Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
          }
*/
          
        
        //if( (RoadType==32 || RoadType==33 || RoadType==36 || RoadType==37) && CarSpeed<0.7){//急弯
        if (RoadType==0)
        {//避免S弯左侧压线

          if(i>=29&&i<=31)
          {//my_putchar('_');
            Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
          }
        }
        else {
          if(i>=28&&i<=30)
          {//my_putchar('_');
            Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;
          }
        }
        

      }
    }
    else
    {//最下面的无连通域行在25行及以下时
      if((i>=27)&&(i<=29))
      {//my_putchar('|');
          Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-mid_line;            //取了30、31、32行的误差
      }
    }
   }
   Middle_Err_Sum=Middle_Err_Sum/3;                                     //取平均
  
   if(RoadType==1)
   {                    //遇到十字路口的情况下，Middle_Err_Sum取全白行前方的且位置在第10行以下的某一行的误差
      for(i=AllWhileEndLine;i>10;i--)
     {//i从最后一个全白行（顶部）循环到正数第11行
       if(LMR[0][i]!=0&&LMR[2][i]!=(mid_line*2))//经过前面的框定，此时左右边线在0~70范围，对于全白行两个条件正好都不符合，直接进入下一次循环
       {
           if(LMR[0][i-2]!=0&&LMR[2][i-2]!=(mid_line*2))
         {
          Middle_Err_Sum=(LMR[0][i-2]+LMR[2][i-2])/2-mid_line;//可能取的是最后一个全白行的上方第3行的误差
          break;
         }
       }
     }
   }
   
   for(line_i=0;line_i<10;line_i++)
   {
     for(line_j=0;line_j<8;line_j++)//对原始图像25、27、58、59行的每个字节的每一位遍历
     {
        if(((imgbuff_process[25*10+line_i]>>line_j)&1)==1)line_con++;
        if(((imgbuff_process[27*10+line_i]>>line_j)&1)==1)line_con3++;
        if(((imgbuff_process[58*10+line_i]>>line_j)&1)==1)line_58++;
        if(((imgbuff_process[59*10+line_i]>>line_j)&1)==1)line_59++;    
     }
   }//这四个变量分别是对应的4行中1的个数
   
   if(line_con==0&&line_con3==0)
   {                                                    //如果中间位置的这两行全黑
     if(last_err>0)                                     //e.g.小车偏左时误差为正
     {
        //lost_line=1;                                  //使control.c中让舵机往右，
     }
     if(last_err<0)
     {
        //lost_line=2;                                  //使control.c中让舵机往左
     }
   }
   if(line_con>30&&line_con3>30)
   {
      lost_line=0;                                      //使control.c中舵机正常工作
   }
   
   /*
   if(Distance>7)
 {
     
      SetSpeed=0;
      Speed_H=0;
      Speed_M=0;
      Speed_L=0;
      
   } 
   */
 /*
   if(line_58<35 && line_59<35 && Distance>8)//识别起跑线
   {
      //Stop=1;//使control.c中电机不输出
      
      RoadType+=100;
      
      SetSpeed=0;
      Speed_H=0;
      Speed_M=0;
      Speed_L=0;
      
   }
 */  
 
//if(search_end_line>55&&Distance>1) Stop=1;
   if(Middle_Err!=0)//
   {
      last_err=Middle_Err;                              //e.g.小车偏左时误差为正
   }

  Middle_Err= Middle_Err_Sum;                           //更新误差 e.g.小车偏左时误差为正
  Push_And_Pull(Previous_Error,10,Middle_Err);          //Middle_Err放入长度为12的数组Previous_Error的首元
  Delt_error=-10*Slope_Calculate(0,10,Previous_Error);  //该变量算出来供在其他文件使用
  
 // if(flag2 == 1) RoadType = 50;
}//search()结束

