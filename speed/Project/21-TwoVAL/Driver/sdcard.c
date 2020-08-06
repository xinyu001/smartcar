#include "include.h"

#define Variable_ADDR   2
#define Para_ADDR   102
#define CCD_ADDR   212
uint8   SD_Buff[1024]; 
uint8   SD_OK=0;
uint8   SaveData=0;
uint16  Block_Index=0;
uint8   Zone_Index=0;
uint8   Zone_Read_Index=0;
uint8   Block_Index_Receiving;
uint8   SD_Type = 0;
void   SD_DisSelect();
uint8   SD_GetResponse(uint8 Response);
uint8   SD_RecvData(uint8*buf,uint16 len);
uint8   SD_Select();
void   SD_SPI_HighSpeed();
void   SD_SPI_LowSpeed();
uint8   SD_WaitReady();
uint8  SD_WriteDisk(uint8*buf,uint32 sector);
uint8  SPI_SendReceiveData(uint8 TxData);

uint8 Spi_Delay=3;

//�����ڵ�һ��ҳ��洢һ���ж���ҳ��������
void  Write_Information()
{
 uint8 i=0;
  uint16 j=0;
  float temp=0;
  uint8 Parameter_num=14;  //14���ɸĲ���
  
  
  
  SD_Buff[j++]=0x55;              
  SD_Buff[j++]=0xaa;
  SD_Buff[j++]=0xff;                 //����������ͷ
  SD_Buff[j++]=BYTE0(Block_Index);   //��һ��ʼ֡������ʶһ���洢�˶���֡����
  SD_Buff[j++]=BYTE1(Block_Index);
  
  SD_Buff[j++]=0x55;                 //���񴮿ڷ�������һ������Щ���ݰ���Э��ķ�ʽ����SD���ϣ���ʱ��ֱ�ӷ��ͼ��ɣ���λ�����ʶ��
  SD_Buff[j++]=0xaa;
  SD_Buff[j++]=0xab;                  ///��һ��ʼ֡���Դ�Ų����Ȳ��ᾭ���ı������
  SD_Buff[j++]=Parameter_num;
  

  
  for(i=0;i<Parameter_num;i++)
  { 
     temp=Control_Para[i];
    SD_Buff[j++]=BYTE0(temp);
    SD_Buff[j++]=BYTE1(temp);
    SD_Buff[j++]=BYTE2(temp);
    SD_Buff[j++]=BYTE3(temp);
  }
    SD_Buff[j++]=0X0b;//֡β
  
  SD_Buff[j++]=0xaa;                 //�������������ָʾ����������Ч���ݽ���
  SD_Buff[j++]=0xbb;
  SD_Buff[j++]=0xcc;
  
  SD_WriteDisk(SD_Buff,BLOCK_BEGIN);  //////��ָ������ʼ����д��Ϣ
  Block_Index=1;                       //��������ֻ�ܴӵڶ�������д
}
//�����洢��������
void Record()
{  
 //�������»��棬ÿ10msִ��һ��
  float temp;
  int i;
  uint16 j=0;
  uint8 Variable_num=16;
   
  int block_temp;
  
 
  SD_Buff[j++]=0x55;       //���񴮿ڷ�������һ������Щ���ݰ���Э��ķ�ʽ����SD���ϣ���ʱ��ֱ�ӷ��ͼ���
  SD_Buff[j++]=0xaa;
  SD_Buff[j++]=0xad;
  SD_Buff[j++]=Variable_num;
  for(i=0;i<Variable_num;i++)    //64������
  {
    temp=Variable[i];
    SD_Buff[j++]=BYTE0(temp);
    SD_Buff[j++]=BYTE1(temp);
    SD_Buff[j++]=BYTE2(temp);
    SD_Buff[j++]=BYTE3(temp);
  }
    SD_Buff[j++]=0x0d;
  
  SD_Buff[j++]=0x55;       //���񴮿ڷ�������һ������Щ���ݰ���Э��ķ�ʽ����SD���ϣ���ʱ��ֱ�ӷ��ͼ���
  SD_Buff[j++]=0xaa;
  SD_Buff[j++]=0xac;
  SD_Buff[j++]=0x8;
  
  
  
  uart_putbuff(UART_PORT, (uint8*)(&LMR[0][0]),     180); //���ͱ��߼�����
  uart_putbuff(UART_PORT, (uint8*)(&LMR[0][0]),     20); //Ԥ��20������λ
  my_putchar(0x0c);
  
   for(i=0;i<CAMERA_SIZE;i++)  //600
  {
    SD_Buff[j++]=*(imgbuff+i);
  }
  for(i=0;i<180;i++) //������� //180
  {
    SD_Buff[j++]=*(&LMR[0][0]+i);
  }
   for(i=0;i<20;i++) //������� //20
  {
     SD_Buff[j++]=0;
  }
  SD_Buff[j++]=0x0C;
  
  SD_Buff[j++]=0xaa;                 //�������������ָʾ����������Ч���ݽ���
  SD_Buff[j++]=0xbb;
  SD_Buff[j++]=0xcc;
  LED_BLUE_OFF;
    
  block_temp=Block_Index*2;
    
  SD_WriteDisk(SD_Buff,BLOCK_BEGIN+block_temp);
  SD_WriteDisk(&(SD_Buff[512]),BLOCK_BEGIN+block_temp+1);
  LED_BLUE_ON;  
                        //LED��ָʾ
  Block_Index=Block_Index++;
  
  
}

void SD_SPI_LowSpeed()
{
  Spi_Delay=10;
}

void SD_SPI_HighSpeed()
{
  Spi_Delay=1;
}

void SD_DisSelect()
{
  SD_CS_HIGH;
  SPI_SendReceiveData(0xff);   //�ṩ�����8��ʱ��
}
uint8 SD_WaitReady()
{
  uint32 t = 0;
  do
  {
    if(SPI_SendReceiveData(0xff) == 0xff) return 0;
    t++;
  }while(t < 0xffffff);
  return 1;
}

uint8 SD_GetResponse(uint8 Response)
{
  uint16 Count=0xFFFF;//�ȴ�����	   						  
  while((SPI_SendReceiveData(0XFF)!=Response)&&Count) Count--;//�ȴ��õ�׼ȷ�Ļ�Ӧ  	  
  if(Count==0)return MSD_RESPONSE_FAILURE;//�õ���Ӧʧ��   
  else return MSD_RESPONSE_NO_ERROR;//��ȷ��Ӧ	
}
//��sd����ȡһ�����ݰ�������
//buf:���ݻ�����
//len:Ҫ��ȡ�����ݳ���.
//����ֵ:0,�ɹ�;����,ʧ��;	
uint8 SD_RecvData(uint8*buf,uint16 len)
{			  	  
   if(SD_GetResponse(0xFE)) return 1;//�ȴ�SD������������ʼ����0xFE
    while(len--)//��ʼ��������
    {
      *buf=SPI_SendReceiveData(0xFF);
       buf++;
    }
    //������2��αCRC��dummy CRC��
    SPI_SendReceiveData(0xFF);
    SPI_SendReceiveData(0xFF);									  					    
    return 0;//��ȡ�ɹ�
}
//��SD������һ������
//����: uint8 cmd   ���� 
//      u32 arg  �������
//      uint8 crc   crcУ��ֵ	   
//����ֵ:SD�����ص���Ӧ															  
uint8 SD_SendCmd(uint8 cmd, uint32 arg, uint8 crc) 
{
    uint8 r1;	
    uint8 Retry=0;
    SD_DisSelect();//ȡ��Ƭѡ
    SD_CS_LOW; //Ƭѡ
    SPI_SendReceiveData(cmd | 0x40);//�ֱ�д������
    SPI_SendReceiveData(arg >> 24);
    SPI_SendReceiveData(arg >> 16);
    SPI_SendReceiveData(arg >> 8);
    SPI_SendReceiveData(arg);	  
    SPI_SendReceiveData(crc); 
    Retry=0X1F;
    do
    {
     r1=SPI_SendReceiveData(0xFF);
    }
    while((r1&0X80)&&Retry--);
    //����״ֵ̬;
    return r1;
}
uint8 SD_Initialize(void)
{
    uint8 r1;           //���SD���ķ���ֵ
    uint16 retry;       //�������г�ʱ����
    uint16 i;
    uint8 success=0;
      uint8 buff[4];
     gpio_init (SD_CS, GPO,0);
     gpio_init (SD_SCK, GPO,0);
     gpio_init (SD_SOUT, GPO,0);
     gpio_init (SD_SIN, GPI,0);
     port_init_NoALT (SD_SIN, PULLUP ); 
    
    SD_SPI_LowSpeed();	//���õ�����ģʽ 
    SD_DisSelect();
    for(i=0;i<10;i++)SPI_SendReceiveData(0XFF);//��������74������
    retry=20;
    do
    {
       r1=SD_SendCmd(CMD0,0,0x95);//����IDLE״̬
       SD_DisSelect();            //ȡ��Ƭѡ
    }
    while((r1!=0X01)&&retry--);
    SD_Type=0;//Ĭ���޿�
    if(r1==0X01)
    {
      if(SD_SendCmd(CMD8,0x1AA,0x87)==1)//SD V2.0
      {
        retry=0XFFFE;
        do
        {
            SD_SendCmd(CMD55,0,1);	       //����CMD55
            r1=SD_SendCmd(CMD41,0x40000000,1);//����CMD41
        }while(r1&&retry--);
        if(r1==0)success=1;  //��ʼ���ɹ���
       
        
         r1 = SD_SendCmd(CMD58, 0, 1);
        if(r1==0)
        {
          buff[0] =SPI_SendReceiveData(0xFF);
          buff[1] =SPI_SendReceiveData(0xFF);
          buff[2] =SPI_SendReceiveData(0xFF);
          buff[3] =SPI_SendReceiveData(0xFF); 
          SD_DisSelect();//ȡ��Ƭѡ
          if(buff[0]&0x40)SD_Type = SD_TYPE_V2HC; //���CCS
          else SD_Type = SD_TYPE_V2;
        } 
        
        
      }
    }
    SD_DisSelect();
    SD_SPI_HighSpeed();//����	
    return success;//��������
}
//��SD��
//buf:���ݻ�����
//sector:����
//cnt:������
//����ֵ:0,ok;����,ʧ��.
uint8 SD_ReadDisk(uint8*buf,uint32 sector)
{
  uint8 r1;
sector<<= 9;//ת��Ϊ�ֽڵ�ַ
  r1=SD_SendCmd(CMD17,sector,0X01);//������
  if(r1==0)//ָ��ͳɹ�
  {
    r1=SD_RecvData(buf,512);//����512���ֽ�	   
  }
  SD_DisSelect();//ȡ��Ƭѡ
  return r1;
}



//дSD��
//buf:���ݻ�����
//sector:��ʼ����
//cnt:������
//����ֵ:0,ok;����,ʧ��.
uint8 SD_WriteDisk(uint8*buf,uint32 sector)
{
  uint8 r1;
  uint16 t;
  uint16 retry;
  if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//ת��Ϊ�ֽڵ�ַ
  if(SD_Type!=SD_TYPE_MMC)
  {
    SD_SendCmd(CMD55,0,0X01);	
    SD_SendCmd(CMD23,1,0X01);//����ָ��	
  }
  r1=SD_SendCmd(CMD24,sector,0X01);//���������
  if(r1==0)
  {
    
    SD_CS_LOW;
    
    SPI_SendReceiveData(0XFF);
    SPI_SendReceiveData(0XFF);
    SPI_SendReceiveData(0XFF);
    
    SPI_SendReceiveData(0XFE);//��ʼд

    for(t=0;t<512;t++)SPI_SendReceiveData(buf[t]);//����ٶ�,���ٺ�������ʱ��
    
    SPI_SendReceiveData(0xFF);//����crc
    SPI_SendReceiveData(0xFF);
    t=SPI_SendReceiveData(0xFF);//������Ӧ
    if((t&0x1F)!=0x05)return 2;//��Ӧ����	
    
   }						 	 
    retry = 0;
    while(!SPI_SendReceiveData(0xff))
    {
      retry++;
      if(retry>0xffe) //�����ʱ��д��û����ɣ������˳�
      {
          SD_CS_HIGH;
          return 1; //д�볬ʱ����1
      }
    }
  SD_DisSelect();//ȡ��Ƭѡ
  return 1;//
}


void SPI_Delay(uint16 i)
{	
  while(i) 
    i--; 
}

//  spi_mosi(SPI0,SPIn_PCS0,cmd,NULL,buff,buff,1,2)
uint8 SPI_SendReceiveData(uint8 TxData)
{

      char i,temp; 

      temp=0; 

      SD_SCK_DAT=0; 

     // SPI_Delay(Spi_Delay);

      for(i=0;i<8;i++) 

      {  

        if(TxData & 0x80) 

        {  
          SD_SOUT_DAT=1; 

        }  

      else SD_SOUT_DAT=0; 

      TxData<<=1; 

      SD_SCK_DAT=1;
    //  SPI_Delay(Spi_Delay);
        
      temp<<=1;  

      if(SD_SIN_IN)temp++; 

      SD_SCK_DAT=0; 

   //   SPI_Delay(Spi_Delay);
      
      }
   return temp; 
}  
      