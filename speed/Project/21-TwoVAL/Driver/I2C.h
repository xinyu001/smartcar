#ifndef _I2C_H_
#define _I2C_H_

#include "common.h"
   

//����I2C�豸��ַ
#define I2C_MMA8451_ADR  0X1C
#define I2C_MPU3050_ADR  0x68

#define I2C_SCL       PTB2_OUT // PTC10_OUT   
#define I2C_SDA_I     PTB3_IN  // PTC11_IN
#define I2C_SDA_O     PTB3_OUT // PTC11_OUT 
//����SDA�������
#define I2C_SDA_OUT()  PTB3_DDR=1  //PTC11_DDR=1
#define I2C_SDA_IN()   PTB3_DDR=0  //PTC11_DDR=0  

#define I2C_DELAY()	I2C_Delay(15)	


extern int read_buff[6];   
   
void I2C_Init(void);

void I2C_WriteReg(uint8 dev_addr,uint8 reg_addr , uint8 data);

uint8 I2C_ReadByte(uint8 dev_addr,uint8 reg_addr);//��һ���ֽڵ�����

int16 I2C_ReadWord(uint8 dev_addr,uint8 reg_addr);//�������ֽڵ�����

void I2C_ReadGryo(uint8 dev_addr,uint8 reg_addr,int16 *x,int16 *y);

int16 Get_Z_Acc();
int16 Get_Y_Gyro();
int16 Get_Z_Gyro();
int16 Get_X_Gyro();
  //MPU3050��ʼ��
void  MPU3050_Init();
  //MMA8451��ʼ��
void  MMA8451_Init();


#endif 
