/*!
 * @file       LED.C
 * @brief      LED���ú���

 */
#include "common.h"
#include "include.h"
 /*
    �������� :����led��ʼ��
    ע�����İ����������ɱ��LED���˿ڷֱ��� PTA19,PTE6,PTC18,�͵�ƽ����
      �˿ڶ����� led.h
 */  
void led_init()
{
//  gpio_init (LED_RED, GPO,1);
//  gpio_init (LED_GREEN, GPO,1); 
//  gpio_init (LED_BLUE, GPO,1);

   gpio_init (PTA17, GPO, 1u);    //��ʼ��PTE0Ϊ�ߵ�ƽ���---LED0
   gpio_init (PTC5, GPO, 1u);    //��ʼ��PTE0Ϊ�ߵ�ƽ���---LED1
   gpio_init (PTD15, GPO, 1u);    //��ʼ��PTE0Ϊ�ߵ�ƽ���---LED2
   gpio_init (PTE24, GPO, 1u);    //��ʼ��PTE0Ϊ�ߵ�ƽ���---LED3
}

void water_lights()  //��δ������
{
  LED_GREEN_ON;
  LED_RED_OFF;
  LED_BLUE_OFF;
  DELAY_MS(150);
  LED_GREEN_OFF;
  LED_RED_ON;
  LED_BLUE_OFF;
  DELAY_MS(150);
  LED_GREEN_OFF;
  LED_RED_OFF;
  LED_BLUE_ON;
  DELAY_MS(150);
}
void led_flash()
{ unsigned char i;
//  LED_GREEN_ON;
//  LED_RED_ON;
//  LED_BLUE_ON;
//  DELAY_MS(500);
//  LED_GREEN_OFF;
//  LED_RED_OFF;
//  LED_BLUE_OFF;
//  DELAY_MS(500);'
  for(i=0;i<6;i++)
  {  //��ת����״̬
    gpio_turn(PTA17);
    gpio_turn(PTC5);
    gpio_turn(PTD15);
    gpio_turn(PTE24);
    DELAY_MS(300);
  }
}