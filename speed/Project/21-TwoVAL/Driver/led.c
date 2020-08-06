/*!
 * @file       LED.C
 * @brief      LED常用函数

 */
#include "common.h"
#include "include.h"
 /*
    函数作用 :板载led初始化
    注：核心板上有三个可编程LED，端口分别是 PTA19,PTE6,PTC18,低电平点亮
      端口定义在 led.h
 */  
void led_init()
{
//  gpio_init (LED_RED, GPO,1);
//  gpio_init (LED_GREEN, GPO,1); 
//  gpio_init (LED_BLUE, GPO,1);

   gpio_init (PTA17, GPO, 1u);    //初始化PTE0为高电平输出---LED0
   gpio_init (PTC5, GPO, 1u);    //初始化PTE0为高电平输出---LED1
   gpio_init (PTD15, GPO, 1u);    //初始化PTE0为高电平输出---LED2
   gpio_init (PTE24, GPO, 1u);    //初始化PTE0为高电平输出---LED3
}

void water_lights()  //从未被调用
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
  {  //反转引脚状态
    gpio_turn(PTA17);
    gpio_turn(PTC5);
    gpio_turn(PTD15);
    gpio_turn(PTE24);
    DELAY_MS(300);
  }
}