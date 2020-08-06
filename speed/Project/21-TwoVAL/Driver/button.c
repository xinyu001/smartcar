/*!
 * @file       LED.C
 * @brief      LED���ú���

 */
#include "common.h"
#include "include.h"
#include "button.h"
 /*
    �������� :����led��ʼ��
    ע�����İ����������ɱ��LED���˿ڷֱ��� PTA19,PTE6,PTC18,�͵�ƽ����
      �˿ڶ����� led.h
 */  

void button_init()
{
  gpio_init (BT_SHOW, GPI,1);
  port_init_NoALT (BT_SHOW, PULLUP );  
  gpio_init (BT_LEFT, GPI,1);
  port_init_NoALT (BT_LEFT, PULLUP ); 
  gpio_init (BT_UP, GPI,1);
  port_init_NoALT (BT_UP, PULLUP ); 
  gpio_init (BT_DOWN, GPI,1);
  port_init_NoALT (BT_DOWN, PULLUP );
  gpio_init (BT_YES, GPI,1);
  port_init_NoALT (BT_YES, PULLUP ); 
  gpio_init (BT_RIGHT, GPI,1);
  port_init_NoALT (BT_RIGHT, PULLUP ); 
 
  gpio_init(BEEP,GPO,1);
}

void switch_init()
{
  gpio_init (SW1, GPI,1);
  port_init_NoALT (SW1, PULLUP );  
  gpio_init (SW2, GPI,1);
  port_init_NoALT (SW2, PULLUP ); 
  gpio_init (SW3, GPI,1);
  port_init_NoALT (SW3, PULLUP ); 
//  gpio_init (SW4, GPI,1);
//  port_init_NoALT (SW4, PULLUP );

}


