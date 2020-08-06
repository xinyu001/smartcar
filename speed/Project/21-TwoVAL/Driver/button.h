/*!
 * @file       LED.h
 * @brief      LED常用函数头文件
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

//#define BT_SHOW    PTB22
//#define BT_RIGHT   PTB23
//#define BT_UP      PTB20
//#define BT_DOWN    PTB21
//#define BT_YES     PTB3
//#define BT_LEFT    PTB2

#define BT_SHOW     PTB16
#define BT_RIGHT    PTC11
#define BT_UP       PTC9
#define BT_DOWN     PTC6
#define BT_YES      PTC7
#define BT_LEFT     PTC8


//#define SW1      PTA5
//#define SW2      PTA13
//#define SW3      PTA16
//#define SW4      PTA17

#define SW1      PTC17
#define SW2      PTC15
#define SW3      PTC13

#define SW1_IN   gpio_get(SW1)
#define SW2_IN   gpio_get(SW2)
#define SW3_IN   gpio_get(SW3)
//#define SW4_IN   gpio_get(SW4)

#define BEEP       PTD13
#define BEEP_ON    gpio_set (BEEP,0)
#define BEEP_OFF   gpio_set (BEEP,1)
#define BEEP_TURN  gpio_turn (BEEP)

#define BT_SHOW_IN   gpio_get(BT_SHOW)
#define BT_LEFT_IN   gpio_get(BT_LEFT)
#define BT_UP_IN     gpio_get(BT_UP)
#define BT_DOWN_IN   gpio_get(BT_DOWN)
#define BT_YES_IN    gpio_get(BT_YES)
#define BT_RIGHT_IN  gpio_get(BT_RIGHT)

void button_init();
void switch_init();

#endif /* _LED_H_ */