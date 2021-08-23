#ifndef _BSP_LED_H
#define _BSP_LED_H
#include "gd32f30x.h"
void led_init(void);

void work_led_on(void);

void work_led_off(void);

void work_led_toggle(void);
extern uint8_t work_led_flag;
#endif