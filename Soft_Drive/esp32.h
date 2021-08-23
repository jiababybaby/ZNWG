#ifndef _ESP32_H
#define _ESP32_H
#include "gd32f30x.h"
#define ESP32_MAX_SIZE 500
extern char UART2_RX_BUF[200];
extern uint16_t UART2_RX_CNT;
void esp32_init(void);
int usart2_sendstr(char *p);
void uart2_deal();

#endif