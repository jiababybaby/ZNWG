#ifndef _BSP_UART_H
#define _BSP_UART_H
#include "gd32f30x.h"
#define TX_EN() gpio_bit_set(GPIOD, GPIO_PIN_1)
#define RX_EN() gpio_bit_reset(GPIOD, GPIO_PIN_1)

extern char UART3_RX_BUF[100];
extern uint16_t uart3_rx_cnt;

void uart_init();
int uart3_sendstr(char *p);
void uart3_deal(void);
#endif