#ifndef _ME3630_H
#define _ME3630_H
#include "gd32f30x.h"

#define ME_MAX_SIZE 500
typedef struct
{
	unsigned int write_pos; //–¥
	unsigned int read_pos;	//∂¡
	unsigned char rev_status;
	char rev_buf[ME_MAX_SIZE]; //Ω” ’ª∫¥Ê
} ME_IO_INFO;

extern ME_IO_INFO meioinfo;
extern unsigned char ME_INIT_FLAG;
extern unsigned char ME_SOCKET_CLOSE_FLAG;
extern unsigned char ME_SOCKET_CREAT_FLAG;
extern unsigned char ME_TCP_CONNECT_FLAG;
extern unsigned char ME_TCP_TX_FLAG;
extern char ME_tx_data[1000];
extern uint8_t ME_time_flag;
void me3630_init(void);
void ME_TCP_CONNECT(void);
int16_t ME_Solve(void);
void ME_TCP_TX(void);
void str2str_TX(char *dest, const char *str);
void str2str_RX(char *dest, const char *str);
int uart1_sendstr(char *p);
extern char UART1_RX_BUF[500];
extern uint16_t uart1_rx_cnt;
#endif
