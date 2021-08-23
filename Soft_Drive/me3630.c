#include "me3630.h"
#include "gd32f30x.h"
#include <stdio.h>
#include "string.h"
#include "esp32.h"
///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART1, (uint8_t)ch);
	while (RESET == usart_flag_get(USART1, USART_FLAG_TC))
		;
	return (ch);
}
void me3630_init(void)
{
	/* USART interrupt configuration */
	nvic_irq_enable(USART1_IRQn, 0, 0);
	/* enable GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);
	/* enable USART clock */
	rcu_periph_clock_enable(RCU_USART1);

	/* connect port to USARTx_Tx */
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

	/* connect port to USARTx_Rx */
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

	/* USART configure */
	usart_deinit(USART1);
	usart_baudrate_set(USART1, 9600);
	usart_receive_config(USART1, USART_RECEIVE_ENABLE);
	usart_stop_bit_set(USART1, USART_STB_1BIT);
	usart_word_length_set(USART1, USART_WL_8BIT);
	usart_parity_config(USART1, USART_PM_NONE);
	usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
	usart_enable(USART1);
	/* enable USART0 receive interrupt */
	usart_interrupt_enable(USART1, USART_INT_RBNE);
}
char UART1_RX_BUF[500];
uint16_t uart1_rx_cnt;
/* retarget the C library printf function to the USART */
int uart1_sendstr(char *p)
{
	while (*p != '\0')
	{
		usart_data_transmit(USART1, (uint8_t)*p);
		while (RESET == usart_flag_get(USART1, USART_FLAG_TC))
			;
		p++;
	}
	while (RESET == usart_flag_get(USART1, USART_FLAG_TC))
		;
	return 0;
}

unsigned char ME_INIT_FLAG = 0;
unsigned char ME_SOCKET_CLOSE_FLAG = 0;
unsigned char ME_SOCKET_CREAT_FLAG = 0;
unsigned char ME_IMEI_SEARCH_FLAG = 0;
unsigned char ME_TCP_CONNECT_FLAG = 0;
unsigned char ME_TCP_TX_FLAG = 0;

char ME_tx_data[1000];
char ME_TCP_CLOCK[1] = {'a'};
ME_IO_INFO meioinfo;

void ME_IO_Clear_Cache(void)
{
	meioinfo.write_pos = 0;
	meioinfo.read_pos = 0;
	meioinfo.rev_status = 0;
	memset(meioinfo.rev_buf, 0, sizeof(meioinfo.rev_buf));
}

uint8_t debug_ME_flag = 1;

extern uint8_t usart1_rx_flag;
uint8_t ME_time = 1;
uint8_t ME_time_flag = 1;
uint8_t connect_on_cnt = 0;
void ME_TCP_CONNECT(void)
{
	if (!ME_TCP_CONNECT_FLAG)
	{
		if (ME_time_flag)
		{
			static uint16_t error_cnt;
			switch (ME_time)
			{
			case 0:
				//						++error_cnt;
				//						if(error_cnt>4)
				//						{
				//							error_cnt=0;
				ME_time++;
				//						}
				ME_time_flag = 0;
				break;
			case 1:
				printf("AT+IPR?\r\n");
				ME_time_flag = 0;
				ME_time++;
				break;
			case 2:
				printf("AT+CPIN?\r\n");
				ME_time_flag = 0;
				ME_time++;
				break;
			case 3: //printf("AT+ZIPCFG=cmnet,None,None\r\n");
				printf("AT^SYSINFO\r\n");
				ME_time_flag = 0;
				ME_time++;
				break;
			case 4:
				printf("AT+ZIPCALL=1\r\n");
				ME_time_flag = 0;
				ME_time++;
				break;
			case 5: //printf("AT+ZIPALIVE=1,1,300,75,9\r\n");
				ME_time_flag = 0;
				ME_time++;
				break;
			case 6:
				if (debug_ME_flag)
					printf("AT+ZIPOPEN=1,0,103.46.128.45,41740\r\n");
				else
					printf("AT+ZIPOPEN=1,0,47.111.112.30,10087\r\n");
				ME_time_flag = 0;
				connect_on_cnt = 0;
				break;
			case 7:
				printf("AT+ZIPCALL=0\r\n");
				ME_time = 0;
				break;
			case 8:
				printf("AT+ZIPCALL=0\r\n");
				ME_time = 0;
				break;
			default:
				break;
			}
		}
	}
	if (ME_TCP_CONNECT_FLAG && connect_on_cnt < 3)
	{

		if (ME_time_flag)
		{
			connect_on_cnt++;
			switch (connect_on_cnt)
			{
			case 1:
				printf("AT+ZIPALIVE=1,1,300,75,9\r\n");
				break;
			case 2:
				str2str_TX(ME_tx_data, "connect");
				ME_TCP_TX();
				break;
			case 3:
				str2str_TX(ME_tx_data, "connect");
				ME_TCP_TX();
				break;
			default:
				break;
			}
			ME_time_flag = 0;
		}
	}
}
extern uint8_t ble_connect_flag;
void ME_TCP_TX(void) //3630
{
	if (ME_TCP_CONNECT_FLAG)
		printf("AT+ZIPSEND=1,%s\r\n", ME_tx_data);
}
/*
字符串的十六进制字符串转字符串
例如：“313233”转换成“123”

全网通版本接收数据可不用此函数。
*/
void str2str_RX(char *dest, const char *str)
{
	for (uint8_t i = 0, j = 0; i < strlen(str); i++, j += 2)
	{
		uint8_t temp1, temp2;
		if (str[j] >= '0' && str[j] <= '9')
		{
			temp1 = str[j] - '0';
		}
		else if (str[j] >= 'a' && str[j] <= 'f')
		{
			temp1 = str[j] - 'a' + 0X0A;
		}
		if (str[j + 1] >= '0' && str[j + 1] <= '9')
		{
			temp2 = str[j + 1] - '0';
		}
		else if (str[j + 1] >= 'a' && str[j + 1] <= 'f')
		{
			temp2 = str[j + 1] - 'a' + 0X0A;
		}
		dest[i] = temp1 << 4 | temp2;
	}
}
/*
字符串转字符串的十六进制字符串
例如：“123”转换成“313233”
发送数据需要此函数
*/
void str2str_TX(char *dest, const char *str)
{
	for (uint16_t i = 0, j = 0; i < strlen(str); i++, j += 2)
	{
		uint8_t temp1, temp2;
		temp1 = (str[i] & 0xf0) >> 4;
		temp2 = (str[i] & 0x0f);
		if (temp1 >= 0 && temp1 <= 9)
		{
			dest[j] = temp1 + '0';
		}
		else if (temp1 >= 0x0a && temp1 <= 0x0f)
		{
			dest[j] = temp1 - 0x0a + 'a';
		}
		if (temp2 >= 0 && temp2 <= 9)
		{
			dest[j + 1] = temp2 + '0';
		}
		else if (temp2 >= 0x0a && temp2 <= 0x0f)
		{
			dest[j + 1] = temp2 - 0x0a + 'a';
		}
	}
}

uint8_t ME_data[ME_MAX_SIZE];
uint16_t ME_data_cnt;
static uint8_t data_rev_flag;

void ME_data_Clear_Cache(void)
{
	ME_data_cnt = 0;
	memset(ME_data, 0, sizeof(ME_data));
}
extern uint8_t ME_time_flag;
extern uint8_t ME_time;
char rx_temp[200];
uint8_t connect_flag_temp;
extern uint8_t ME_TCP_CONNECT_FLAG;
int16_t ME_Solve()
{
	if (meioinfo.read_pos != meioinfo.write_pos)
	{
		ME_data[ME_data_cnt++] = meioinfo.rev_buf[meioinfo.read_pos++];
		if (ME_data_cnt == (ME_MAX_SIZE))
		{
			ME_data_cnt = 0;
		}
		if (meioinfo.read_pos == (ME_MAX_SIZE))
		{
			meioinfo.read_pos = 0;
		}
		if (ME_data_cnt > 2 && (ME_data[ME_data_cnt - 1] == 0x0d || ME_data[ME_data_cnt - 1] == 0x0a))
		{
			ME_data_cnt = 0;
			data_rev_flag = 1;
		}
	}
	if (data_rev_flag)
	{
		if (strstr((const char *)ME_data, "+ZREADY"))
		{
			//ME_time_flag=1;
		}
		if (strstr((const char *)ME_data, "OK"))
		{
			//ME_time_flag=1;
		}
		if (ME_time == 6 && strstr((const char *)ME_data, "ERROR"))
		{
			ME_time = 7;
			ME_data_Clear_Cache();
			data_rev_flag = 0;
		}
		if (strstr((const char *)ME_data, "OK"))
		{
			//			ME_time_flag=1;
		}
		if (strstr((const char *)ME_data, "ERROR"))
		{
			//printf("AT+ZRST");
			//			ME_time_flag=1;
		}
		if (strstr((const char *)ME_data, "+CSQ:"))
		{
			//ME_time_flag=1;
		}
		if (strstr((const char *)ME_data, "+CREG:"))
		{
			//ME_time_flag=1;
		}
		if (strstr((const char *)ME_data, "+ZIPSTAT: 1,1"))
		{
			ME_TCP_CONNECT_FLAG = 1;
		}
		if (strstr((const char *)ME_data, "+ZIPSTAT: 1,0"))
		{
			ME_TCP_CONNECT_FLAG = 0;
			ME_time_flag = 0;
			ME_time = 1;
		}
		char *p;
		//		if((p=strstr((const char *)ME_data,"BLEMAC:"))){
		//			//uint8_t data_temp[25]={0};
		//			strncpy((char *)ble_mac+1,(const char *)p+7,17);
		//			ble_connect_flag=0;
		//			//memcpy(ble_mac+ 2,data_temp,17);
		//			ble_mac_rx_flag=1;
		//			ME_data_Clear_Cache();
		//			data_rev_flag=0;
		//		}
		if (strstr((const char *)ME_data, "+ZIPRECV"))
		{
			memset(rx_temp, 0, sizeof(rx_temp));
			strcpy(rx_temp, (const char *)ME_data + 35);
			usart2_sendstr(rx_temp);
		}
		if (strstr((const char *)ME_data, "+ZIPSTAT"))
		{
			//uint8_t data_temp[25]={0};
			connect_flag_temp = ME_data[12] - '0';
		}
		memset(rx_temp, 0, sizeof(rx_temp));
		ME_data_Clear_Cache();
		data_rev_flag = 0;
		//ME_URC_READ();
	}
	return 0;
}
