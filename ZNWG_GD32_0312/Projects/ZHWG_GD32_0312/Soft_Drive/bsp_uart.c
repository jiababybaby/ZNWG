#include "bsp_uart.h"
#include "gd32f30x.h"
#include "stdio.h"
#include "cJSON.h"
#include "string.h"
#include "flash.h"
#include "w5500.h"
#include "me3630.h"
extern uint8_t w5500_send_flag;
char UART3_RX_BUF[100];
uint16_t uart3_rx_cnt;


extern uint8_t uart3_rx_flag;
static char data[50]={0};

extern uint8_t esp32_config_status,esp32_config_flag;
void uart3_deal(){
	if(uart3_rx_flag){
		uart3_rx_flag=0;
		memcpy(data,UART3_RX_BUF,100);
		
	  if(canshu_get_from_Json(data)){//配置信息
			char str_temp[100]={0};
			canshu_write_to_Jsonstr(str_temp);
			write_to_flash((uint8_t *)str_temp);
		}else{ //服务器上传信息
			if(mode==1){//w5500 socket0发送数据				
				Write_SOCK_Data_Buffer(0,(uint8_t*)data,strlen(data));
			}else if(mode==2){ //4g发送数据
				str2str_TX(ME_tx_data,data);
				ME_TCP_TX();			
			}
		}
		
	
		memset(data,0,sizeof(data));
	}
}	

int uart3_sendstr(char *p)
{
		TX_EN();
		while(*p!='\0'){
			usart_data_transmit(UART3,(uint8_t)*p);
			while (RESET == usart_flag_get(UART3, USART_FLAG_TBE));
			p++;
		}
		while (RESET == usart_flag_get(UART3, USART_FLAG_TC));
		RX_EN();
		return 0;
}


void uart_init(){
/* USART interrupt configuration */
    nvic_irq_enable(UART3_IRQn, 0, 0);
	 /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
		rcu_periph_clock_enable(RCU_GPIOD);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART3);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	
		gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
		RX_EN();
    /* USART configure */
    usart_deinit(UART3);
    usart_baudrate_set(UART3, 115200u);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
		usart_stop_bit_set(UART3,USART_STB_1BIT);
		usart_word_length_set(UART3,USART_WL_8BIT);
		usart_parity_config(UART3,USART_PM_NONE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_enable(UART3);
	    /* enable USART0 receive interrupt */
    usart_interrupt_enable(UART3, USART_INT_RBNE);

}