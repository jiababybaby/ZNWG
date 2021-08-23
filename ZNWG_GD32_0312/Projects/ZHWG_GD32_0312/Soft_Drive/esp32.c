#include "esp32.h"
#include "gd32f30x.h"
#include "string.h"
#include "flash.h"
#include "cJSON.h"
#include "w5500.h"
#include "me3630.h"
extern uint32_t mcuID[3];
uint8_t mode=0;
uint8_t mesh_id[10];
extern uint8_t usart2_rx_flag;
static char data[100]={0};
char UART2_RX_BUF[200];
uint16_t UART2_RX_CNT=0;
uint8_t esp32_config_flag=0;
uint8_t esp32_config_status=0;
uint8_t config_data[100];
extern uint8_t esp32_config_status,esp32_config_flag;
void esp32_config(){
	memset(config_data,0,sizeof(config_data));
	read_from_flash(config_data);
	canshu_get_from_Json((char *)config_data);
	memset(config_data,0,sizeof(config_data));
	canshu_write_to_Jsonstr((char *)config_data);
	usart2_sendstr((char *)config_data);
}
uint8_t config_ans_get_from_Json(char *data){
		uint8_t ret_val=0;
		cJSON *json_root  = NULL;
		json_root = cJSON_Parse((char *)data);
		cJSON *json_config_ans  = NULL;
		json_config_ans=cJSON_GetObjectItem(json_root, "config_ans");
		if(json_config_ans){
				ret_val=(uint8_t)json_config_ans->valueint;
		}
		if(!cJSON_IsInvalid(json_root)){
			cJSON_Delete(json_root);
		}
		return ret_val;
}
void uart2_deal(){
	if(esp32_config_status==0&&esp32_config_flag==1){
		esp32_config_flag=0;
		esp32_config();
	}
	if(usart2_rx_flag){
		usart2_rx_flag=0;
		strcpy(data,UART2_RX_BUF);
	
		memset(UART2_RX_BUF,0,sizeof(UART2_RX_BUF));
		if(config_ans_get_from_Json(data)){
			esp32_config_status=1;
		}
	  if(canshu_get_from_Json(data)){//配置信息
			char str_temp[100]={0};
			canshu_write_to_Jsonstr(str_temp);
			write_to_flash((uint8_t *)str_temp);
		}else{ //服务器上传信息
			if(mode==1){//w5500发送数据
				Write_SOCK_Data_Buffer(0, (uint8_t *)data,strlen(data));
				
			}else if(mode==2){ //4g发送数据
				memset(ME_tx_data,0,sizeof(ME_tx_data));
				str2str_TX(ME_tx_data,data);
				ME_TCP_TX();			
			}
		}
		
	
		memset(data,0,sizeof(data));
	
	
	}
	
}	
void esp32_init(void){
	
	/* USART interrupt configuration */
    nvic_irq_enable(USART2_IRQn, 0, 0);
	 /* enable GPIO clock */
		rcu_periph_clock_enable(RCU_GPIOB);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	
    /* USART configure */
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200u);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
		usart_stop_bit_set(USART2,USART_STB_1BIT);
		usart_word_length_set(USART2,USART_WL_8BIT);
		usart_parity_config(USART2,USART_PM_NONE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
	    /* enable USART0 receive interrupt */
    usart_interrupt_enable(USART2, USART_INT_RBNE);
	
}
int usart2_sendstr(char *p)
{
		while(*p!='\0'){
			usart_data_transmit(USART2,(uint8_t)*p);
			while (RESET == usart_flag_get(USART2, USART_FLAG_TBE));
			p++;
		}
		while (RESET == usart_flag_get(USART2, USART_FLAG_TC));
		return 0;
}
	
	
	