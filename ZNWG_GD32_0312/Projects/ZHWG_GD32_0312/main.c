#include <stdio.h>
#include "gd32f30x.h"
#include "systick.h"
#include "gd25qxx.h"
#include "gd32f307c_eval.h"
#include "w5500.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "bsp_uart.h" 
#include "me3630.h"
#include "esp32.h"
#include "flash.h"
#include "cJSON.h"
#include "string.h"
#define  VERSION_ID                    "1.0.0"
#define  SFLASH_ID                     0xC84015
#define  FLASH_WRITE_ADDRESS           0x000000
#define  FLASH_READ_ADDRESS            FLASH_WRITE_ADDRESS

ErrStatus memory_compare(uint8_t* src,uint8_t* dst,uint16_t length);
int fputc(int ch,FILE *f);
uint32_t mcuID[3];
extern uint8_t mesh_id[10];
void cpuidGetId(void)
{
    mcuID[0] = *(volatile uint32_t*)(0x1FFFF7E8);
    mcuID[1] = *(volatile uint32_t*)(0x1FFFF7EC);
    mcuID[2] = *(volatile uint32_t*)(0x1FFFF7F0);
		sprintf((char *)mesh_id,"%x%x%x",(mcuID[2]&0x00ff0000)>>16,(mcuID[2]&0x0000ff00)>>8,(mcuID[2]&0xff));
//		memcpy((char *)mesh_id,(uint8_t*)mcuID+6,6);
}
char default_canshu[100]="\"mesh_channel\":1,\"mode\":0,\"ssid\":\"baby\",\"pwd\":\"1234567890\"}";
uint8_t default_mode=0;
uint8_t canshu[300]={0};
void get_local_canshu(){
		read_from_flash(canshu);
		if(canshu_get_from_Json((char *)canshu)!=1){
			mode=default_mode;
			canshu_write_to_Jsonstr((char *)canshu);
			write_to_flash(canshu);
			
		}
}
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint8_t web_flag=0;
uint8_t work_led_flag=0;
uint8_t net_flag=0;

int main(void)
{
    /* systick configuration*/

    systick_config();
		cpuidGetId();
		cJSON_InitHooks(NULL);
		get_local_canshu();
		uart_init();
		me3630_init();
		esp32_init();
		led_init();
		Load_Net_Parameters();
		W5500_GPIO_Configuration();
		W5500_Initialization();	
		timer0_config();

    while(1){
			uart2_deal();
			uart3_deal(); 
			if(web_flag==1){
				web_flag=0;
				WEB();
			}
			if(work_led_flag){
				work_led_flag=0;
				work_led_toggle();
			}
			ME_TCP_CONNECT();
			ME_Solve();
    }
}






