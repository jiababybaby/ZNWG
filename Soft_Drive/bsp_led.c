#include "gd32f30x.h"
#include "bsp_led.h"

void led_init(){

	rcu_periph_clock_enable(RCU_GPIOD);
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);	
}
void work_led_on(){
	gpio_bit_reset(GPIOD, GPIO_PIN_0);
}

void work_led_off(){
	gpio_bit_set(GPIOD, GPIO_PIN_0);
}
void work_led_toggle(){
	static uint8_t work_led_flag=0;
	work_led_flag++;
	if(work_led_flag%2){
		work_led_on();
	}else{
		work_led_off();
	}
}
