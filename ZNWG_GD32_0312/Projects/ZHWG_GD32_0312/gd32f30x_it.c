/*!
    \file  gd32f30x_it.c
    \brief interrupt service routines

    \version 2017-02-10, V1.0.0, demo for GD32F30x
    \version 2018-10-10, V1.1.0, demo for GD32F30x
    \version 2018-12-25, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "systick.h"
#include "bsp_led.h"
#include "w5500.h"
#include "me3630.h"
#include "bsp_uart.h"
#include "me3630.h"
#include "esp32.h"
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}
/*!
    \brief      this function handles USART0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint8_t uart3_rx_done=0;
uint8_t uart3_rx_flag=0;
uint32_t uart3_rx_done_cnt=0;
uint32_t usart2_rx_done_cnt=0;
void UART3_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE)){
        /* read one byte from the receive data register */
        UART3_RX_BUF[uart3_rx_cnt++]=usart_data_receive(UART3);
				if(uart3_rx_cnt==500){
					uart3_rx_cnt=0;
				}
				uart3_rx_done=1;
				uart3_rx_done_cnt=0;
    }
       
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_TBE)){
 
    }
}

void USART1_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){
			meioinfo.rev_buf[meioinfo.write_pos++]=usart_data_receive(USART1);
			meioinfo.rev_status=1;
			if(meioinfo.write_pos==ME_MAX_SIZE){
				meioinfo.write_pos=0;
			}
    }
       
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)){
 
    }
}
uint8_t usart2_rx_done=0;
uint8_t usart2_rx_flag=0;
void USART2_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)){
			UART2_RX_BUF[UART2_RX_CNT++]=usart_data_receive(USART2);
			if(UART2_RX_CNT==500){
				UART2_RX_CNT=0;
			}
			usart2_rx_done=1;
			usart2_rx_done_cnt=0;
    }
       
    if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_TBE)){
 
    }
}



/*
定时器0中断 500us一次。用作基本时钟定时

*/
uint32_t time_1sec_buf=0;
uint32_t web_buf=0;
uint32_t me_buf=0;



extern uint8_t esp32_config_flag;
extern uint8_t esp32_config_status;
uint32_t esp32_config_cnt=0;
void TIMER0_UP_TIMER9_IRQHandler(void){
	if(RESET!=timer_interrupt_flag_get(TIMER0,TIMER_INT_FLAG_UP)){
		timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_UP);
		time_1sec_buf++;
		web_buf++;
		me_buf++;
		if(time_1sec_buf==2000){
			time_1sec_buf=0;
			work_led_flag=1;
		}
		if(web_buf==20){ //10ms
			web_buf=0;
			web_flag=1;
		}
		if(me_buf==3000){
			me_buf=0;
			ME_time_flag=1;
		}
		if(uart3_rx_done==1){
			uart3_rx_done_cnt++;
		}
		if(uart3_rx_done_cnt==50){
			uart3_rx_cnt=0;
			uart3_rx_done=0;
			uart3_rx_done_cnt=0;
			uart3_rx_flag=1;
		}
		if(usart2_rx_done==1){
			usart2_rx_done_cnt++;
		}
		if(usart2_rx_done_cnt==50){
			UART2_RX_CNT=0;
			usart2_rx_done=0;
			usart2_rx_done_cnt=0;
			usart2_rx_flag=1;	
		}
		if(esp32_config_status==0&&esp32_config_flag==0){
			esp32_config_cnt++;
		}
		if(esp32_config_cnt==2000){
			esp32_config_flag=1;
			esp32_config_cnt=0;
		}
	}
}