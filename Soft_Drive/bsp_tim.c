#include "bsp_tim.h"

#include "gd32f30x.h"
#include <stdio.h>
#include "systick.h"



/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer0_config(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 500;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);
    timer_enable(TIMER0);
		nvic_irq_enable(TIMER0_UP_TIMER9_IRQn,0,0);
		timer_interrupt_enable(TIMER0,TIMER_INT_UP);
}