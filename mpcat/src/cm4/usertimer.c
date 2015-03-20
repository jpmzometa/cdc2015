/**
 * MPC Algorithm Testing and Implementing code
 * timer configuration file: usertimer.c
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    usertimer.c
 * @brief   Timer configuration of MPC Algorithm Testing and Implementing code.
 * @details This file helps to configure and enable Timer2(TIM2) and also read
 *          the counting number in timer register.  
 * @details Error should be measured and compensated
 */

#include <usertimer.h>

void TIM2_Configuration(void)
{
 	RCC->APB1ENR |= RCC_APB1RSTR_TIM2RST;
        //TIM2->PSC=2400/2-1; /* used for 24MHz core frquency */
	TIM2->PSC = 168/2-1; /* used for 168MHz core frquency */
	TIM2->ARR = 65000;
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint16_t TIM2_Read(void)
{
 	uint16_t Time;
	TIM2->CR1 = 0x0; 
	Time = TIM2->CNT;
        TIM2->CNT = 0x0000;
	return(Time);
}
