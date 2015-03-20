/**
 * MPC Algorithm Testing and Implementing code
 * timer configuration file: usertimer.h
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    usertimer.h
 * @brief   Timer configuration of MPC Algorithm Testing and Implementing code.
 * @details This file is the header of usertimer.c.
 */

#ifndef __USERTIMER_H__
#define __USERTIMER_H__

#include <stm32f4xx.h>

void TIM2_Configuration(void);
uint16_t TIM2_Read(void);

#endif
