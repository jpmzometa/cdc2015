/**
 * MPC Algorithm Testing and Implementing code
 * function confirmation file: blinkout.h
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    blinkout.h
 * @brief   Debug and test file of MPC Algorithm Testing and Implementing code.
 * @details This file is the header of blinkout.c.  
 */
#ifndef __BLINKOUT_H
#define __BLINKOUT_H

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "lis302dl.h"

void Blinking_number_output(uint16_t number);
void Blinking_green(void);
void Blinking_orange(void);
void Blinking_blue(void);
void Blinking_delay(uint16_t time);
void Blinking_off(void);
void Blinking_test(void);

#endif
