/**
 * MPC Algorithm Testing and Implementing code
 * function confirmation file: blinkout.c
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    blinkout.c
 * @brief   Debug and test file of MPC Algorithm Testing and Implementing code.
 * @details This file defines several visual function in order to test if any 
 *          thread or function is executed successfully.  
 */

#include <blinkout.h>

void Blinking_number_output(uint16_t number)
{
 	static uint16_t temp[16];
	uint16_t i;
	for(i=0;i<16;i++)
	{
		temp[i] = number & (1 << i);
	}
	for(i=0;i<16;i++)
	{
	 	if(temp[i] == 0)
			Blinking_orange();
		else
			Blinking_blue();
		Blinking_delay(500);
		Blinking_off();
		Blinking_delay(500);
	}
}

void Blinking_green(void)
{
    palSetPad(GPIOD, GPIOD_LED4); 
}

void Blinking_orange(void)
{
   palSetPad(GPIOD, GPIOD_LED3);
}

void Blinking_blue(void)
{
   palSetPad(GPIOD, GPIOD_LED6);
}

void Blinking_off(void)
{
    palClearPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    palClearPad(GPIOD, GPIOD_LED6);     /* Blue.  */
    palClearPad(GPIOD, GPIOD_LED4);     /* Green.  */
}

void Blinking_delay(uint16_t time)
{
    chThdSleepMilliseconds(time);
}

void Blinking_test(void)
{
 	Blinking_orange();
	Blinking_blue();
	Blinking_delay(20);
	Blinking_off();
}
