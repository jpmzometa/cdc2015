/**
 * MPC Algorithm Testing and Implementing code
 * communication format file: test.c
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    test.c
 * @brief   Support file of MPC Algorithm Testing and Implementing code.
 * @details This file defines the format of data transmission via serial port.  
 */

#include "ch.h"
#include "hal.h"
#include "test.h"
#include <mpc_const.h>
#include <pce.h>

/*
 * Console output.
 */
static BaseChannel *chp;

/**
 * @brief   Prints a decimal unsigned number.
 *
 * @param[in] n         the number to be printed
 */
void test_printn(uint32_t n) {
  char buf[16], *p;

  if (!n)
    chIOPut(chp, '0');
  else {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      chIOPut(chp, *--p);
  }
}

/**
 * @brief   Prints a line without final end-of-line.
 *
 * @param[in] msgp      the message
 */
void test_print(const char *msgp) {

  while (*msgp)
    chIOPut(chp, *msgp++);
}

/**
 * @brief   Prints a line.
 *
 * @param[in] msgp      the message
 */
void test_println(const char *msgp) {

  test_print(msgp);
  chIOPut(chp, '\r');
  chIOPut(chp, '\n');
}

/**
 * @brief   Prints a float number.
 *
 * @param[in] float32_t      the message
 */
void usr_fprint(float *number)
{
   char *segment, i = 4;
   segment = (char *)number;
   while(i)
   {
      chIOPut(chp, *segment++);
      i--;
   }
}

/**
 * @brief   Prints a line.
 *
 * @param[in] int32_t      the message
 */
void usr_intprint(int32_t *number)
{
   char *segment, i = 4;
   segment = (char *)number;
   while(i)
   {
      chIOPut(chp, *segment++);
      i--;
   }
}

/**
 * @brief   Prints a line.
 *
 * @param[in] msgp      the message
 *            void *    point to the serial port
 */
void output_number(void *p, msg_t Time)
{
	chp = p;
        int16_t i;
        extern real_t states[];
        extern real_t inputs[];
        extern int32_t k;
        usr_intprint(&k);
        usr_intprint(&Time);
        for(i=0; i<PCE_NX; i++)
          usr_fprint(&states[i]);
        for(i=0; i<MPC_HOR_INPUTS; i++)
          usr_fprint(&inputs[i]);
}
