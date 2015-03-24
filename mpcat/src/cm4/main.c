/**
 * MPC Algorithm Testing and Implementing code
 * Main file: main.c
 *                                      by Weichen Li
 &                                      2012.8.18
 * @file    main.c
 * @brief   Main file of MPC Algorithm Testing and Implementing code.
 * @details This file is modified from ChibiOS/RT main file.
 * @details This files contains three threads: Main thread(totally empty after
 *          initialization),Threa1 and Thread2.
 *
 */

#include "ch.h"
#include "hal.h"
#include "lis302dl.h"
#include <test.h>
#include <usertimer.h>
#include <blinkout.h>
#include <mpcctl.h>

#ifdef CMPC
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <pce.h>
#endif

#define MPC_Thread_Space 32020
#define Com_Thread_Space 2028
#define MPC_Thread_Deadline 100 /* milliseconds */
//#define USER_BOTTOM /* uncomment to enable the function of user bottom */
static WORKING_AREA(waThreadMpc, MPC_Thread_Space);
static WORKING_AREA(waThreadCom, Com_Thread_Space);
Thread* CuThread;
Thread* ImThread;

/* current states of the emulated system */
#ifdef AIRCRAFT
extern struct aircraftpce_cvp cvp;
real_t states[PCE_NX] = {0.0, 0.0, 0.0, -401.0, 0.0}; /* initial state */
real_t inputs[PCE_HOR*1];
enum {SIM_POINTS = 5};  /* this should match the value in rs.py */
#endif
#ifdef AIRCRAFTNOM
real_t states[MPC_STATES] = {0.0, 0.0, 0.0, -401.0, 0.0}; /* initial state */
real_t inputs[MPC_HOR_INPUTS];
enum {SIM_POINTS = 60};  /* this should match the value in rs.py */
#endif

int32_t k = 0;  /* simulation mark */

#if 0
struct mpc_data
{
  real_t states[PCE_JAC_ROWS*PCE_HOR];
  real_t inputs[PCE_HOR];
  int32_t k;
} mpc_save[SIM_POINTS];
#endif


/*
 * @biref     Executing time measurement thread
 * @param     None
 * @retval    None
 * @message   Snychronous message to Thread2
 * @notes     Thread is executed every 0.5 second
 */
static msg_t ThreadMpc(void *arg) {

  (void)arg;
  systime_t time = chTimeNow(); 
  chRegSetThreadName("TMeasure");
  while (TRUE) {
    uint16_t MTime = 0;
    /* periodical thread setting */
    time += MS2ST(MPC_Thread_Deadline);

    /* send message to the print thread */
    /* MTime contains the execution time of previous iteration */
    chMsgSend(ImThread, (msg_t)MTime);
    /* Timer configure */
    TIM2_Configuration();
    /* Insert ms delay */
    /* Use this to check the time you are measuring is right 
     * chThdSleepMilliseconds(123);
     */
    mpcctl();
    /* error compensate and unit transformation*/
    MTime = TIM2_Read(); 
    /* save the data into RAM */
#if 0
    if(k<SIM_POINTS)
    {
      for(i=0;i<PCE_NX;i++) {
        mpc_save[k].states[i] = states[i];
	      }
      for(i=0;i<PCE_HOR;i++){
        mpc_save[k].inputs[i] = inputs[i];
      }
    }
#endif
    k++;
    chThdSleepUntil(time);
  }
}

/*
 * @biref     Print the data through USART2
 * @param     None
 * @retval    None
 * @message   Snychronous message from Thread1
 * @notes     Data will be printed out while hoding user button
 *            Execution of thread is confirmed by green LED
 */
static msg_t ThreadCom(void *arg) {
  (void)arg;
  chRegSetThreadName("DataPrint");
  while (TRUE) {
    msg_t Time;
    /* waiting for the message from other thread */
    chMsgWait();
    Time = chMsgGet(CuThread);
    /* release the sending hold of sender thread */
    chMsgRelease(CuThread, Time);
#ifdef USER_BOTTOM
    if (palReadPad(GPIOA, GPIOA_BUTTON))
    {
      Blinking_green();
      /* print number via Serial Port */
      Output_number(&SD2, Time);
      Blinking_off();
    }
#else
    Blinking_green();
    /* print number via Serial Port */
    output_number(&SD2, Time);
    Blinking_off();
#endif
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
#if 0 
  aircraftpce_initialize_problem_structure(&cvp);
#endif
  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Creates the example thread.
   */
  ImThread = chThdCreateStatic(waThreadCom, sizeof(waThreadCom), 
		NORMALPRIO, ThreadCom, NULL);
  CuThread = chThdCreateStatic(waThreadMpc, sizeof(waThreadMpc), 
		NORMALPRIO, ThreadMpc, NULL);
  
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop.
   */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
