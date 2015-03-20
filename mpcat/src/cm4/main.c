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
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>

#define MPC_Thread_Space 1020
#define Com_Thread_Space 1028
#define MPC_Thread_Deadline 500 /* milliseconds */
//#define USER_BOTTOM /* uncomment to enable the function of user bottom */
static WORKING_AREA(waThreadMpc, MPC_Thread_Space);
static WORKING_AREA(waThreadCom, Com_Thread_Space);
Thread* CuThread;
Thread* ImThread;

/* current states of the emulated system */
#ifdef AIRCRAFT
struct aircraftpce_cvp cvp;
real_t states[MPC_STATES] = {0.0,0.0,0.0,-400.0,0.0}; /* initial state */
enum {SIM_POINTS = 40};  /* this should match the value in rs.py */
#endif

real_t inputs[MPC_HOR_INPUTS];
int32_t k = 0;  /* simulation mark */

struct mpc_data
{
  real_t states[MPC_STATES];
  real_t inputs[MPC_HOR_INPUTS];
  int32_t k;
} mpc_save[SIM_POINTS];


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
    uint16_t i;
    /* periodical thread setting */
    time += MS2ST(MPC_Thread_Deadline);
    /* Timer configure */
    TIM2_Configuration();
    /* Insert ms delay */
    mpcctl();
    /* error compensate and unit transformation*/
    MTime = TIM2_Read(); 
    /* send message to the print thread */
    chMsgSend(ImThread, (msg_t)MTime);
    /* save the data into RAM */
    if(k<SIM_POINTS)
    {
      for(i=0;i<MPC_STATES;i++)
        mpc_save[k].states[i] = states[i];
      for(i=0;i<MPC_HOR_INPUTS;i++)
        mpc_save[k].inputs[i] = inputs[i];
    }
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
  
  aircraftpce_initialize_problem_structure(&cvp);
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
