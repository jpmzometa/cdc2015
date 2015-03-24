#ifdef PC_TEST
#include <stdio.h>
#endif
#include "mpcctl.h"  /* the auto-generated code */

#ifdef CMPC 
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <pce.h>
#include <mpcpce.h>
#endif


void mpcctl(void)
{
      extern real_t states[];
      extern real_t inputs[];
      extern struct mpc_ctl ctl;
      static int init = 1;

      uint32_t i; /* loop iterator */
#ifdef CMPCNOM 
      if (init) {
              ctl.conf->in_iter = 5;  /* iterations internal loop */
              ctl.conf->ex_iter = 10;  /* iterations external loop */
              ctl.conf->warmstart = 1;  /* warmstart each iteration */
              init = 0;
      }
      mpc_ctl_solve_problem(&ctl, states);  /* solve the MPC problem */
      mpc_predict_next_state(&ctl, states);
      for(i=0; i<MPC_HOR_INPUTS; i++) {
              inputs[i] = ctl.u_opt[i];
      }
#endif

#ifdef CMPC
      extern struct aircraftpce_cvp cvp;
      mpcpce_solve_problem();
#endif
      return;
}

