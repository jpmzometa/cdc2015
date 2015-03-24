#ifdef PC_TEST
#include <stdio.h>
#endif
#include "mpcctl.h"  /* the auto-generated code */
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <pce.h>
#include <mpcpce.h>


void mpcctl(void)
{
      extern real_t states[];
      extern real_t inputs[];
      extern struct mpc_ctl ctl;
      extern struct aircraftpce_cvp cvp;
      static int init = 1;

      uint32_t i; /* loop iterator */
      if (init) {
              ctl.conf->in_iter = 5;  /* iterations internal loop */
              ctl.conf->ex_iter = 10;  /* iterations external loop */
              ctl.conf->warmstart = 1;  /* warmstart each iteration */
              init = 0;
      }
#if 0
      mpc_ctl_solve_problem(&ctl, states);  /* solve the MPC problem */
#else
      mpcpce_solve_problem();
#endif
#if 0
      for (i=0; i<MPC_STATES; i++) {
              cvp.prb->x_k->data[i] = states[i];
      }
      aircraftpce_cvp_form_problem(&cvp);
      mpc_predict_next_state(&ctl, states);
      for(i=0; i<MPC_HOR_INPUTS; i++) {
              inputs[i] = ctl.u_opt[i];
      }
#endif
      return;
}

