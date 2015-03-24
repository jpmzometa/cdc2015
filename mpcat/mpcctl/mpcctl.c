#ifdef PC_TEST
#include <stdio.h>
#include <sys/time.h>
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
              ctl.conf->ex_iter = 20;  /* iterations external loop */
              ctl.conf->warmstart = 1;  /* warmstart each iteration */
              init = 0;
      }
      mpc_ctl_solve_problem(&ctl, states);  /* solve the MPC problem */
      mpc_predict_next_state(&ctl, states);
#endif

#ifdef CMPC
      extern struct aircraftpce_cvp cvp;
      if (init) {
              ctl.conf->in_iter = 5;  /* iterations internal loop */
              ctl.conf->ex_iter = 20;  /* iterations external loop */
	    static real_t mu = 1.4;
	    static real_t Linv = 0.030021378282011981;
	    static real_t nu = 0.79996534344746051;

        ctl.alm->mu = &mu;
        ctl.alm->Linv = &Linv;
        ctl.alm->fgm->nu = &nu;
	    aircraftpce_initialize_problem_structure(&cvp);
              init = 0;
      }
      mpcpce_solve_problem(&ctl, &cvp, states);
      sym_real_system(states, ctl.u_opt);
#endif
      for(i=0; i<MPC_HOR_INPUTS; i++) {
              inputs[i] = ctl.u_opt[i];
      }
      return;
}

