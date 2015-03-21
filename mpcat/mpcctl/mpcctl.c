#ifdef PC_TEST
#include <stdio.h>
#endif
#include "mpcctl.h"  /* the auto-generated code */
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <pce.h>


void mpcctl(void)
{
        extern real_t states[];
        extern real_t inputs[];
	extern struct mpc_ctl ctl;
        extern struct aircraftpce_cvp cvp;
	static int init = 1;
	FILE *fp;

	uint32_t i; /* loop iterator */
	if (init) {
		ctl.conf->in_iter = 1;  /* iterations internal loop */
	        ctl.conf->ex_iter = 4;  /* iterations external loop */
        	ctl.conf->warmstart = 1;  /* warmstart each iteration */
		init = 0;
	}

	mpc_ctl_solve_problem(&ctl, states);  /* solve the MPC problem */

    for (i=0; i<MPC_STATES; i++) {
    cvp.prb->x_k->data[i] = states[i];
    }
    aircraftpce_cvp_form_problem(&cvp);
    fp = fopen( "mtx.py", "w" );
    mpc_predict_next_state(&ctl, states);
	 fprintf(fp, "E=[");
    for (i=0; i<(cvp.prb->V->rows*cvp.prb->V->cols); i++) {
         fprintf(fp, "%f, ", cvp.prb->V->data[i]);
    }
	 fprintf(fp, "]");
	 fclose(fp);
    for(i=0; i<MPC_HOR_INPUTS; i++) {
      inputs[i] = ctl.u_opt[i];
    }
  return;
}

