#include <sys/time.h>
#include <stddef.h>  /* NULL */

#include "mpcctl.h"  /* the auto-generated code */
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>

/* This file is a test of the C routines of the ALM+FGM MPC
 * algorithm. The same routines can be used in a real systems.
 */

uint64_t get_time_stamp_(void) {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

#ifdef CVXGEN
Vars vars;
Params params;
Workspace work;
Settings settings;
#endif
#ifdef CVXGEN_MPC
void copy_mpc2cvx(struct mpc_ctl *ctl, real_t x_0[]);
#endif
#ifdef CVXGEN_QPX
void copy_qpx2cvx(struct mpc_qpx *qpx);
#endif

uint64_t mpcctl(void)
{
  uint64_t t0, t1, dt;
        extern real_t states[];
        extern real_t inputs[];
	extern struct mpc_ctl ctl;
  struct aircraftpce_cvp cvp;
	static int init = 1;

	int i; /* loop iterator */
#ifdef CMPC
	if (init) {
		ctl.conf->in_iter = 1;  /* iterations internal loop */
	        ctl.conf->ex_iter = 4;  /* iterations external loop */
        	ctl.conf->warmstart = 1;  /* warmstart each iteration */

          aircraftpce_initialize_problem_structure(&cvp);
		init = 0;
	}

	mpc_ctl_solve_problem(&ctl, states);  /* solve the MPC problem */
#endif

#ifdef CVXGEN
	if (init) {
  		set_defaults();
  		setup_indexing();
#ifdef AIRCRAFT
      settings.eps = 1e-9;
		settings.kkt_reg = 1e-13;
  settings.refine_steps = 1;
  		settings.max_iters = 7;
#endif
#ifdef LEGOARM
  		settings.max_iters = 4;
#endif
#ifdef GTCAR1
  		settings.max_iters = 9;
#endif
  		settings.verbose = 1;
		init = 0;
	} 

    mpc_ctl_form_qp(&ctl, states);
#ifdef CVXGEN_MPC
copy_mpc2cvx(&ctl, states);
#endif
#ifdef CVXGEN_QPX
copy_qpx2cvx(ctl.qpx);
#endif
    t0 = get_time_stamp_();
    for(i=0; i<1000; i++) {
      ;
    }
    t1 = get_time_stamp_();
    dt = t1-t0;

    solve();
#ifdef CVXGEN_MPC
      ctl.u_opt = *vars.u;
#endif
#ifdef CVXGEN_QPX
      ctl.u_opt = vars.u;
#endif
#endif
    for (i=0; i<MPC_STATES; i++) {
    cvp.prb->x_k->data[i] = states[i];
    }
    aircraftpce_cvp_form_problem(&cvp);
    mpc_predict_next_state(&ctl, states);
    for(i=0; i<MPC_HOR_INPUTS; i++)
      inputs[i] = ctl.u_opt[i];
  return dt;
}

#ifdef CVXGEN
void mtx2cvx(real_t cvx[], const real_t qpx[],
    const int rows, const int cols) {
  int i, j;
  for (i = 0; i < rows; i++) {
    for (j = 0; j < cols; j++) {
      cvx[i + j * rows] = qpx[i * cols + j];
    }
  }

  return;
}
#endif

#ifdef CVXGEN_MPC
void copy_mpc2cvx(struct mpc_ctl *ctl, real_t x_0[]) {
  real_t Kx[15] = {0, 1, 0, 0, 0, -128.2, 128.2, 0, 0, 0, 0., 0., 0., 0., -1.};
  real_t Ku[3] = {0, 0, 1};
  real_t e_ub[3] = {0.349, 30., 0.262};
  real_t e_lb[3] = {-0.349, -30., -0.262};
  mtx2cvx(params.x_0, x_0, MPC_STATES, 1);
  mtx2cvx(params.Kx, Kx, MPC_MXCONSTRS, MPC_STATES);
  mtx2cvx(params.Ku, Ku, MPC_MXCONSTRS, MPC_INPUTS);
  mtx2cvx(params.A, ctl->sys->Ad, MPC_STATES, MPC_STATES);
  mtx2cvx(params.B, ctl->sys->Bd, MPC_STATES, MPC_INPUTS);
  mtx2cvx(params.Q, ctl->wmx->Q, MPC_STATES, MPC_STATES);
  mtx2cvx(params.P, ctl->wmx->P, MPC_STATES, MPC_STATES);
  mtx2cvx(params.R, ctl->wmx->R, MPC_INPUTS, MPC_INPUTS);
  mtx2cvx(params.u_lb, ctl->qpx->u_lb, MPC_INPUTS, 1);
  mtx2cvx(params.u_ub, ctl->qpx->u_ub, MPC_INPUTS, 1);
  mtx2cvx(params.e_lb, e_lb, MPC_MXCONSTRS, 1);
  mtx2cvx(params.e_ub, e_ub, MPC_MXCONSTRS, 1);
  mtx2cvx(params.f_lb, e_lb, MPC_MXCONSTRS, 1);
  mtx2cvx(params.f_ub, e_ub, MPC_MXCONSTRS, 1);
}

#endif

#ifdef CVXGEN_QPX
void copy_qpx2cvx(struct mpc_qpx *qpx) {
  mtx2cvx(params.H, qpx->HoL, MPC_HOR_INPUTS, MPC_HOR_INPUTS);
  mtx2cvx(params.g, qpx->gxoL, MPC_HOR_INPUTS, 1);
  mtx2cvx(params.u_lb, qpx->u_lb, MPC_HOR_INPUTS, 1);
  mtx2cvx(params.u_ub, qpx->u_ub, MPC_HOR_INPUTS, 1);
  mtx2cvx(params.v_lb, qpx->zx_lb, MPC_HOR_MXCONSTRS, 1);
  mtx2cvx(params.v_ub, qpx->zx_ub, MPC_HOR_MXCONSTRS, 1);
  mtx2cvx(params.V, qpx->E, MPC_HOR_MXCONSTRS, MPC_HOR_INPUTS);
}
#endif

