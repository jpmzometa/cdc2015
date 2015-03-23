#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <aircraftpcemtxops.h>
#include <pce.h>

struct aircraftpce_cvp cvp;
real_t xorig_k[PCE_NX];

void mpcpce_solve_problem(void)
{
    extern real_t states;
    extern struct mpc_ctl ctl;

    static uint32_t init = 1;
    uint32_t i;
    real_t mu = 1.2;
    real_t Linv = 0.031128858194943453;
    real_t nu = 0.79668083435185955;
    uint32_t in_iter = 5;
    uint32_t ex_iter = 15;

    //Define the output arrays and the input data
    real_t func_eval[PCE_JAC_ROWS]; 			// This is the first output of this function
    real_t jac_eval[PCE_JAC_ROWS*PCE_JAC_COLS]; 	// The jabocian in a flat vector

    real_t x_pred[PCE_NXE*(PCE_HOR+1)];
    real_t E[PCE_JAC_ROWS*PCE_HOR];
    real_t bdiag_jac[PCE_NX*PCE_JAC_COLS];
    real_t JXpred[PCE_JAC_ROWS];
    real_t JAx0[PCE_JAC_ROWS];
    real_t JXpred_JAx0[PCE_JAC_ROWS];
    real_t v_ub_x[PCE_JAC_ROWS];
    real_t zx_ub[PCE_JAC_ROWS];
    real_t zx_lb[PCE_JAC_ROWS];

real_t A_sys[] = {0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,  0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0.239000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.178000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
-0.372000000000000,	-0.0620000000000000,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
-0.0620000000000000,	-0.372000000000000,	0,	-0.0744000000000000,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	-0.372000000000000,	0,	-0.0620000000000000,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	-0.0744000000000000,	0,	-0.372000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	-0.0620000000000000,	0,	-0.372000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	-0.372000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0.270000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	-0.990000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0.139000000000000,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
-48.9000000000000,	-8.15000000000000,	0,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
-8.15000000000000,	-48.9000000000000,	0,	-9.78000000000000,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	-48.9000000000000,	0,	-8.15000000000000,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	-9.78000000000000,	0,	-48.9000000000000,	0,	0,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	-8.15000000000000,	0,	-48.9000000000000,	0,	0,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	-48.9000000000000,	0,	0,	0,	0,	0,	64.1000000000000,	0,	0,	0,	0,	0,	2.40000000000000,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};

real_t B_sys[] = { -1.23000000000000,
0,
-0.0820000000000000,
0,
0,
0,
-1.44000000000000,
0,
-0.0960000000000000,
0,
0,
0,
-4.48000000000000,
0,
-0.298666666666667,
0,
0,
0,
-1.80000000000000,
0,
-0.120000000000000,
0,
0,
0,
1.00000000000000,
0,
0.0666666666666667,
0,
0,
0};  

static real_t u_sequence[PCE_HOR];
static real_t xorig[PCE_NX];

    if (init) {
    aircraftpce_initialize_problem_structure(&cvp);
      for (i=0; i<PCE_HOR; i++) {
      u_sequence[i] = -0.2620;
      }
      for (i=0; i<PCE_NX; i++) {
      xorig[i] = 0.;
      }
      xorig[3] =  -400.;
      init=0;
    }
        state_orig2pce(cvp.prb->x_k->data, xorig, PCE_NX, PCE_P+1);
        pce_get_prediction(x_pred, cvp.prb->x_k->data, A_sys, B_sys, u_sequence);
        pce_jacobian_function_reduced(func_eval, jac_eval, x_pred);
        aircraftpce_cvp_form_problem(&cvp);

        aircraftpce_mtx_multiply_mtx_mtx(E, jac_eval, &(cvp.prb->V->data[PCE_NXE*(PCE_HOR*1)]), PCE_JAC_ROWS,
            PCE_JAC_COLS, cvp.prb->V->cols);
    /* v_ub contains -A*x0 */
        mtx_bdiag2cols(bdiag_jac, jac_eval, PCE_NCX, PCE_NXE, PCE_HOR);

        mtx_multiply_block_diagonal(JAx0, bdiag_jac, &(cvp.prb->v_ub->data[PCE_NXE]), PCE_NCX, PCE_NXE, 1, PCE_HOR);
        mtx_multiply_block_diagonal(JXpred, bdiag_jac, &(x_pred[PCE_NXE]), PCE_NCX, PCE_NXE, 1, PCE_HOR);
        aircraftpce_mtx_add(JXpred_JAx0, JXpred, JAx0, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_substract(v_ub_x, JXpred_JAx0, func_eval, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_add(zx_ub, ctl.alm->e_ub, v_ub_x, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_add(zx_lb, ctl.alm->e_lb, v_ub_x, PCE_JAC_ROWS, 1);

        for (i=0; i<25; i++) {
        ctl.qpx->HoL[i] = cvp.prb->H->data[i] * Linv;
        }
        for (i=0; i<5; i++) {
        ctl.qpx->gxoL[i] = cvp.prb->g->data[i] * Linv;
        }
        for (i=0; i<PCE_JAC_ROWS*cvp.prb->V->cols; i++) {
        ctl.qpx->E[i+PCE_NCX*PCE_HOR] = E[i];
        }
        for (i=0; i<PCE_JAC_ROWS; i++) {
        ctl.qpx->zx_ub[i+PCE_NCX] = zx_ub[i];
        ctl.qpx->zx_lb[i+PCE_NCX] = zx_lb[i];
        }
        for (i=0; i<PCE_NCX; i++) {
        ctl.qpx->zx_ub[i] = 0.;
        ctl.qpx->zx_lb[i] = 0.;
        }

        ctl.alm->mu = &mu;
        ctl.alm->Linv = &Linv;
        ctl.alm->fgm->nu = &nu;

        ctl.conf->in_iter = in_iter;
        ctl.conf->ex_iter = ex_iter;
        stc_alm_minimize_qp(ctl.alm, ctl.u_opt, ctl.l_opt);
        for (i=0; i<PCE_NX; i++) {
          xorig_k[i] = xorig[i];
        }

        sym_real_system(xorig, ctl.u_opt);

        stc_ctl_warmstart(&ctl);
        for (i=0; i<PCE_HOR; i++) {
          u_sequence[i] = ctl.u_opt[i];
        }

}
