#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <aircraftpcemtxops.h>
#include <pce.h>
#include <pcesysmtx.h>


void mpcpce_solve_problem(struct mpc_ctl *ctl, struct aircraftpce_cvp *cvp, real_t states[])
{
    static uint32_t init = 1;
    uint32_t i;
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
	static real_t u_sequence[PCE_HOR];

    if (init) {
      for (i=0; i<PCE_HOR; i++) {
      u_sequence[i] = -0.2620;
      }

      init = 0;
    }
        state_orig2pce(cvp->prb->x_k->data, states, PCE_NX, PCE_P+1);
        pce_get_prediction(x_pred, u_sequence, cvp->prb->x_k->data, A_sys, B_sys);
        pce_jacobian_function_reduced(func_eval, jac_eval, x_pred);
        aircraftpce_cvp_form_problem(cvp);
#if 0
        aircraftpce_mtx_multiply_mtx_mtx(E, jac_eval, &(cvp->prb->V->data[PCE_NXE*(PCE_HOR*1)]), PCE_JAC_ROWS,
            PCE_JAC_COLS, cvp->prb->V->cols);
#endif
        mtx_bdiag2cols(bdiag_jac, jac_eval, PCE_NCX, PCE_NXE, PCE_HOR);
        mtx_multiply_block_diagonal(E, bdiag_jac, &(cvp->prb->V->data[PCE_NXE*(PCE_HOR*PCE_NU)]), PCE_NCX,
            PCE_NXE, PCE_NU*PCE_HOR, PCE_HOR);
    /* v_ub contains -A*x0 */
        mtx_multiply_block_diagonal(JAx0, bdiag_jac, &(cvp->prb->v_ub->data[PCE_NXE]), PCE_NCX, PCE_NXE, 1, PCE_HOR);
        mtx_multiply_block_diagonal(JXpred, bdiag_jac, &(x_pred[PCE_NXE]), PCE_NCX, PCE_NXE, 1, PCE_HOR);
        aircraftpce_mtx_add(JXpred_JAx0, JXpred, JAx0, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_substract(v_ub_x, JXpred_JAx0, func_eval, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_add(zx_ub, ctl->alm->e_ub, v_ub_x, PCE_JAC_ROWS, 1);
        aircraftpce_mtx_add(zx_lb, ctl->alm->e_lb, v_ub_x, PCE_JAC_ROWS, 1);

        for (i=0; i<25; i++) {
        ctl->qpx->HoL[i] = cvp->prb->H->data[i] * *ctl->alm->Linv;
        }
        for (i=0; i<5; i++) {
        ctl->qpx->gxoL[i] = cvp->prb->g->data[i] * *ctl->alm->Linv;
        }
        for (i=0; i<PCE_JAC_ROWS*cvp->prb->V->cols; i++) {
        ctl->qpx->E[i+PCE_NCX*PCE_HOR] = E[i];
        }
        for (i=0; i<PCE_JAC_ROWS; i++) {
        ctl->qpx->zx_ub[i+PCE_NCX] = zx_ub[i];
        ctl->qpx->zx_lb[i+PCE_NCX] = zx_lb[i];
        }
        for (i=0; i<PCE_NCX; i++) {
        ctl->qpx->zx_ub[i] = 0.;
        ctl->qpx->zx_lb[i] = 0.;
        }

        stc_ctl_warmstart(ctl);
        stc_alm_minimize_qp(ctl->alm, ctl->u_opt, ctl->l_opt);
        for (i=0; i<PCE_HOR; i++) {
          u_sequence[i] = ctl->u_opt[i];
        }
}
