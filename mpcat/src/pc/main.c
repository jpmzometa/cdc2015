/*
 * MPC Algorithm Testing and Implementing code
 */

#include <stdio.h>
#include <sys/time.h>

#include <mpcctl.h>
#include <mpc_stc.h>
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <aircraftpcemtxops.h>
#include <pce.h>

/* current states of the simulated system */
#ifdef AIRCRAFT
struct aircraftpce_cvp cvp;
real_t states[MPC_STATES] = {0.0,0.0,0.0,-400.0,0.0}; /* initial state */
enum {SIM_POINTS = 1};  /* this should match the value in rs.py */
#endif
real_t inputs[MPC_HOR_INPUTS];
int32_t k = 0;  /* simulation mark */

struct mpc_data
{
  real_t states[MPC_STATES];
  real_t inputs[MPC_HOR_INPUTS];
  int32_t k;
} mpc_save[SIM_POINTS];

uint64_t get_time_stamp(void) {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int main(void)
{

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

real_t u_sequence[] = {-0.2620,
					   -0.2620,
					   -0.2620,
					   -0.2620,
					   -0.2620};




/* start */

    uint32_t i, j;

    extern struct mpc_ctl ctl;
    aircraftpce_initialize_problem_structure(&cvp);
    real_t mu = 1.2;
    real_t Linv = 0.031128858194943453;
    real_t nu = 0.79668083435185955;
    uint32_t in_iter = 5;
    uint32_t ex_iter = 15;
    uint32_t p, nx, Np, n_rows, n_cols, nx_expanded, ncx;

    //Define the output arrays and the input data
     p = 5; // This is the number of elements of the expansion
    nx = 5; // Number of states of the original system
    Np = 5; // Prediction horizon
    ncx = 2; // number of state constraints
    nx_expanded = nx * (p+1);
    n_rows = ncx * Np; // Rows of the function evaluation and of the Jacobian
    n_cols = (p+1) * nx * Np; // Columns of the Jacobian

    real_t func_eval[n_rows]; 			// This is the first output of this function
    real_t jac_eval[n_rows*n_cols]; 	// The jabocian in a flat vector

    real_t xorig[] = {0., 0., 0., -400., -0.};
    real_t x_k[nx];
    real_t x_pred[nx_expanded*(Np+1)];
    real_t E[n_rows*cvp.prb->V->cols];
    real_t bdiag_jac[nx*n_cols];
    real_t JXpred[n_rows];
    real_t JAx0[n_rows];
    real_t JXpred_JAx0[n_rows];
    real_t v_ub_x[n_rows];
    real_t zx_ub[n_rows];
    real_t zx_lb[n_rows];

    FILE *fp;
    fp = fopen( "xutraj.csv", "w" );

    for (i=0; i<nx; i++) {
      fprintf(fp, "x_%d, ", i);
    }
    for (i=0; i<Np*1; i++) {
      fprintf(fp, "u_%d, ", i);
    }
    fprintf(fp, "\n");
    printf("simulating: \n");

    for (k=0; k<SIM_POINTS; k++) {
        state_orig2pce(cvp.prb->x_k->data, xorig, nx, p+1);
        pce_get_prediction(x_pred, cvp.prb->x_k->data, A_sys, B_sys, u_sequence);
        pce_jacobian_function_reduced(func_eval, jac_eval, x_pred);
        aircraftpce_cvp_form_problem(&cvp);

        aircraftpce_mtx_multiply_mtx_mtx(E, jac_eval, &(cvp.prb->V->data[nx_expanded*(Np*1)]), n_rows,
            n_cols, cvp.prb->V->cols);
    /* v_ub contains -A*x0 */
        mtx_bdiag2cols(bdiag_jac, jac_eval, ncx, nx_expanded, Np);

        mtx_multiply_block_diagonal(JAx0, bdiag_jac, &(cvp.prb->v_ub->data[nx_expanded]), ncx, nx_expanded, 1, Np);
        mtx_multiply_block_diagonal(JXpred, bdiag_jac, &(x_pred[nx_expanded]), ncx, nx_expanded, 1, Np);
        aircraftpce_mtx_add(JXpred_JAx0, JXpred, JAx0, n_rows, 1);
        aircraftpce_mtx_substract(v_ub_x, JXpred_JAx0, func_eval, n_rows, 1);
        aircraftpce_mtx_add(zx_ub, ctl.alm->e_ub, v_ub_x, n_rows, 1);
        aircraftpce_mtx_add(zx_lb, ctl.alm->e_lb, v_ub_x, n_rows, 1);

        for (i=0; i<25; i++) {
        ctl.qpx->HoL[i] = cvp.prb->H->data[i] * Linv;
        }
        for (i=0; i<5; i++) {
        ctl.qpx->gxoL[i] = cvp.prb->g->data[i] * Linv;
        }
        for (i=0; i<n_rows*cvp.prb->V->cols; i++) {
        ctl.qpx->E[i+ncx*Np] = E[i];
        }
        for (i=0; i<n_rows; i++) {
        ctl.qpx->zx_ub[i+ncx] = zx_ub[i];
        ctl.qpx->zx_lb[i+ncx] = zx_lb[i];
        }
        for (i=0; i<ncx; i++) {
        ctl.qpx->zx_ub[i] = 0.;
        ctl.qpx->zx_lb[i] = 0.;
        }


        ctl.alm->mu = &mu;
        ctl.alm->Linv = &Linv;
        ctl.alm->fgm->nu = &nu;

        for (j=0; j<1; j++) {
        ctl.conf->in_iter = in_iter;
        ctl.conf->ex_iter = ex_iter;
        stc_alm_minimize_qp(ctl.alm, ctl.u_opt, ctl.l_opt);
        for (i=0; i<nx; i++) {
            fprintf(fp, "%f, ", xorig[i]);
#if 0
            printf(" %f, ", xorig[i]);
#endif
        }
        for (i=0; i<Np; i++) {
            fprintf(fp, "%f, ", ctl.u_opt[i]);
#if 1
            printf(" %f, ", ctl.u_opt[i]);
#endif
            u_sequence[i] = ctl.u_opt[i];
        }
        fprintf(fp, "\n");

        }


        sym_real_system(xorig, ctl.u_opt);

        stc_ctl_warmstart(&ctl);

    }
    fclose(fp);


    fp = fopen( "Emtx.py", "w" );
	 fprintf(fp, "V=[");
    for (i=0; i<(cvp.prb->V->rows*cvp.prb->V->cols); i++) {
         fprintf(fp, "%f, ", cvp.prb->V->data[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "J=[");
    for (i=0; i<(n_rows*n_cols); i++) {
         fprintf(fp, "%f,", jac_eval[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "f=[");
    for (i=0; i<(n_rows); i++) {
         fprintf(fp, "%f,", func_eval[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "e_ub=[");
    for (i=0; i<(n_rows); i++) {
         fprintf(fp, "%f,", ctl.alm->e_ub[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "v_ub=[");
    for (i=0; i<(nx_expanded*(Np+1)); i++) {
         fprintf(fp, "%f,", cvp.prb->v_ub->data[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "E=[");
    for (i=0; i<(n_rows*cvp.prb->V->cols); i++) {
         fprintf(fp, "%f,", ctl.qpx->E[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "zx_ub=[");
    for (i=0; i<(n_rows); i++) {
         fprintf(fp, "%f,",ctl.qpx->zx_ub[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "x0=[");
    for (i=0; i<(nx_expanded); i++) {
         fprintf(fp, "%f,",cvp.prb->x_k->data[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "H=[");
    for (i=0; i<(25); i++) {
         fprintf(fp, "%f,",cvp.prb->H->data[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "G=[");
    for (i=0; i<(5*30); i++) {
         fprintf(fp, "%f,", cvp.pmetric[AIRCRAFTPCE_G]->fac[AIRCRAFTPCE_X_K]->data[i]);
    }
	 fprintf(fp, "]\n");

	 fclose(fp);
  while (0) {

    mpcctl();
    if(k<SIM_POINTS)
    {
      for(i=0;i<MPC_STATES;i++)
        mpc_save[k].states[i] = states[i];
      for(i=0;i<MPC_HOR_INPUTS;i++)
        mpc_save[k].inputs[i] = inputs[i];
    } else {
      break;
    }
      printf("u: %f \n", mpc_save[k].inputs[0]);
      for (i=0; i<MPC_STATES;i++) {
      printf("x[%d]: %f ,", i,  mpc_save[k].states[i]);
      }
    k++;

  }

  return 0;
}
