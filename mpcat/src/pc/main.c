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
enum {SIM_POINTS = 2};  /* this should match the value in rs.py */
#endif
void mtx_multiply_block_diagonal(real_t pout[], const real_t pmtxA[],
		const real_t pmtxB[],
		const uint32_t rowsA,
		const uint32_t colsA,
    		const uint32_t colsB, const uint32_t Nblocks)
{
	uint32_t i,j; /* loop counters */

	for (i = 0; i < Nblocks; i++) {
      aircraftpce_mtx_multiply_mtx_mtx(
          &(pout[(colsB*rowsA*i)]),
          &(pmtxA[(colsA*rowsA*i)]),
          &(pmtxB[(colsB*colsA*i)]),
          rowsA, colsA, colsB);
  }
	return;
}

void mtx_bdiag2cols(real_t pout[], const real_t pmtx[],
		const uint32_t rows,
		const uint32_t cols,
    const uint32_t blocks)
{
	uint32_t i,j,k; /* loop counters */

	for (i = 0; i < blocks; i++) {
    for (j=0; j<rows; j++) {
      for (k=0; k<cols; k++) {
          pout[cols*rows*i + cols*j + k] =
          pmtx[blocks*cols*rows*i + cols*i + blocks*cols*j + k];
      }
  }
  }
	return;
}

void state_orig2pce(real_t xpce[], const real_t xorig[], const uint32_t n, const uint32_t p)
{
	uint32_t i,j; /* loop counters */
	for (i = 0; i < n; i++) {

    for (j=0; j<p; j++) {
      xpce[i*n+j] = 0.;
    }
    xpce[i*p] = xorig[i];
  }
}

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
#if 0
  extern real_t x_measured_expanded[];
  extern real_t x[];
  extern real_t u_sequence[];
#endif
  real_t x[] = {0.131676852301864,
0,
0,
0,
0,
0,
0.342162647496714,
0,
0,
0,
0,
0,
0.328054300903874,
0,
0,
0,
0,
0,
-382.970735187983,
0,
0,
0,
0,
0,
-0.110467615734654,
0,
0,
0,
0,
0,
0.208659597518526,
0,
0.00791967761716606,
0,
0,
0,
0.520830785156651,
-0.00816396484271556,
0.00927181769814564,
0,
0,
0,
0.347924289960257,
0,
0.0288456550608975,
0,
0,
0,
-366.515930656995,
-1.07316634626019,
0.0115897721226820,
0,
0,
0,
-0.0965814343556837,
0,
-0.00643876229037891,
0,
0,
0,
0.0850928411841191,
0,
0.00524684113562684,
0,
0,
0,
0.505881859527193,
-0.0211008598888642,
0.0120295502482132,
0,
-0.000491020012264296,
0,
-0.255486989902352,
0,
-0.0103159657650955,
0,
0,
0,
-342.538197243257,
-3.29705221245424,
0.285265030437065,
0,
-0.0645453725799034,
0,
0.0217132733623855,
0,
0.00144755155749237,
0,
0,
0,
0.0262256704940051,
0,
0.00284209750213578,
0,
0,
0,
0.465380663415352,
-0.0263766160422796,
0.0113014031280033,
0,
-0.000816324162673160,
0,
0.0673315274428519,
0,
0.00584411682181828,
0,
0,
0,
-314.810210222130,
-5.34312398698101,
0.780041587652672,
0,
-0.138781510621403,
0,
-0.0417602972793653,
0,
-0.00278401981862436,
0,
0,
0,
-0.00241161731025181,
0,
0.000341876467750994,
0,
0,
0,
0.449611565589663,
-0.0280026076129079,
0.0102092103450005,
0,
-0.000992534207805578,
0,
-0.0918703873324738,
0,
-0.00701908134607935,
0,
0,
0,
-286.130390144514,
-7.24760428981727,
1.37749278560798,
0,
-0.214270984091160,
0,
0.0168004588968672,
0,
0.00112003059312448,
0,
0,
0,
-0.00946697006274020,
0,
-0.000670198975836950,
0,
0,
0,
0.434440075335659,
-0.0278530873396723,
0.00876930651464565,
0,
-0.00101373054880614,
0,
0.0167974056554676,
0,
0.000497882546990299,
0,
0,
0,
-257.401929142486,
-9.02291675672611,
1.99906764694276,
0,
-0.280678720023668,
0,
-0.00606693936552058,
0,
-0.000404462624368039,
0,
0,
0};
  

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
#if 0
    real_t mu = 1.000;
    real_t Linv = 0.032117260179852634;
    real_t nu = 0.79380834980270076;
#endif
#if 1
    real_t mu = 1e0;
    real_t Linv = 0.00092363382016503125;
    real_t nu = 0.96175968103476439;
    uint32_t in_iter = 60;
    uint32_t ex_iter = 7;
#endif
    uint32_t p, nx, Np, n_rows, n_cols, nx_expanded;

    //Define the output arrays and the input data
     p = 5; // This is the number of elements of the expansion
    nx = 5; // Number of states of the original system
    Np = 5; // Prediction horizon
    nx_expanded = nx * (p+1);
    n_rows = nx * Np; // Rows of the function evaluation and of the Jacobian
    n_cols = (p+1) * nx * Np; // Columns of the Jacobian

    real_t func_eval[n_rows]; 			// This is the first output of this function
    real_t jac_eval[n_rows*n_cols]; 	// The jabocian in a flat vector

    real_t xorig[] = {0., 0., 0., -400., -0.};
    real_t x_pred[nx_expanded*(Np+1)];
    real_t E[n_rows*cvp.prb->V->cols];
    real_t bdiag_jac[nx*n_cols];
    real_t JXpred[n_rows];
    real_t JAx0[n_rows];
    real_t JXpred_JAx0[n_rows];
    real_t v_ub_x[n_rows];
    real_t zx_ub[n_rows];
    real_t zx_lb[n_rows];

    state_orig2pce(cvp.prb->x_k->data, xorig, nx, p+1);
    pce_get_prediction(x_pred, cvp.prb->x_k->data, A_sys, B_sys, u_sequence);
    pce_jacobian_function(func_eval, jac_eval, x_pred);
    aircraftpce_cvp_form_problem(&cvp);

    aircraftpce_mtx_multiply_mtx_mtx(E, jac_eval, &(cvp.prb->V->data[nx_expanded*(Np*1)]), n_rows,
        n_cols, cvp.prb->V->cols);
/* v_ub contains -A*x0 */
    mtx_bdiag2cols(bdiag_jac, jac_eval, nx, nx_expanded, Np);

    mtx_multiply_block_diagonal(JAx0, bdiag_jac, &(cvp.prb->v_ub->data[nx_expanded]), nx, nx_expanded, 1, Np);
    mtx_multiply_block_diagonal(JXpred, bdiag_jac, &(x_pred[nx_expanded]), nx, nx_expanded, 1, Np);
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
    ctl.qpx->E[i+25] = E[i];
    }
    for (i=0; i<n_rows; i++) {
    ctl.qpx->zx_ub[i+5] = zx_ub[i];
    ctl.qpx->zx_lb[i+5] = zx_lb[i];
    }
    for (i=0; i<5; i++) {
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
      printf("u_opt (iter:%d x %d ): ", ctl.conf->in_iter, ctl.conf->ex_iter);
      for (i=0; i<5; i++) {
      printf(" %f, ", ctl.u_opt[i]);
      }
      printf("\n");
    }
    stc_ctl_warmstart(&ctl);


	FILE *fp;
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
	 fprintf(fp, "x_pred=[");
    for (i=0; i<(nx_expanded*(Np+1)); i++) {
         fprintf(fp, "%f,", x[i]);
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
