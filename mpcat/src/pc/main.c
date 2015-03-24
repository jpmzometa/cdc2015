/*
 * MPC Algorithm Testing and Implementing code
 */

#include <stdio.h>
#include <sys/time.h>

#include <mpcctl.h>
#include <mpc_stc.h>
#ifdef CMPC
#include <aircraftpcecvp.h>
#include <aircraftpcecvpdata.h>
#include <pce.h>
#endif

/* current states of the simulated system */
#ifdef AIRCRAFT
struct aircraftpce_cvp cvp;
extern struct mpc_ctl ctl;
real_t states[MPC_STATES] = {0.0,0.0,0.0,-400.0,0.0}; /* initial state */
enum {SIM_POINTS = 60};  
#endif
#ifdef AIRCRAFTNOM
extern struct mpc_ctl ctl;
real_t states[MPC_STATES] = {0.0,0.0,0.0,-400.0,0.0}; /* initial state */
enum {SIM_POINTS = 60}; 
#endif
real_t inputs[MPC_HOR_INPUTS];
int32_t k = 0;  /* simulation mark */

uint64_t get_time_stamp(void) {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int main(void)
{
  uint32_t i;
  uint64_t t1, t2, dt;

    FILE *fp;
    fp = fopen( "xutraj.csv", "w" );
#ifdef CMPC
    uint32_t _nx_ = PCE_NX;
    uint32_t _hnu_ = PCE_HOR*1;
#endif
#ifdef CMPCNOM
    uint32_t _nx_ = MPC_STATES;
    uint32_t _hnu_ = MPC_HOR_INPUTS;
#endif
    for (i=0; i<_nx_; i++) {
      fprintf(fp, "x_%d, ", i);
    }
    for (i=0; i<_hnu_*1; i++) {
      fprintf(fp, "u_%d, ", i);
    }
    fprintf(fp, "\n");
    printf("simulating: \n");

    for (k=0; k<SIM_POINTS; k++) {
        for (i=0; i<_nx_; i++) {
            fprintf(fp, "%f, ", states[i]);
#if 0
            printf(" %f, ", xorig_k[i]);
#endif
        }
        for (i=0; i<_hnu_; i++) {
            fprintf(fp, "%f, ", ctl.u_opt[i]);
        }

	t1 = get_time_stamp();
        mpcctl();
	t2 = get_time_stamp();
	dt = t2 - t1;
        fprintf(fp, "%f ", (double)dt);
        fprintf(fp, "\n");
    }

    fclose(fp);

#ifdef CMPC
    fp = fopen( "Emtx.py", "w" );
	 fprintf(fp, "V=[");
    for (i=0; i<(cvp.prb->V->rows*cvp.prb->V->cols); i++) {
         fprintf(fp, "%f, ", cvp.prb->V->data[i]);
    }
	 fprintf(fp, "]\n");
#if 0
	 fprintf(fp, "J=[");
    for (i=0; i<(PCE_JAC_ROWS*PCE_JAC_COLS); i++) {
         fprintf(fp, "%f,", jac_eval[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "f=[");
    for (i=0; i<(PCE_JAC_ROWS); i++) {
         fprintf(fp, "%f,", func_eval[i]);
    }
	 fprintf(fp, "]\n");
#endif
	 fprintf(fp, "e_ub=[");
    for (i=0; i<(PCE_JAC_ROWS); i++) {
         fprintf(fp, "%f,", ctl.alm->e_ub[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "v_ub=[");
    for (i=0; i<(PCE_NXE*(PCE_HOR+1)); i++) {
         fprintf(fp, "%f,", cvp.prb->v_ub->data[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "E=[");
    for (i=0; i<(PCE_JAC_ROWS*cvp.prb->V->cols); i++) {
         fprintf(fp, "%f,", ctl.qpx->E[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "zx_ub=[");
    for (i=0; i<(PCE_JAC_ROWS); i++) {
         fprintf(fp, "%f,",ctl.qpx->zx_ub[i]);
    }
	 fprintf(fp, "]\n");
	 fprintf(fp, "x0=[");
    for (i=0; i<(PCE_NXE); i++) {
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
#endif

  return 0;
}
