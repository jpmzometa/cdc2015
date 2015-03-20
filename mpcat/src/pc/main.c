/*
 * MPC Algorithm Testing and Implementing code
 */

#include <stdio.h>
#include <sys/time.h>

#include <mpcctl.h>
#include "pce.h"

/* current states of the simulated system */
#ifdef AIRCRAFT
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
  int i;
  uint64_t t0, t1, dt;
  real_t mtx[] = {1., 0., 0., 1., 10., 1};
  real_t vec[] = {4., 9.};
  real_t out[3];

  pce_sqrt(out, mtx, vec, 3, 2);

  for (i=0; i<3; i++) {
    printf("out[%d]= %f; ", i, out[i]);
    printf("\n");
  }

  while (0) {

    dt = mpcctl();
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
      printf("dt: %lu \n", dt);
    k++;

  }

  return 0;
}
