#include <math.h>  /* sqrt */
#include <stdio.h>
#include <mtx_ops.h>
#include <mpc.h>


extern void pce_jacobian_function(real_t func_eval[], real_t jac_eval[], real_t x[]) {
// The only external input which is needed is the vector of expanded states 
  
  uint32_t i,j,k,l, index_mean, p, nx, Np, offset, n_rows, n_cols;

  real_t mean, var = 0;
  real_t PHI_VAR[] = {0.333333333, 0.333333333, 0.2, 0.111111111, 0.2};
  real_t kappa_e = 4.3589; // Kappa for a probability of violation of beta = 0.05
  p = 5; // This is the number of elements of the expansion
  nx = 5; // Number of states of the original system
  Np = 5; // Prediction horizon
  offset = 0;
  
  n_rows = nx * Np; // Rows of the function evaluation and of the Jacobian
  n_cols = (p+1) * nx * Np; // Columns of the Jacobian
  printf("iter %d, %d;  \n", n_rows, p);  
  //real_t func_eval[n_rows]; 			// This is the first output of this function
  //real_t jac_eval[n_rows*n_cols]; 	// The jabocian in a flat vector
  for (k=1; k<Np + 1; k++) { 				// for all the prediction horizon except the first stage
	for (i = 0; i < nx; i++){			// for all the states
		printf("iter %d, %d; \n", k, i);
		index_mean = (k)*nx*(p+1) + ((i)*(p+1));
		printf("iter %f, \n", x[index_mean]);
		mean = x[index_mean];	
			
		for (j = 0; j < p; j++){
			var  = var + x[index_mean + j + 1]*x[index_mean + j + 1] * PHI_VAR[j]; // the variance is equal to the sum of the squares of the coefficients, scaled by PHI_VAR
		}
		func_eval[offset] = mean + kappa_e * sqrt(var);
		printf("mean %f, \n", mean);
		printf("var %f, \n", var);
		for (l = 0; l < p+1; l++){
			if (l == 0){
				jac_eval[offset*n_cols + l + (offset) * (p+1)] = 1; // The derivative with respect to the first coefficient is one because it is the mean
			}
			else{
				jac_eval[offset*n_cols + l + (offset) * (p+1)] = x[index_mean + l]* PHI_VAR[l-1] * 1/sqrt(var);
			}
		}
		offset = offset + 1;
		var = 0; // initialize variance for the next iteration
	}
  }

  return;
}

extern void pce_get_prediction(real_t x_pred[], real_t x_measured_expanded[], const real_t A_sys[], const real_t B_sys[], real_t u_sequence[]) {

  uint32_t i, j, Np, nx, p, nu, nx_expanded;

  extern struct mpc_ctl ctl;
  p = 5; // This is the number of elements of the expansion
  nx = 5; // Number of states of the original system
  nu = 1;
  Np = 5; // Prediction horizon
  nx_expanded = nx * (p+1);
  real_t x_next[nx_expanded];
  real_t x_0[nx_expanded];
  
  real_t out_state[nx_expanded];
  real_t out_control[nx_expanded];
  for (j = 0; j < nx_expanded; j++){	
	  x_0[j] = x_measured_expanded[j];
  }
  // Adapt the previous control sequence (discard first element and last element is maintained constant)
  for (i = 0; i < Np-1; i++){
	  u_sequence[i] = u_sequence[i+1];
  }
  u_sequence[Np-1] = u_sequence[Np-2];

  // The first element of x_pred is just the initial condition
  for (i = 0; i < nx_expanded; i++){
	  x_pred[i] = x_measured_expanded[i];
  }	
  // Simulate the system to get the rest of the prediction
  for (i = 0; i < Np; i++){
	  ctl.u_opt[i] = u_sequence[i];
  }
  for (i=1; i < Np+1; i++) {
	  printf("control %f, \n", u_sequence[i-1]);  
	  mtx_multiply_mtx_vec(out_state, A_sys, x_0, nx_expanded, nx_expanded);
	  mtx_multiply_mtx_vec(out_control, B_sys, &(u_sequence[i-1]), nx_expanded, nu);
	  mtx_add(x_next,out_state, out_control, nx_expanded, 1);
	  //x_0 = x_next;
	  for (j = 0; j < nx_expanded; j++){	
		  x_0[j] = x_next[j];
		  x_pred[i*nx_expanded+j] = x_next[j];
	  }
#if 0
    mpc_predict_next_state(&ctl, x_0);
	  for (j = 0; j < nx_expanded; j++){	
		  x_pred[i*nx_expanded+j] = x_0[j];
	  }
#endif
  }

  return;
}
