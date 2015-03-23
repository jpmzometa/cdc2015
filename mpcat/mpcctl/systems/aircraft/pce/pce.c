#include <math.h>  /* sqrt */
#include <aircraftpcemtxops.h>

void pce_jacobian_function(real_t func_eval[], real_t jac_eval[], real_t x[]) {
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
  //real_t func_eval[n_rows]; 			// This is the first output of this function
  //real_t jac_eval[n_rows*n_cols]; 	// The jabocian in a flat vector
  for (k=1; k<Np + 1; k++) { 				// for all the prediction horizon except the first stage
	for (i = 0; i < nx; i++){			// for all the states
		index_mean = (k)*nx*(p+1) + ((i)*(p+1));
		mean = x[index_mean];	
			
		for (j = 0; j < p; j++){
			var  = var + x[index_mean + j + 1]*x[index_mean + j + 1] * PHI_VAR[j]; // the variance is equal to the sum of the squares of the coefficients, scaled by PHI_VAR
		}
		func_eval[offset] = mean + kappa_e * sqrt(var);
		for (l = 0; l < p+1; l++){
			if (l == 0){
				jac_eval[offset*n_cols + l + (offset) * (p+1)] = 1; // The derivative with respect to the first coefficient is one because it is the mean
			}
			else{
				if (offset*n_cols + l + (offset) * (p+1) == 2341){
				}
				jac_eval[offset*n_cols + l + (offset) * (p+1)] = x[index_mean + l]* PHI_VAR[l-1] * 1/sqrt(var);
			}
		}
		offset = offset + 1;
		var = 0; // initialize variance for the next iteration
	}
  }
  return;
}

void pce_jacobian_function_reduced(real_t func_eval[], real_t jac_eval[], real_t x[]) {
// The only external input which is needed is the vector of expanded states 
  
  uint32_t i,j,k,l, index_mean, p, nx, Np, offset, n_rows, n_cols, ng, offset_col;

  real_t mean, var = 0;
  real_t PHI_VAR[] = {0.333333333, 0.333333333, 0.2, 0.111111111, 0.2};
  real_t kappa_e = 4.3589; // Kappa for a probability of violation of beta = 0.05
  p = 5; // This is the number of elements of the expansion
  nx = 5; // Number of states of the original system
  Np = 5; // Prediction horizon
  offset = 0;
  offset_col = 0;
  ng = 2;  // Number of constraints
  n_rows = ng * Np; // Rows of the function evaluation and of the Jacobian. Only two constraints!
  n_cols = (p+1) * nx * Np; // Columns of the Jacobian
  //real_t func_eval[n_rows]; 			// This is the first output of this function
  //real_t jac_eval[n_rows*n_cols]; 	// The jabocian in a flat vector
  for (k=1; k<Np + 1; k++) { 				// for all the prediction horizon except the first stage
	for (i = 0; i < nx; i++){			// for all the states
		index_mean = (k)*nx*(p+1) + ((i)*(p+1));
		mean = x[index_mean];	
			
		for (j = 0; j < p; j++){
			var  = var + x[index_mean + j + 1]*x[index_mean + j + 1] * PHI_VAR[j]; // the variance is equal to the sum of the squares of the coefficients, scaled by PHI_VAR
		}
		// Add the function value and the jacobian element only if there is a constraint (states [1] and [4])
		if (i == 1 || i == 4){
			func_eval[offset] = mean + kappa_e * sqrt(var);
			for (l = 0; l < p+1; l++){
				if (l == 0){
					jac_eval[offset*n_cols + l + (offset_col) * (p+1)] = 1; // The derivative with respect to the first coefficient is one because it is the mean
				}
				else{
					jac_eval[offset*n_cols + l + (offset_col) * (p+1)] = x[index_mean + l]* PHI_VAR[l-1] * 1/sqrt(var);
				}
			}
			offset = offset + 1;
			
		}
		offset_col = offset_col +1;
		var = 0; // initialize variance for the next iteration
	}
  }
  return;
}


void pce_get_prediction(real_t x_pred[], real_t x_measured_expanded[], real_t A_sys[], real_t B_sys[], real_t u_sequence[]) {

  uint32_t i, j, Np, nx, p, nu, nx_expanded;
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
  for (i=1; i < Np+1; i++) {
	  aircraftpce_mtx_multiply_mtx_vec(out_state, A_sys, x_0, nx_expanded, nx_expanded);
	  aircraftpce_mtx_multiply_mtx_vec(out_control, B_sys, &(u_sequence[i-1]), nx_expanded, nu);
	  aircraftpce_mtx_add(x_next,out_state, out_control, nx_expanded, 1);
	  //x_0 = x_next;
	  for (j = 0; j < nx_expanded; j++){	
		  x_0[j] = x_next[j];
		  x_pred[i*nx_expanded+j] = x_next[j];
	  }
  }

  return;
}


/* Auxiliary matrix operations */

void mtx_multiply_block_diagonal(real_t pout[], const real_t pmtxA[],
		const real_t pmtxB[],
		const uint32_t rowsA,
		const uint32_t colsA,
    		const uint32_t colsB, const uint32_t Nblocks)
{
	uint32_t i; /* loop counters */

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

void sym_real_system(real_t x_k[], real_t u_k[])
{
  uint32_t n = 5;
  uint32_t m = 1;
  real_t out_state[n];
  real_t out_control[n];
  extern real_t A_nom[];
  extern real_t B_nom[];
	  aircraftpce_mtx_multiply_mtx_vec(out_state, A_nom, x_k, n, n);
	  aircraftpce_mtx_multiply_mtx_vec(out_control, B_nom, u_k, n, m);
	  aircraftpce_mtx_add(x_k, out_state, out_control, n, 1);
    return;
}
