#ifndef PCE_H
#define PCE_H

void pce_sqrt(real_t out[], real_t mtx[], real_t vec[], uint32_t rows, uint32_t cols);

void pce_jacobian_function(real_t func_eval[], real_t jac_eval[], real_t x[]);

void pce_get_prediction(real_t x_pred[], real_t x_measured_expanded[], real_t A_sys[], real_t B_sys[], real_t u_sequence[]);
#endif
