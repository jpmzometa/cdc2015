#ifndef PCE_H
#define PCE_H
#include <mpcctl.h>

void pce_jacobian_function(real_t func_eval[], real_t jac_eval[], real_t x[]);
void pce_jacobian_function_reduced(real_t func_eval[], real_t jac_eval[], real_t x[]);

void pce_get_prediction(real_t x_pred[], real_t x_measured_expanded[], const real_t A_sys[], const real_t B_sys[], real_t u_sequence[]);
#endif
