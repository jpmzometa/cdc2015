#ifndef PCE_H
#define PCE_H
#include <mpcctl.h>

enum {
    PCE_P = 5, // This is the number of elements of the expansion
    PCE_NX = 5, // Number of states of the original system
    PCE_HOR = 5, // Prediction horizon
    PCE_NCX = 2, // number of state constraints
    PCE_NXE = PCE_NX * (PCE_P+1),
    PCE_JAC_ROWS = PCE_NCX * PCE_HOR, // Rows of the function evaluation and of the Jacobian
    PCE_JAC_COLS = (PCE_P+1) * PCE_NX * PCE_HOR, // Columns of the Jacobian
};

extern void pce_jacobian_function(real_t func_eval[], real_t jac_eval[], real_t x[]);
extern void pce_jacobian_function_reduced(real_t func_eval[], real_t jac_eval[], real_t x[]);

extern void pce_get_prediction(real_t x_pred[], real_t x_measured_expanded[], const real_t A_sys[], const real_t B_sys[], real_t u_sequence[]);

extern void mtx_multiply_block_diagonal(real_t pout[], const real_t pmtxA[],
const real_t pmtxB[],
const uint32_t rowsA,
const uint32_t colsA,
const uint32_t colsB, const uint32_t Nblocks);

extern void mtx_bdiag2cols(real_t pout[], const real_t pmtx[],
const uint32_t rows,
const uint32_t cols,
const uint32_t blocks);

extern void state_orig2pce(real_t xpce[], const real_t xorig[], const uint32_t n, const uint32_t p);

extern void sym_real_system(real_t x_k[], real_t u_k[]);

#endif
