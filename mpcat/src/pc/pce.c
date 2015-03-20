#include <math.h>  /* sqrt */

#include <mtx_ops.h>

extern void pce_sqrt(real_t out[], real_t mtx[], real_t vec[], uint32_t rows, uint32_t cols) {

  uint32_t i;
  mtx_multiply_mtx_vec(out, mtx, vec, rows, cols);

  for (i=0; i<rows; i++) {
    out[i] = sqrt(out[i]);
  }

  return;
}
