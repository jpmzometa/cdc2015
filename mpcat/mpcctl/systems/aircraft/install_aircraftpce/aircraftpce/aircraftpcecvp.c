#include "aircraftpcemtxops.h"
#include "aircraftpcecvp.h"


static void aircraftpce_copy_data(struct aircraftpce_term *dest, struct aircraftpce_term *src);

void aircraftpce_cvp_form_problem(struct aircraftpce_cvp *cvp)  {
  int i, j;
  struct aircraftpce_pmetric *pm;

    for (i=0; i<AIRCRAFTPCE_PMETRIC_NUM; i++) {
        pm = cvp->pmetric[i];
        aircraftpce_copy_data(pm->val, pm->fac0);
        for (j=0; j<pm->fac_num[0]; j++) {
                aircraftpce_mtx_mul_add(pm->val->data, pm->aux->data,
                pm->fac[j]->data, pm->par[j]->data,
                pm->fac[j]->rows, pm->fac[j]->cols);
            }
        }
    return;
}

void aircraftpce_copy_data(struct aircraftpce_term *dest, struct aircraftpce_term *src)  {
    int j;
        for (j=0; j<(dest->cols*dest->rows); j++) {
            dest->data[j] = src->data[j];
      }
    return;
}

