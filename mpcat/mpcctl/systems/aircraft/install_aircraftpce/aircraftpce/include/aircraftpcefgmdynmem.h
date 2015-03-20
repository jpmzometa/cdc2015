#ifndef AIRCRAFTPCE_FGMDYNMEM_H
#define AIRCRAFTPCE_FGMDYNMEM_H
#include "aircraftpcefgm.h"

struct aircraftpce_fgm *aircraftpce_fgm_allocate_solver(void);

void aircraftpce_fgm_setup_solver(struct aircraftpce_fgm *fgm, struct aircraftpce_cvp_prb *prb, char *fname);

struct aircraftpce_fgm_solver_dynmem {
struct aircraftpce_fgm_solver *solver;
void (*setup_solver)(const struct aircraftpce_fgm *data);
struct aircraftpce_fgm *(*allocate_solver)(void);
};
#endif /* AIRCRAFTPCE_FGMDYNMEM_H */
