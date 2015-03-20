cimport aircraftpceCcvp as Ccvp

cdef extern from "../include/aircraftpcefgm.h":

    cdef struct aircraftpce_fgm_conf:
        int in_iter
        int warmstart

    cdef struct aircraftpce_fgm:
        int *j_in
        double *u_ini
        double *u_opt
        int optvar_seqlen

    void aircraftpce_fgm_solve_problem(aircraftpce_fgm *fgm)

cdef extern from "../include/aircraftpcefgmdynmem.h":

    aircraftpce_fgm *aircraftpce_fgm_allocate_solver()
    void aircraftpce_fgm_setup_solver(aircraftpce_fgm *fgm, Ccvp.aircraftpce_cvp_prb *prb, char *fname)
