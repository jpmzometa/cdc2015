cdef extern from "../include/aircraftpcecvp.h":

    cdef enum:
        AIRCRAFTPCE_X_K,

        AIRCRAFTPCE_PAR_NUM

    cdef struct aircraftpce_term:
        int rows
        int cols
        double *data

    cdef struct aircraftpce_cvp_prb:
        aircraftpce_term *v_ub
        aircraftpce_term *g
        aircraftpce_term *v_lb
        aircraftpce_term *H
        aircraftpce_term *u_lb
        aircraftpce_term *V
        aircraftpce_term *u_ub


    cdef struct aircraftpce_cvp:
        aircraftpce_term *par[AIRCRAFTPCE_PAR_NUM]
        aircraftpce_cvp_prb *prb

    void aircraftpce_cvp_form_problem(aircraftpce_cvp *cvp)

cdef extern from "../include/aircraftpcecvpdynmem.h":

    int aircraftpce_cvp_setup_former(aircraftpce_cvp *cvp, char *fname)
    aircraftpce_cvp *aircraftpce_cvp_allocate_former()

cdef _py2c_pardata(aircraftpce_cvp *ccvp, pardata)
cdef _c2py_qp(aircraftpce_cvp_prb *cprb)

