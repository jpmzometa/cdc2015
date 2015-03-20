import json

from cython.view cimport array as cvarray

cimport aircraftpceCcvp as Ccvp
cimport aircraftpceCfgm as Cfgm

cdef class Solver:
    cdef Cfgm.aircraftpce_fgm *fgm
    cdef Ccvp.aircraftpce_cvp *cvp
    cdef cvarray u_opt
    cdef cvarray u_ini

    def __cinit__(self):
        self.fgm = Cfgm.aircraftpce_fgm_allocate_solver()
        self.cvp = Ccvp.aircraftpce_cvp_allocate_former()

    def __dealloc__(self):
        # TODO: free memory allocated by malloc
        pass

    property u_opt:
        def __get__(self):
          return self.u_opt

    property u_ini:
        def __get__(self):
          return self.u_ini
        def __set__(self, u_ini):
          cdef double [:]u_ini_cv = u_ini
          self.u_ini[:] = u_ini_cv

    cpdef setup_solver(self, data, fname='data.json'):
        cdef int optvar_seqlen
        # TODO: check data has consistent sizes
        with open(fname, 'w') as f:
            json.dump(data, f)

        fname = fname.encode()  # python 3, char *
        Ccvp.aircraftpce_cvp_setup_former(self.cvp, fname)
        cdef Ccvp.aircraftpce_cvp_prb *prb
        prb = self.cvp.prb
        Cfgm.aircraftpce_fgm_setup_solver(self.fgm, prb, fname)
        optvar_seqlen = self.fgm.optvar_seqlen
        self.u_opt = <double[:optvar_seqlen]> self.fgm.u_opt
        self.u_ini = <double[:optvar_seqlen]> self.fgm.u_ini

    cpdef solve_problem(self, pardata):
        cdef double *u
        cdef Ccvp.aircraftpce_cvp *ccvp = self.cvp
        Ccvp._py2c_pardata(ccvp, pardata)
        Ccvp.aircraftpce_cvp_form_problem(self.cvp)
        Cfgm.aircraftpce_fgm_solve_problem(self.fgm)
        return

    cpdef configure(self, int niter):
        self.fgm.j_in[0] = niter

