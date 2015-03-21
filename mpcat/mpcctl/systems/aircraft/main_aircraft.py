import muaompc as mpcpy
import sys_aircraft_pce as sys
if sys.dt == 0.5:
    sys.mu = 1e2
    mpc = mpcpy.ltidt.setup_mpc_problem(sys, verbose=True)
mpc.generate_c_files(matlab=True)

