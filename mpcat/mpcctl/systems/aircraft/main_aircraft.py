import muaompc as mpcpy
from sys_aircraft import dt
import sys_aircraft as sys
sys.N = 10
if dt == 0.5:
    sys.mu = 1e2
    mpc = mpcpy.ltidt.setup_mpc_problem(sys)
elif dt == 0.05:
    sys.mu = 1
    mpc = mpcpy.ltidt.setup_mpc_problem(sys)

#pc.generate_c_files(numeric='float32')
mpc.generate_c_files(matlab=True)

