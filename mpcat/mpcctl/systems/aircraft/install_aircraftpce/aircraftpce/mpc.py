import os
import pickle
from pudb import set_trace

def setup_mpc_ctl(data):
    mpc = _get_mpc()
    prefix = mpc.prefix
    __import__(prefix+'.'+prefix+'Ccvp')
    s = __import__(prefix+'.'+prefix+'fgm')
    solver = s.__getattribute__(prefix+'fgm')
    data = mpc.ddg.generate_data(data, prefix)
    ctl = solver.Solver()
    ctl.setup_solver(data)
    return ctl

def _get_mpc():
    base_dir = os.path.dirname(__file__)
    prb_path = os.path.join(base_dir, 'mpc.pickle')
    with open(prb_path, 'rb') as f:
        mpc = pickle.load(f)
    return mpc
