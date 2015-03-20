#!/usr/bin/python3

import os
import unittest
import shutil
from subprocess import call

import numpy as np
from numpy.testing import assert_allclose
from pudb import set_trace

from muaompc.ldt import setup_mpc_problem

install = 1

def _install():
    prefix='aircraftpce'
    setup_mpc_problem('regorigmpc.prb', prefix)
    shutil.os.chdir('./install_%s/' % prefix)
    call(['python3', prefix+'setup.py', 'install', '--user'])
    shutil.os.chdir('..')

def main():

    paramref = dict(x_k=np.array([0, 0, 0, -400., 0]),
    xr=[np.array([5, 7]), np.array([5, 7]), np.array([11, 13])],
    ur=[np.array([17]), np.array([19])])

    from sys_aircraft_pce import data
    data['N'] = 10
    data['phik'] = data['phi2']
    data['M'] = np.eye(data['n'])
    from aircraftpce.mpc import setup_mpc_ctl
    from aircraftpce import aircraftpceCcvp
    from aircraftpce import aircraftpcecvp
    from aircraftpce import mpc
    m = mpc._get_mpc()
    #et_trace()
    d = m.ddg.generate_data(data)
    f = aircraftpcecvp.Former()
    f.initialize(d)
    print(np.array(f.prb.H.data))
    m.sdg.generate_static_data(data, m.prefix)
    print(m.prefix)
    data = m.ddg.generate_data(data, m.prefix)
    
    #et_trace()
    #tl = setup_mpc_ctl(data)
    ctl = None
    pardata = dict()
    pardata['xr'] = _vecseq2array(paramref['xr'])
    pardata['ur'] = _vecseq2array(paramref['ur'])
    pardata['x_k'] = np.matrix(paramref['x_k']).T
    #tl.configure(2)
    #tl.u_ini = np.zeros((data['N']*data['m'],))
    #tl.solve_problem(pardata)
    return ctl

def _vecseq2array(vecseq):
    outer_rows = len(vecseq)
    n = vecseq[0].shape[0]
    arr = np.zeros((n*outer_rows, 1))

    for i, mtx in enumerate(vecseq):
            arr[n*i:n*(i+1), 0] = mtx

    return arr


if __name__ == '__main__':
    if install:
        _install()
    main()
