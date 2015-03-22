#!/usr/bin/python3

import os
import unittest
import shutil
from subprocess import call
from sys import path
path.append('../../../src/pc')

import numpy as np
from numpy import linalg as la
from numpy import sqrt
from numpy.testing import assert_allclose
from pudb import set_trace

from muaompc.ldt import setup_mpc_problem

install = 0

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
    from aircraftpce.mpc import setup_mpc_ctl
    from aircraftpce import aircraftpceCcvp
    from aircraftpce import aircraftpcecvp
    from aircraftpce import mpc
    m = mpc._get_mpc()
    #et_trace()
    d = m.ddg.generate_data(data)
    f = aircraftpcecvp.Former()
    f.initialize(d)
    H = np.array(f.prb.H.data).reshape((5, 5))
    m.sdg.generate_static_data(data, m.prefix)
    print(m.prefix)
    data = m.ddg.generate_data(data, m.prefix)
    import Emtx
    E = np.array(Emtx.E).reshape((25, 5))
    #et_trace()
    #tl = setup_mpc_ctl(data)
    ctl = None
    pardata = dict()
    pardata['xr'] = _vecseq2array(paramref['xr'])
    pardata['ur'] = _vecseq2array(paramref['ur'])
    pardata['x_k'] = np.matrix(paramref['x_k']).T
    from muaompc.ltidt import setup_mpc_problem
    cmpc = setup_mpc_problem('sys_aircraft_pce')
    ctl = cmpc.ctl


    #tl.configure(2)
    #tl.u_ini = np.zeros((data['N']*data['m'],))
    #tl.solve_problem(pardata)
    return ctl
def test_qpoases():
    ROWS, COLS = (0,1)
    from qpoases import PyQProblem as QProblem
    import Emtx
    E = np.array(Emtx.E).reshape((25, 5))
    H = np.array(Emtx.H).reshape((5, 5))
    from muaompc.ltidt import setup_mpc_problem
    cmpc = setup_mpc_problem('sys_aircraft_pce')
    ctl = cmpc.ctl
    zx_ub = np.array(Emtx.zx_ub).reshape((25,1))
    e_ub = np.array(Emtx.e_ub).reshape((25,1))
    zx_lb = zx_ub - 2*e_ub
    x0 = np.array(Emtx.x0).reshape((30,1))
    ctl.form_qp(x0)
    qpx = ctl.qpx
    qpx.E = E
    qpx.zx_ub = zx_ub
    qpx.zx_lb = zx_lb

    u_opt = np.zeros(ctl.u_opt.shape[ROWS])
    oas = QProblem(ctl.qpx.E.shape[COLS], ctl.qpx.E.shape[ROWS])
    oas.init(qpx.HoL, qpx.gxoL[:,0], qpx.E, qpx.u_lb[:,0], qpx.u_ub[:,0],qpx.zx_lb[:,0], qpx.zx_ub[:,0], 25)
    oas.getPrimalSolution(u_opt)
    print(u_opt)
    Linv = compute_muaompc_constants(H, E)
    oas = QProblem(ctl.qpx.E.shape[COLS], ctl.qpx.E.shape[ROWS])
    oas.init(qpx.HoL*Linv[0], qpx.gxoL[:,0]*Linv[0], qpx.E, qpx.u_lb[:,0], qpx.u_ub[:,0],qpx.zx_lb[:,0], qpx.zx_ub[:,0], 25)
    oas.getPrimalSolution(u_opt)
    print(u_opt)


def compute_muaompc_constants(H, E):
    mues = [1.]
    Linv = []
    nu = []
    for mu in mues:
        print('mu: ', mu)
        print('Hcond: ', la.cond(H))
        L = np.max(la.eigvalsh(H+mu*np.dot(E.T, E)))
        print('L: ', L)
        phi = np.min(la.eigvalsh(H))
        print('phi: ', phi)
        print('cond: ', L/phi)
        print('ratio: ', (L/phi)/la.cond(H))
        Linv.append(1./L)
        nu.append(((sqrt(L) - sqrt(phi)) / (sqrt(L) + sqrt(phi))))
    print('mu, ', mues)
    print('Linv, ', Linv)
    print('nu, ', nu)
    return Linv

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
    #ain()
    u = test_qpoases()

