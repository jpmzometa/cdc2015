#!/usr/bin/python3
import numpy as np
from copy import deepcopy
import shutil
from subprocess import call
from pudb import set_trace

import muaompc
from compute_system_matrices import get_sys, write_c_sys


def store_current_step_data(mpc, data, xk, uk, k):
    data.t[k] = mpc.sys.dt * k
    data.x[:, k] = xk
    data.u0[:, k] = uk

def main():
    steps = 60
    runs = 1
    mpc = muaompc.ltidt.setup_mpc_problem('sys_aircraft')
# configure the controller
    mpc.sim.regulate_ref(steps, np.zeros(mpc.size.states))
    data_base = deepcopy(mpc.sim.data)
    mpc.ctl.conf.in_iter = 5
    mpc.ctl.conf.ex_iter = 15
    mpc.ctl.conf.warmstart = True

    x_ini = np.zeros(mpc.size.states)
    x_ini[3] = -400
    mpc.sim.regulate_ref(steps, x_ini)
    uncert = np.random.uniform(-1., 1., (runs, 2))
    costs = np.zeros(runs)
    nviol = np.zeros(runs)
    for s, ps in enumerate(uncert):
        sys = get_sys(ps)
        xk = x_ini
        data = deepcopy(data_base)
        for k in range(steps):
            mpc.ctl.solve_problem(xk)
            uk = mpc.ctl.u_opt[:mpc.size.inputs, 0]
            store_current_step_data(mpc, data, xk, uk, k)
            xk = np.dot(sys['A'], xk) + np.dot(sys['B'], uk)
        mpc.sim.data = data
        costs[s] = comp_cost(mpc)
        nviol[s] = comp_violations(mpc)
    print("mean cost: ", np.mean(costs))
    print("mean viol: ", np.mean(nviol))
    plot_results(mpc.sim.data)
    print("last cost: ", comp_cost(mpc))
    print("last viol: ", comp_violations(mpc))

def main_C():
    steps = 60
    runs = 1
    mpc = muaompc.ltidt.setup_mpc_problem('sys_aircraft')
    mpc.sim.regulate_ref(steps, np.zeros(mpc.size.states))
    data_base = deepcopy(mpc.sim.data)

    uncert = np.random.uniform(-1., 1., (runs, 2))
    costs = np.zeros(runs)
    nviol = np.zeros(runs)
    shutil.os.chdir('../../../src/pc/')
    for s, ps in enumerate(uncert):
        write_c_sys('../../mpcctl/systems/aircraft/pce/', ps)
        call(['make'])
        call(['./main'])
        xu = np.genfromtxt('xutraj.csv', delimiter=',', skiprows=1)
        x = xu[:,0:5]
        u0 = xu[:,5:6]
        data = deepcopy(data_base)
        if (len(u0) != steps):
            print('WARNING! simulation steps in C files differ from current steps.')
            steps = len(u0)

        for k in range(steps):
            store_current_step_data(mpc, data, x[k,:], u0[k,:], k)
        mpc.sim.data = data
        costs[s] = comp_cost(mpc)
        nviol[s] = comp_violations(mpc)
    shutil.os.chdir('../../mpcctl/systems/aircraft/')

    print("mean cost: ", np.mean(costs))
    print("mean viol: ", np.mean(nviol))
    plot_results(mpc.sim.data)
    print("last cost: ", comp_cost(mpc))
    print("last viol: ", comp_violations(mpc))


def comp_cost(mpc):
    data = mpc.sim.data
    x = np.matrix(data.x)
    u0 = np.matrix(data.u0)
    stage_cost = 0
    for j in (data.t):
        st_cost = x[:,j].T * mpc.wmx.Q * x[:,j]
        in_cost = u0[:,j].T * mpc.wmx.R * u0[:,j]
        stage_cost += st_cost + in_cost

    return 0.5 * stage_cost

def comp_violations(mpc):
    tol = 1e-9
    x = mpc.sim.data.x[1,:]
    e_ub = mpc.constr.e_ub[0]
    z = 0
    #rint("x", x)
    #rint("e", e_ub)
    for j in mpc.sim.data.t:
        if (x[j]-e_ub) > tol:
            print("viol")
            z +=1
    return z

def plot_results(data):
    import matplotlib.pyplot as plt
    plt.subplot(311)
    plt.plot(data.t, data.x[3,:])
    plt.title('Altitude [m]')
    plt.ylabel(r"$x_4$")
    plt.xlabel(r"$t$")
    plt.subplot(312)
    plt.plot(data.t, data.x[1,:])
    plt.title('Constraint [m]')
    plt.ylabel(r"$x_2$")
    plt.xlabel(r"$t$")
    plt.subplot(313)
    plt.plot(data.t, data.u0[0,:], linestyle='steps-')
    plt.title('Input [rad]')
    plt.tight_layout()
    plt.show()
    plt.savefig('aircraft.eps')
    return

if __name__ == '__main__':
    main_C()
