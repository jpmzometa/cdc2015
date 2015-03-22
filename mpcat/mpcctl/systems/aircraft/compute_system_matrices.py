#!/usr/bin/python3

import numpy as np
from scipy import io
from pudb import set_trace

def main():
    A = get_A_original(0.)
    B = get_B_original(0.)
    print_mtx_c([A, B], ['A_nom', 'B_nom'])

    io.savemat('sysmtx.mat', dict(A=np.array(A).reshape((5,5)), B= np.array(B).reshape((5,1))))

def get_A_original(x1):
    A = [0.23996015,     0,         0.17871287,     0,        0,
        -0.37221757 * (1 + x1/2.0),    1,        0.27026411,     0,  0,
        -0.99008755,    0,        0.13885973,     0,        0,
        -48.93540655* (1 + x1/2.0),    64.1,    2.39923411,    1,        0,
        0,        0,            0,     0,        0]

    return A


def get_B_original(x2):

    B = [-1.2346445      * (1 + x2/5.0),
          -1.43828223     * (1 + x2/5.0),
           -4.48282454     * (1 + x2/5.0),
           -1.79989043      * (1 + x2/5.0),
            1.0        * (1 + x2/5.0)]

    return B

def print_mtx_c(mtxs, names):
    c = '#include <mpc.h>\n'
    for k, m in enumerate(mtxs):
        s = str(m)
        s = s.replace('[', '{')
        s = s.replace(']', '}')
        c += 'real_t ' + names[k] + '[] = ' + s  + ';\n'

    with open('../../../src/pc/sysmtx.c', 'w') as f:
        f.write(c)

if __name__ == '__main__':
    main()
