r"""
.. default-role:: math

The system description
----------------------

The system considered is the Cessna Citation 500
airfcraft presented in ([M01]_, p.64).  A continuous-time
linear model is given by `\dot{x} = A_c x + B_c u, y = C x`, where

.. math::
   A_c = \left[ \begin{matrix}
   -1.2822 & 0 & 0.98 & 0 \\
   0 & 0 & 1 & 0 \\
   -5.4293 & 0 & -1.8366 & 0 \\
   -128.2 & 128.2 & 0 & 0 \\
   \end{matrix} \right], \;\;
   B_c = \left[ \begin{matrix}
   -0.3 \\
   0 \\
   -17 \\
   0 \\
   \end{matrix} \right],
   C = \left[ \begin{matrix}
   0 & 1 & 0 & 0 \\
   0 & 0 & 0 & 1 \\
   -128.2 & 128.2 & 0 & 0 \\
   \end{matrix} \right],

and the state vector is given by `x_1` (angle of attack),
`x_2` is the pitch angle (rad), `x_3` is the pitch angle rate (rad/s), and
`x_4` is the altitude (m). The only input `u_1` is the elevator angle (rad).
The outputs are `y_1 = x_2`,  `y_2 = x_4`, and `y_3 = -128.2 x_1 + 128.2 x_2`
is the altitude rate (m/s)

The system is subject to the following constraints:

* input constraints `-0.262 \leq u_1 \leq 0.262`,
* slew rate constraint in the input `-0.524 \leq \dot{u}_1 \leq 0.524`
* state constraints `-0.349 \leq x_2 \leq 0.349`,
* output constraints `-30.0 \leq y3 \leq 30.0`.

To consider the slew rate constraint in the input, we introduce an additional
state `x_5`. The sampling interval is `dt = 0.5 s`.

The controller parameters
-------------------------

The book proposes to use identity matrices of appropriate size for
the weighting matrices `Q` and `R`. We instead select them diagonal
with values that give a similar controller performance and much lower
condition number of the Hessian of the MPC quadratic program (see
[WAD11]_), a desirable property for any numerical algorithm.

.. [M01] Maciejowski, J. M., "Predictive Control with Constraints."

.. [WAD11]  Waschl, H. and Alberer, D. and del Re, L., "Numerically Efficient Self Tuning Strategies for MPC of Integral Gas Engines".
"""

import numpy as np

Acref = [[-1.2822, 0, 0.98, 0], [0, 0, 1, 0], [-5.4293, 0, -1.8366, 0], [-128.2, 128.2, 0, 0]]
Bcref = [[-0.3], [0], [-17], [0]]
#t = 0.5
dt = 0.5

if dt == 0.5:
    # discrete-time system
    Ad = [[  0.23996015,   0., 0.17871287,   0., 0.],
          [ -0.37221757,   1., 0.27026411,   0., 0.],
          [ -0.99008755,   0., 0.13885973,   0., 0.],
          [-48.93540655, 64.1, 2.39923411,   1., 0.],
          [0., 0., 0., 0., 0.]]
    Bd = [[-1.2346445 ],
          [-1.43828223],
          [-4.48282454],
          [-1.79989043],
          [1.]]
# Weighting matrices for a problem with a better condition number
    #Q = np.diag([55.6875, 88.234375, 465.703125, 0.52070271, 20.96875])
    #R = np.diag([20.96875])
    Q = np.diag([1.22772146e+03, 3.12342118e+03,   6.12107095e+03,   5.08874350e-01,   3.45774408e+02])
    R = np.diag([3.45774408e+02])
    #Q = np.diag([1, 1, 1, 1, 1])
    #R = np.diag([1])
    P = Q
    
elif dt == 0.05:
    Ad = [[  9.31728017e-01, 0., 4.52256496e-02, 0., 0.],
          [ -6.43699632e-03, 1., 4.76688027e-02, 0., 0.],
          [ -2.50554714e-01, 0., 9.06143221e-01, 0., 0.],
          [ -6.20934559e+00, 6.41, 6.33186262e-03, 1., 0.],
          [ 0., 0., 0., 0., 0.]]
    Bd = [[-0.03424998],
          [-0.02055917],
          [-0.80843855],
          [ 0.04548753],
          [1.]]
    Q = np.diag([1, 1, 1, 1, 1e0])
    R = np.diag([1e0])
    P = Q
    
elif dt == 0.1:
    Ad = [[  9.31728017e-01, 0., 4.52256496e-02, 0., 0.],
          [ -6.43699632e-03, 1., 4.76688027e-02, 0., 0.],
          [ -2.50554714e-01, 0., 9.06143221e-01, 0., 0.],
          [ -6.20934559e+00, 6.41, 6.33186262e-03, 1., 0.],
          [ 0., 0., 0., 0., 0.]]
    Bd = [[-0.03424998],
          [-0.02055917],
          [-0.80843855],
          [ 0.04548753],
          [1.]]
    Q = np.diag([1.39629111e+08, 2.22786136e+08, 2.50705915e+09, 3.55583591e+09, 3.36082895e+09])
    R = np.diag([3.36082895e+09])
    P = Q
# input constraints
eui = 0.262  # rad (15 degrees). Elevator angle.
u_lb = [[-eui]]
u_ub =  [[eui]]

# mixed constraints
ex2 = 0.349  # rad/s (20 degrees). Pitch angle constraint.
ex5 = 0.524 * dt  # rad/s * dt input slew rate constraint in discrete time
ey3 = 30.
# bounds
e_lb = [[-ex2], [-ey3], [-ex5]]
e_ub = [[ex2], [ey3], [ex5]]
# constraint matrices
Kx = [[0, 1, 0, 0, 0],
      [-128.2, 128.2, 0, 0, 0],
      [0., 0., 0., 0., -1.]]
Ku = [[0],
      [0],
      [1]]

# terminal state constraints
f_lb = e_lb
f_ub = e_ub
F = Kx
phi2 = [[1, 2, 3, 4, 5], [6., 7, 8, 9, 0], [1, 2, 3, 4, 5]];
phi2f = [[6, 7, 8, 9, 10], [11, 12, 13, 14, 15], [16.,17,18,19,20]];
(n, m) = np.array(Bd).shape
(r, n) = np.array(Kx).shape
data = dict(A=np.array(Ad), B=np.array(Bd), P=np.array(P), Q=np.array(Q), R=np.array(R), n=n, m=m, r=r,
u_lb=np.array(u_lb), u_ub=np.array(u_ub),
e_lb=np.array(e_lb), e_ub=np.array(e_ub),
f_lb=np.array(f_lb), f_ub=np.array(f_ub), F=np.array(F),
Kx=np.array(Kx), Ku=np.array(Ku),
phi2=np.array(phi2), phi2f=np.array(phi2f),
zero=np.zeros((n,1)))
