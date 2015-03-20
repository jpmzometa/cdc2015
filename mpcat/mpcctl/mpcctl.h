#ifndef MPCCTL_H
#define MPCCTL_H

#include <stdint.h>
#ifdef CMPC
#include "mpc.h"
#endif

#ifdef CVXGEN_MPC
#define CVXGEN
#endif
#ifdef CVXGEN_QPX
#define CVXGEN
#endif

#ifdef CVXGEN
#include "mpc.h"
#include "solver.h"
#endif

#ifdef QPOASES
#include <qpOASES.hpp>
#endif

void mpcctl(void);

#endif  /* MPCCTL_H */
