#ifndef MPC_CONST_H
#define MPC_CONST_H

#include "mpc_base.h"

#define STATE_CONSTR  /* State constrained problem. */ 


enum { 
MPC_HOR = 5,  /**< MPC prediction horizon. */
MPC_STATES = 30,  /**< Number of system states. */
MPC_INPUTS = 1,  /**< Number of system inputs. */
MPC_MXCONSTRS = 5, /**< Number of mixed stage constraints. */
MPC_HOR_INPUTS = 5,  /**< Horizon times number of inputs. */
MPC_HOR_STATES = 150,  /**< Horizon times number of states. */
MPC_HOR_MXCONSTRS = 30  /**< Horizon times number of mixed constrained
plus the number of end state constraints. */
}; 

#endif /* MPC_CONST_H */
