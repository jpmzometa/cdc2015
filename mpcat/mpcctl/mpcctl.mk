 #the macros are defined in the main Makefile

ifeq ($(MPCCTL_SYSTEM), $(AIRCRAFT))
	SYSTEM = ${CHIBIOS}/mpcctl/systems/aircraft
	SYSTEMPCE = ${SYSTEM}/install_aircraftpce/aircraftpce
endif

ifeq ($(MPCCTL_SOLVER), $(CMPC))
	SOLVER = ${SYSTEM}/cmpc/mpc_inc.c \
		${SYSTEM}/cmpc/mpc_stc.c \
		${SYSTEM}/cmpc/mpc_ref.c \
		${SYSTEM}/cmpc/mpc.c \
		${SYSTEM}/cmpc/mtx_ops.c \
		${SYSTEMPCE}/aircraftpcecvp.c \
		${SYSTEMPCE}/aircraftpcemtxops.c \

	SOLVERINC = ${SYSTEM}/cmpc/include \
							${SYSTEMPCE}/include
endif

ifeq ($(MPCCTL_SOLVER), $(CVXGEN_MPC))
	SOLVER = ${SYSTEM}/cvxgen_mpc/ldl.c \
		${SYSTEM}/cvxgen_mpc/matrix_support.c \
		${SYSTEM}/cvxgen_mpc/util.c \
		${SYSTEM}/cvxgen_mpc/solver.c \
		${SYSTEM}/cmpc/mpc_inc.c \
		${SYSTEM}/cmpc/mpc_stc.c \
		${SYSTEM}/cmpc/mpc_ref.c \
		${SYSTEM}/cmpc/mpc.c \
		${SYSTEM}/cmpc/mtx_ops.c

	SOLVERINC = ${SYSTEM}/cmpc/include \
		 ${SYSTEM}/cvxgen_mpc
endif

ifeq ($(MPCCTL_SOLVER), $(CVXGEN_QPX))
	SOLVER = ${SYSTEM}/cvxgen_qpx/ldl.c \
		${SYSTEM}/cvxgen_qpx/matrix_support.c \
		${SYSTEM}/cvxgen_qpx/util.c \
		${SYSTEM}/cvxgen_qpx/solver.c \
		${SYSTEM}/cmpc/mpc_inc.c \
		${SYSTEM}/cmpc/mpc_stc.c \
		${SYSTEM}/cmpc/mpc_ref.c \
		${SYSTEM}/cmpc/mpc.c \
		${SYSTEM}/cmpc/mtx_ops.c

	SOLVERINC = ${SYSTEM}/cmpc/include \
		 ${SYSTEM}/cvxgen_qpx
endif

# List of all the board related files.
MPCCTLSRC = ${CHIBIOS}/mpcctl/mpcctl.c \
				 ${SYSTEM}/cmpc/mpc_const.c \
				 ${SYSTEMPCE}/aircraftpcecvpdata.c \
				 ${SOLVER}

# Required include directories
MPCCTLINC = ${CHIBIOS}/mpcctl/ \
		${SOLVERINC}

