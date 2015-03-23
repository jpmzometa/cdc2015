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
		${SYSTEM}/pce/pce.c \
		${SYSTEM}/pce/mpcpce.c \

	SOLVERINC = ${SYSTEM}/cmpc/include \
							${SYSTEM}/pce/include \
							${SYSTEMPCE}/include
endif

# List of all the board related files.
MPCCTLSRC = ${CHIBIOS}/mpcctl/mpcctl.c \
				 ${SYSTEM}/cmpc/mpc_const.c \
				 ${SYSTEM}/pce/pcedata.c \
				 ${SYSTEM}/pce/sysmtx.c \
				 ${SYSTEMPCE}/aircraftpcecvpdata.c \
				 ${SOLVER}

# Required include directories
MPCCTLINC = ${CHIBIOS}/mpcctl/ \
		${SOLVERINC}

