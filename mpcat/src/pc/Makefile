##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#
CMPC = 1
CVXGEN_QPX = 2
CVXGEN_MPC = 3
AIRCRAFT = 11
LEGOARM = 12
GTCAR1 = 13
# Choose here your solver and system from the list above.
# They influence the behaviour of mpcctl.mk, UDEFS,...
MPCCTL_SOLVER = $(CMPC)
MPCCTL_SYSTEM = $(AIRCRAFT)
#PCCTL_SYSTEM = $(GTCAR1)

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes 
endif


##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = pc

# Imported source files and paths
CHIBIOS = ../..
include $(CHIBIOS)/mpcctl/mpcctl.mk

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(MPCCTLSRC) \
			 pce.c \
       main.c


INCDIR = $(MPCCTLINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

CC   = gcc

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

DLIBS = -lm

# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DPC_TEST

ifeq ($(MPCCTL_SYSTEM),$(AIRCRAFT))
	UDEFS += -DAIRCRAFT
endif

ifeq ($(MPCCTL_SYSTEM),$(LEGOARM))
	UDEFS += -DLEGOARM
endif

ifeq ($(MPCCTL_SYSTEM),$(GTCAR1))
	UDEFS += -DGTCAR1
endif

ifeq ($(MPCCTL_SOLVER),$(CMPC))
	UDEFS += -DCMPC
endif

ifeq ($(MPCCTL_SOLVER),$(CVXGEN_MPC))
	UDEFS += -DCVXGEN_MPC
endif

ifeq ($(MPCCTL_SOLVER),$(CVXGEN_QPX))
	UDEFS += -DCVXGEN_QPX
endif


#
# End of user defines
##############################################################################

include rules.mk
