
# Output directory and files
ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif
#UTFILES = $(BUILDDIR)/$(PROJECT)

OBJDIR    = $(BUILDDIR)/obj
OUTFILES = main

# Automatic compiler options
OPT = $(USE_OPT)
COPT = $(USE_COPT)
CPPOPT = $(USE_CPPOPT)
ifeq ($(USE_LINK_GC),yes)
  OPT += -ffunction-sections -fdata-sections -fno-common
endif

COBJS    = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
DEFS      = $(DDEFS) $(UDEFS)
LIBS      = $(DLIBS) $(ULIBS)
CFLAGS    = $(MCFLAGS) $(OPT) $(COPT) $(CWARN) $(DEFS)
# Paths where to search for sources
SRCPATHS  = $(sort $(dir $(CSRC)))
OBJS	  = $(COBJS)
IINCDIR   = $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR))
LLIBDIR   = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

VPATH     = $(SRCPATHS)
all: $(OBJS) $(OUTFILES) MAKE_ALL_RULE_HOOK

MAKE_ALL_RULE_HOOK:

$(OBJS): | $(BUILDDIR)

$(BUILDDIR) $(OBJDIR):
ifneq ($(USE_VERBOSE_COMPILE),yes)
	@echo Compiler Options
	@echo $(CC) $(CFLAGS) -I. $(IINCDIR) main.c
	@echo
endif
	mkdir -p $(OBJDIR)

$(COBJS) : $(OBJDIR)/%.o : %.c Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) -fPIC -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(CC) -c $(CFLAGS) -fPIC -I. $(IINCDIR) $< -o $@
endif

libmpcformqpx: $(OBJ)
	$(CC) $(CFLAGS) -shared -Wl,-soname,libmpcformqpx.so.1 -o libmpcformqpx.so $(OBJ) -lm

$(OUTFILES) : $(COBJS) libmpcformqpx
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) $(CFLAGS) -I. $(IINCDIR) -L./ -o main $(COBJS) -lmpcformqpx -lm

else
	@echo Compiling $<
	@$(CC) $(CFLAGS) -I. $(IINCDIR) -L./ -o main $(COBJS) -lmpcformqpx -lm

endif
clean:
	@echo Cleaning
	-rm -fR .dep $(BUILDDIR)
	@echo Done

