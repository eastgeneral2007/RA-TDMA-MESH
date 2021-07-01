RED=\033[0;31m
BLUE=\033[;34m
NC=\033[0m
# run this make with X86 flag YES or NO

CROSS?=NO
SIMU?=NO

#get the .so name :
PROJECT:=libsbp
ifeq ($(CROSS),YES)
	PROJECT:=$(PROJECT)_arm
endif

ifeq ($(SIMU),YES)
	PROJECT:=$(PROJECT)_sim
endif

PROJECT:=$(PROJECT).so




INCLUDES:=-I$(DRK_ROOT)/include

LIB_SRC_FLDR:=${CURDIR}

LD_FLAGS=-lm -pthread -lrt 
#LD_FLAGS = -lm -lpthread -lrt -ldrk_sched

C_FLAGS= -fpic -Wall -Wextra -pedantic -std=gnu11

#define compilers and D flags
ifdef VERBOSE
	C_FLAGS+=-D VERBOSE
endif

ifdef LOGDATA
	C_FLAGS+=-D LOGDATA
endif

ifeq ($(SIMU),YES)
	C_FLAGS+=-D SIMULATOR
endif

ifeq ($(CROSS),NO)
	CC=gcc
	C_FLAGS+=-D X86
else
	# get crosscompiler path
	include $(DRK_ROOT)/toolchain.mk
endif

# select .C source code to compile:
ifeq ($(SIMU),YES)
	SRC:=read_simgps.c
else
	SRC:=read_piksi.c edc.c sbp.c 
endif




# define objects to compile
OBJ = $(SRC:.c=.o)

default: $(PROJECT) 

$(PROJECT): $(OBJ)
	mkdir -p $(DRK_ROOT)/lib
	@echo -n "$(BLUE)LINKING for$(NC)"
	@if [ "$(CROSS)" = "NO" ]; then echo -n "$(BLUE) X86 $(NC)"; else echo -n "$(RED) ARM $(NC)"  ; fi
	$(CC) $(OBJ) $(INCLUDES) -shared $(LD_FLAGS) -o $(DRK_ROOT)/lib/$@
	@echo "$(BLUE)Built $(RED)$(PROJECT)$(NC)"

%.o : %.c
	@if [ "$(CROSS)" = "NO" ]; then echo -n "$(BLUE)X86 $(NC)"; else echo -n "$(RED)ARM $(NC)"  ; fi
	$(CC) $(C_FLAGS) $(INCLUDES) -c $< -o $@

clean:
	$(RM) $(OBJ) 

realclean: clean
	$(RM) $(DRK_ROOT)/lib/$(PROJECT)

print-%  : 
	@echo $* = $($*)
	
.PHONY: clean all
