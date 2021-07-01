# editor: Luis Pinto
# colors
RED=\033[0;31m
GRN=\033[;32m
BLUE=\033[;34m
NC=\033[0m

LIBPCAP?=NO

# name of libraries drk, sbp and sim depend on the current target
s_uav:=UAV
s_bs:=BS
s_sim:=sim
s_cc:=arm



#obj directory - only change depending on the target
ifeq ($(CROSS),NO)
		OBJDIR=./obj_x86
else
		OBJDIR=./obj_arm
endif



LD_FLAGS=-lm -pthread -lrt 
#LD_FLAGS = -lm -lpthread -lrt -ldrk_sched

C_FLAGS= -O3 -fpic -Wall -Wextra -pedantic -std=gnu11
#-O1

#Verbose makes all printfs output line ,funcitoname and module
ifeq ($(VERBOSE),YES)
	C_FLAGS+=-D VERBOSE
endif

# log functions are enable. saving data into files.
ifeq ($(LOGDATA),YES)
	C_FLAGS+=-D LOGDATA
endif

# simulator (is x86), normal x86, or drone (non-x86,ie,arm).
ifeq ($(SIMU),YES)
	CC=gcc
	C_FLAGS+= -D SIMULATOR
endif




PROJECT_BIN:=$(PROJECT)
ifeq ($(CROSS),YES)
		PROJECT_BIN:=$(PROJECT_BIN)_$(s_cc)
endif

ifeq ($(BS),YES)
		PROJECT_BIN:=$(PROJECT_BIN)_$(s_bs)
else
		PROJECT_BIN:=$(PROJECT_BIN)_$(s_uav)
endif

ifeq ($(SIMU),YES) 
	PROJECT_BIN:=$(PROJECT_BIN)_$(s_sim)
endif



# where is the compiler
ifeq ($(CROSS),NO)
	CC:=gcc
	AR:=ar
	RPATH:='$(DRK_ROOT)/lib/' #runtime path
	C_FLAGS+= -D X86 
else
	include $(DRK_ROOT)/toolchain.mk # cc and ar defined here
	RPATH:='/usr/lib/'  #runtime path
	C_FLAGS+= -mtune=cortex-a8 -march=armv7-a
endif


DRKNAME:=drk
ifeq ($(CROSS),YES)
		DRKNAME:=$(DRKNAME)_$(s_cc)
endif

ifeq ($(BS),YES)
		DRKNAME:=$(DRKNAME)_$(s_bs)
else
		DRKNAME:=$(DRKNAME)_$(s_uav)
endif

ifeq ($(SIMU),YES) 
	DRKNAME:=$(DRKNAME)_$(s_sim)
endif


SBPNAME:=sbp
ifeq ($(CROSS),YES)
	SBPNAME:=$(SBPNAME)_arm
endif

SIMU?=NO
ifeq ($(SIMU),YES)
	SBPNAME:=$(SBPNAME)_sim
endif





# Get all .c files from proj dir
#SRC = $(wildcard *.c) 
OBJECTS = $(SRC:%.c=$(OBJDIR)/%.o)
C_FLAGS += -O3 -Wall -pedantic -Wextra -std=gnu11


LD_FLAGS:= -l$(DRKNAME) -L$(DRK_ROOT)/lib 
LD_FLAGS+= -l$(SBPNAME) -L$(DRK_ROOT)/lib 
LD_FLAGS+= -lm -pthread -lrt
INCLUDES+= -I$(DRK_ROOT)/include -I.

ifeq ($(LIBPCAP),YES)
	INCLUDES+= -I$(PROJ_DIR)/libpcap
	LD_FLAGS+= -lpcap -L$(PROJ_DIR)/libpcap
endif

# default: compile the binary
# binary needs drklib.so at runtime.
# @drone:	it's in /usr/lib 
# @computer:	it's in current folder
default: $(PROJECT) 

$(PROJECT) : $(OBJECTS)
	@echo -n "$(GRN)Linking for$(NC)"
	@if [ "$(CROSS)" = "NO" ]; then echo -n "$(BLUE) X86. $(NC)"; else echo -n "$(RED) ARM. $(NC)"  ; fi
	@$(CC) $(OBJECTS) $(LD_FLAGS) -Wl,-rpath -Wl,$(RPATH) -o $(PROJECT_BIN)
	@echo "$(GRN)Built $(RED)$(PROJECT)$(NC)"


$(OBJDIR)/%.o : %.c %.h
	@mkdir -p $(OBJDIR)
	@if [ "$(CROSS)" = "NO" ]; then echo -n "$(BLUE)Compiling $(NC)" ; else echo -n "$(BLUE)Crosscompiling $(NC)" ; fi
	@echo "[" $< "] into [" $@ "]"
	@$(CC) -c $(C_FLAGS) $(INCLUDES) $< -o $@

clean:
	$(RM) $(OBJECTS)

realclean: clean
	$(RM) $(PROJECT_BIN)

print-%  : 
	@echo $* = $($*)
	
.PHONY: clean all
