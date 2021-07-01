RED=\033[0;31m
GRN=\033[;32m
BLUE=\033[;34m
NC=\033[0m



SIMU?=NO # this compiles different versions of functions
CROSS?=NO #versus yes - this changes the compiler only
BS?=YES #versus UAV - this changes what gets compiled.

s_uav:=UAV
s_bs:=BS
s_sim:=sim
s_cc:=arm

STRING1:=$(STRING1)$(STRING2)



DRKNAME=libdrk

TARGET:=$(DRKNAME)
ifeq ($(CROSS),YES)
		TARGET:=$(TARGET)_$(s_cc)
endif

ifeq ($(BS),YES)
		TARGET:=$(TARGET)_$(s_bs)
else
		TARGET:=$(TARGET)_$(s_uav)
endif

ifeq ($(SIMU),YES) 
	TARGET:=$(TARGET)_$(s_sim)
endif
TARGET:=$(TARGET).so


INCDIR:=$(DRK_ROOT)/include
SRCDIR:=.
BINDIR:=$(DRK_ROOT)/lib

#obj directory
ifeq ($(CROSS),NO)
		OBJDIR=./obj_x86
else
		OBJDIR=./obj_arm
endif



LD_FLAGS=-lm -pthread -lrt 
#LD_FLAGS = -lm -lpthread -lrt -ldrk_sched

C_FLAGS= -g -O3 -fpic -Wall -Wextra -Wshadow -Wunreachable-code -D_FORTIFY_SOURCE=2 -pedantic -std=gnu11
#-O1

#Verbose makes all printfs output line ,funcitoname and module
ifdef VERBOSE
	C_FLAGS+=-D VERBOSE
endif

# log functions are enable. saving data into files.
ifdef LOGDATA
	C_FLAGS+=-D LOGDATA
endif

# simulator (is x86), normal x86, or drone (non-x86,ie,arm).
ifeq ($(SIMU),YES)
	CC=gcc
	C_FLAGS+= -D SIMULATOR
endif

ifeq ($(CROSS),NO)
	CC=gcc
	C_FLAGS+=-D X86 
else
	C_FLAGS+= -mtune=cortex-a8 -march=armv7-a
	include $(DRK_ROOT)/toolchain.mk # get crosscompiler path
endif

# simulator (is x86), normal x86, or drone (non-x86,ie,arm).
ifeq ($(BS),YES)
	C_FLAGS+=-D BS
endif



# select .C source code to compile:
ifeq ($(BS),YES)
	# basestation code:
	SRC := pdr.c
	SRC += tdma.c
	SRC += tdma_utils.c
	SRC += tdma_slot.c
	SRC += packet_manager.c
	SRC += utils.c
	SRC += video.c
	SRC += image_process.c
	SRC += keyboard.c
else	
	# UAV code:
	# consider all
	SRC := $(wildcard *.c)
	#remove these:
	SRC := $(filter-out rssi_localization.c, $(SRC))
	SRC := $(filter-out slip_stream_server.c, $(SRC))
	SRC := $(filter-out expansion_sensor_api.c, $(SRC))
	SRC := $(filter-out expansion_sensor_serial_handlers.c, $(SRC))
	SRC := $(filter-out drk_udp.c, $(SRC))

endif
ifeq ($(SIMU),NO) 
	SRC := $(filter-out sim_clock.c, $(SRC))
endif

toberemoved:= $(wildcard $(OBJDIR)/*.o) 

# pathnames 
OBJECTS:=$(SRC:%.c=$(OBJDIR)/%.o)
INCLUDES:=$(SRC:%.c=$(INCDIR)/drk/%.h)




default: $(TARGET) 

# define objects to compile
all: $(SRC) $(TARGET)

#.so file:
$(TARGET) : $(OBJECTS)
	@mkdir -p $(BINDIR)
	@echo -n "$(GRN)Linking$(NC) ";
	@if [ "$(CROSS)" = "YES" ]; then\
		echo "$(BLUE)$(TARGET)$(NC)";\
	else\
		echo "$(RED)$(TARGET)$(NC)";\
	fi
	$(CC) -shared $^ $(LD_FLAGS) -o $(BINDIR)/$(TARGET) 
	@echo -n "$(GRN)Built$(NC) "
	@if [ "$(CROSS)" = "YES" ]; then\
		echo "$(BLUE)$(TARGET)$(NC)";\
	else\
		echo "$(RED)$(TARGET)$(NC)";\
	fi
	


$(OBJDIR)/%.o : %.c ../../../include/drk/%.h ../../../include/drk/tdma_types.h ../../../include/drk/utils.h
	@if [ "$(CROSS)" = "NO" ]; then\
		echo "$(BLUE)Compiling $(NC)[" $< "] into [" $@ "]";\
	else\
		echo "$(RED)Crossompiling $(NC)[" $< "] into [" $@ "]";\
	fi
	
	$(CC) $(C_FLAGS) -I$(INCDIR) -c $< -o $@
	
print-%  : 
	@echo $* = $($*)

printvars: 
	@echo objects: $(OBJECTS)
	@echo
	@echo includes: $(INCLUDES)
	@echo
	@echo Bindir: $(BINDIR)
	
clean:

	$(RM) $(toberemoved)

realclean: clean
	$(RM) $(TARGET)
	$(RM) $(DRK_ROOT)/lib/$(TARGET)

.PHONY: clean all
