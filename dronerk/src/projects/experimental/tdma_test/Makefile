# the name of this project

PROJECT=tdma_test

CROSS=NO
SIMU=NO
BS=YES


LOGDATA=YES
VERBOSE=YES 
#BS basestation vs drone


SRC:=main.c
PROJ_DIR=${CURDIR}
include ../../projects.mk #all juice is here


debug: 
	@echo "objects" $(OBJECTS)
	@echo "obj dir" $(OBJDIR)
	@echo "includes" $(INCLUDES)
	@echo "project" $(PROJECT_BIN)
	@echo "options: Cross" $(CROSS) ", BS" $(BS) ",SIMU" $(SIMU)

$(shell mkdir -p usb)
