PWD		:= $(shell pwd)

CONFIG_PLATFORM_ARM_DRONE=y
CONFIG_PLATFORM_I386_PC=n


ifeq ($(CONFIG_PLATFORM_ARM_DRONE), y)
EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN
ARCH := arm
CROSS_COMPILE := ${DRK_ROOT}/toolchain/bin/arm-cortex_a8-linux-gnueabi-
KERNEL_DIR ?= $(DRK_ROOT)/kernel/linux-ardrone2.2
endif

ifeq ($(CONFIG_PLATFORM_I386_PC), y)
EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN
SUBARCH := $(shell uname -m | sed -e s/i.86/i386/)
ARCH ?= $(SUBARCH)
KERNELRELEASE := `uname -r`
#KERNELRELEASE := `uname -r`
KERNEL_DIR := /lib/modules/$(KERNELRELEASE)/build
endif


obj-m		:= v4l2loopback.o

PREFIX ?= /usr/local
BINDIR  = $(PREFIX)/bin
MANDIR  = $(PREFIX)/share/man
MAN1DIR = $(MANDIR)/man1


MODULE_OPTIONS = devices=2

##########################################
# note on build targets
#
# module-assistant makes some assumptions about targets, namely
#  <modulename>: must be present and build the module <modulename>
#                <modulename>.ko is not enough
# install: must be present (and should only install the module)
#
# we therefore make <modulename> a .PHONY alias to <modulename>.ko
# and remove utils-installation from 'install'
# call 'make install-all' if you want to install everything
##########################################


.PHONY: all install clean distclean
.PHONY: install-all install-utils install-man
.PHONY: modprobe v4l2loopback

# we don't control the .ko file dependencies, as it is done by kernel
# makefiles. therefore v4l2loopback.ko is a phony target actually
.PHONY: v4l2loopback.ko

all: v4l2loopback.ko
v4l2loopback: v4l2loopback.ko
v4l2loopback.ko:
	@echo "Building v4l2-loopback driver..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
	#$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_DIR) M=$(PWD) modules
	
#install-all: install install-utils install-man
#install:
#	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
#	depmod -a  $(KERNELRELEASE)

#install-utils: utils/v4l2loopback-ctl
#	$(INSTALL_DIR) "$(DESTDIR)$(BINDIR)"
#	$(INSTALL_PROGRAM) $< "$(DESTDIR)$(BINDIR)"

#install-man: man/v4l2loopback-ctl.1
#	$(INSTALL_DIR) "$(DESTDIR)$(MAN1DIR)"
#	$(INSTALL_DATA) $< "$(DESTDIR)$(MAN1DIR)"

clean:
	rm -f *~
	rm -f Module.symvers Module.markers modules.order
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

distclean: clean
	rm -f man/v4l2loopback-ctl.1


