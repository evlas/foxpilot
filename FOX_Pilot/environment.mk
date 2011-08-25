ifneq (,)
This makefile requires GNU Make.
endif

ifeq ($(ARCH), x86)
CC := gcc
endif
ifeq ($(ARCH), arm)
CC := arm-linux-gnueabi-gcc
endif

CFLAGS := -g -O2 
LDFLAGS :=
#INCLUDE= -Iinclude/ -Ilibbridge/ $(KERNEL_HEADERS)
#INCLUDE= -Iinclude/

#LIBS= -L/opt/brcm/hndtools-mipsel-uclibc/lib/libbridge.a -lpthread -lm  -lbridge
LIBS := -lpthread -lm -lrt

LOADLIBES := $(LIBS)


