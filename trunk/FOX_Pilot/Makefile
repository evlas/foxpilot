include environment.mk
#include common.mk

SRC_FIRMWARE := firmware/main.c \
				firmware/attuatori.c \
				firmware/sensori.c \
				firmware/watchdog.c \
				firmware/groundcontrol.c \
				firmware/pilota.c \
				firmware/filtri.c \
				firmware/lookup_sin_cos.c \
				firmware/matrix.c \
				firmware/pid.c \
				firmware/transformation.c 

SRC_ATTUATORI := attuatori/pololu/pololu.c

SRC_SENSORI := sensori/arduimu/arduimu.c \
			   sensori/batteria/mybatteria.c

SRC_PILOTA := pilota/quad/quad.c

SRC_GROUNCONTROLL := groundcontrol/proto.c \
					 groundcontrol/TCP_Server/tcpserver.c \
					 groundcontrol/UDP_Server/udpserver.c \
					 groundcontrol/waypoint/waypoint.c

SOURCES := $(SRC_FIRMWARE) $(SRC_ATTUATORI) $(SRC_SENSORI) $(SRC_PILOTA) $(SRC_GROUNCONTROLL)

PROGRAM := fox_pilot

SUBDIRS := firmware \
		   sensori \
		   attuatori

#directory-build:	
#	list='$(SUBDIRS)'; \
	for subdir in $$list; do \
	  (cd $$subdir && $(MAKE)); \
	done

#directory-clean:	
#	list='$(SUBDIRS)'; \
	for subdir in $$list; do \
	  (cd $$subdir && $(MAKE) clean); \
	done

OBJECTS=$(SOURCES:.cpp=.o)

$(PROGRAM): $(OBJECTS) 
	$(CC) $(CFLAGS) $(LOADLIBES) -I. -Iinclude -Imavlink/include/common -o $@ $^

clean: 
	rm -rf $(PROGRAM)
#directory-clean

all: $(PROGRAM) 
#directory-build 

# This is GNU makefile extension to notify that roughly means: 'clean' does
# not depend on any files in order to call it.
.PHONY: clean

