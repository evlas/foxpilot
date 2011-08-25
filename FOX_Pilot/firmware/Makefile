include ../environment.mk

INCLUDE += -I../include/ -I../

#SERIAL_OBJ=serial.o ring.o
#GROUNDCONTROL_OBJ=groundcontrol.o tcpserver.o udpserver.o proto.o
#SENSORS_OBJ=sensori.o arduimu.o batteria.o
#ATTUATORS_OBJ=attuatori.o pololu.o
#PILOTA_OBJ=pilota.o manual.o quad.o

SENSORS_OBJ=sensori.o
ATTUATORS_OBJ=attuatori.o
WATCHDOG_OBJ=watchdog.o

#OBJS= main.o loadconf.o filtri.o matrix.o $(SERIAL_OBJ) $(WATCHDOG_OBJ) $(GROUNDCONTROL_OBJ) $(SENSORS_OBJ) $(ATTUATORS_OBJ) $(PILOTA_OBJ)

OBJS= main.o filtri.o  matrix.o $(SENSORS_OBJ) $(ATTUATORS_OBJ) $(WATCHDOG_OBJ) $(GROUNDCONTROL_OBJ) $(SENSORS_OBJ) $(ATTUATORS_OBJ) $(PILOTA_OBJ)

CFLAGS+= $(INCLUDE)

default: all

all: $(OBJS)

clean:
	-rm -f $(OBJS)

rebuild: clean all

