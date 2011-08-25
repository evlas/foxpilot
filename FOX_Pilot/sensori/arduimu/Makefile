include ../../environment.mk

INCLUDE += -I../../include/

CFLAGS += $(INCLUDE)

OBJ_ARDUIMU  := arduimu.o

ANAME := libarduimu.a

default: all

all: $(OBJ_ARDUIMU) $(ANAME)

$(ANAME): $(OBJ_ARDUIMU)
		ar r $(ANAME)

clean:
		-rm -f $(OBJ_ARDUIMU) $(ANAME) 

rebuild: clean all