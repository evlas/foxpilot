include ../../environment.mk

INCLUDE += -I../../include/

CFLAGS += $(INCLUDE)

OBJ_BATTERIA := mybatteria.o

ANAME = libmybatteria.a

default: all

all: $(OBJ_BATTERIA) $(ANAME)

$(ANAME): $(OBJ_BATTERIA)
		ar r $(ANAME)

clean:
		-rm -f $(OBJ_BATTERIA) $(ANAME) 

rebuild: clean all