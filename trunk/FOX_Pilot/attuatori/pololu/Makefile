include ../../environment.mk

INCLUDE += -I../../include/

CFLAGS += $(INCLUDE)

OBJ_POLOLU := pololu.o

ANAME := libpololu.a

default: all

all: $(OBJ_POLOLU) $(ANAME)

$(ANAME): $(OBJ_POLOLU)
		ar r $(ANAME)

clean:
		-rm -f $(OBJ_POLOLU) $(ANAME) 

rebuild: clean all