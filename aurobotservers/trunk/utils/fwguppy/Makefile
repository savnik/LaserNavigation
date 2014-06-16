# makefile
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# preprocessor flags like -I(include dir)
CPPFLAGS =-I../../include
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info)
LDFLAGS =  -L../../lib -lugen4o -lpng -lraw1394 -ldc1394
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects = fwguppy.o
# executable file name
exe = fwguppy

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps

install:
	cp $(exe) ../../build/bin

include ../../include/dependencies4objects.mk
