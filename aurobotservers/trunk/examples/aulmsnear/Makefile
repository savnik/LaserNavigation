# makefile
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# libraryname (for shared library?)
libname = aulmsnear.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname)
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = ufuncnear.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)
	cp $(shlib) ../../build/lib
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps


install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
