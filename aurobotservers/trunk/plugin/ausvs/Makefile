# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library)
libname = ausvs.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) \
          -L../../lib -lsvs -lsvscalc -lsvscap -lsvslagr -ldcap -L../../lib/svs
# -ldcap-gcc40
# extra preprocessor defines
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link (all *.cpp files)
objects = ufuncsvs.o uressvs.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
