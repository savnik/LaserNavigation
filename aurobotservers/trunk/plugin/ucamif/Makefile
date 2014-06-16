# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library?)
libname = ucamif.so
#
# preprocessor flags like -I(include dir)
#CPPFLAGS = -I/usr/src/linux/drivers/usb/media -I/home/jca/cpp/lib/include
#CPPFLAGS = -I/usr/src/linux/drivers/usb/media/pwc -I/home/jca/cpp/lib/include
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
LDFLAGS = -g3 -O0 -shared -Wl,-soname,$(libname)
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = uclientcamifgmk.o uclientcamifpath.o uclientfuncgmk.o \
          uclientfuncpath.o ufunctioncamif.o \
          uclienthandlercamif.o ugmkpool.o uvisdata.o \
          urescamifimg.o urescamifcam.o
# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	-rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	# Copying $(shlib) to local ../../build/lib/$(shlib)
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
