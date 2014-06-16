# makefile
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 \
          -L../../lib -lurob4o -lugen4o -lumap4o -lucam4o -liau_mat \
	  -lpng -ldl -rdynamic -lcurses -lreadline -lraw1394 -ldc1394 -lgstreamer-0.10 -lgobject-2.0 -lglib-2.0 -lpthread
# -ldc1394_control
# -lPvAPI
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D USE_NOT_IEEE1394 -D USE_GUPPY
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects = ucamserver.o userverstatic.o \
          ubarcode.o  \
	  ufunctioncamgmk.o ufunctioncampath.o \
	  uimgproj.o ucalibrate.o ucamparest.o uimageana.o \
	  uprobgrid.o
# executable file name
exe = ucamserver

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
	cp $(exe) ../../build/bin

.PHONY : clean install
clean :
	rm -f $(exe) $(objects)
	-@rm -fr *~ .deps


install:
	c++ -o $(exe) $(objects) $(LDFLAGS)
	cp $(exe) ../../build/bin

include ../../include/dependencies4objects.mk
