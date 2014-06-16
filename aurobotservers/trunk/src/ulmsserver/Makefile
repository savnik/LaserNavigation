# makefile
# $Rev: 197 $
# $Id: Makefile 197 2013-05-20 16:43:25Z jcan $
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info)
LDFLAGS = -g3 -O0 -L../../lib -lurob4o -lugen4o -lumap4o -ldl -rdynamic \
          -lcurses -lreadline -lpng -liau_mat -lgstreamer-0.10 -lgobject-2.0 -lglib-2.0 -lpthread
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects = ufunctionlaser.o userverstatic.o ulmsserver.o usick.o \
          usickdata.o ufunclaserbase.o \
          ulaserpool.o uhokuyo.o ulaserdevice.o ulaserdata.o \
          ulasersim.o uclientfuncsimscan.o \
          uresv360.o uv360scan.o uv360meas.o \
	  ufuncv360.o ufakedevice.o ureplaydevice.o \
          ulms100.o
# executable file name
exe = ulmsserver
lib = libulms4o.a

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
	cp $(exe) ../../build/bin
  
.PHONY : clean install mklib
clean :
	rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps


install: mklib
	cp $(exe) ../../build/bin

mklib:
	ar -r $(lib) $(objects)
	-ar -d $(lib) $(exe).o
	ranlib $(lib)
	cp $(lib) ../../lib
	-rm ../../include/ulms4/*.h
	cp *.h ../../include/ulms4

include ../../include/dependencies4objects.mk

