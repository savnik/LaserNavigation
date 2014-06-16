# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library)
libname = auv4l2.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
#CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -lv4l2 -ludev -lavcodec -lavutil
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
objects = ufuncv4l2.o v4l2_controls.o v4l2_devices.o  \
          v4l2_formats.o v4l2uvc.o colorspaces.o utils.o huffman.o \
          jpgenc.o dct.o v4l2_dyna_ctrls.o marker.o readYUV.o quant.o \
          globals.o  string_utils.o  \
             ms_time.o

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
