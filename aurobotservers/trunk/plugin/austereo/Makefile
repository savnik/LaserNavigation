# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library)
libname = austereo.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include 
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# -I/usr/include/opencv-2.3.1/
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
# ifeq ($(OPENCV),2)
#   CXXFLAGS += `pkg-config opencv --cflags`
#   LDFLAGS += `pkg-config opencv --libs`
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
#   LDFLAGS += `pkg-config opencv-$(OPENCV) --libs`
# endif
##
# Object files to produce before link
objects = ufuncstereo.o

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

help:
	echo ppp$(OCV2)ppp $(OPENCV) 

include ../../include/dependencies4objects.mk
