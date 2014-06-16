# makefile $Id: Makefile 225 2013-08-18 06:04:45Z jcan $
#
# libraryname (for shared library)
libname = aupcltest.so
#
CXX = g++
#
ifeq ($(USE_PCL),)
  # find version 1.4 include directory, and use if it is there
  PCL17 := $(shell ls -d /usr/include/pcl-1.7 2>/dev/null)
  PCL15 := $(shell ls -d /usr/include/pcl-1.5 2>/dev/null)
  PCL16 := $(shell ls -d /usr/include/pcl-1.6 2>/dev/null)
  ifeq ($(PCL17),)
     ifeq ($(PCL15),)
	ifeq ($(PCL16),)
	else
	export USE_PCL:=1.6
	endif
     else
       export USE_PCL:=1.5
     endif
  else
    export USE_PCL:=1.7
  endif
endif
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -I/usr/include/pcl-$(USE_PCL) -I/usr/include/eigen3 -I/usr/include/vtk-5.2
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`

#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -lpcl_segmentation -lpcl_filters -lpcl_visualization
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -fpermissive -Wall -fPIC $(DEFINES) -std=c++0x
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = ufuncpcltest.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	$(CXX) -o $(shlib) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
