# makefile $Id: Makefile 224 2013-08-18 05:17:57Z jcan $
#
# libraryname (for shared library)
libname = auview.so
#
# preprocessor flags like -I(include dir)
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -L../../lib  -ugen4o
ifeq ($(USE_PCL),)
  # find version PCL 1.4 or 1.5 include directory, and use if it is there
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
CPPFLAGS  = -I../../include -I/usr/local/include/libusb-1.0 -I/usr/include/libusb-1.0
CPPFLAGS += -I/usr/include/pcl-$(USE_PCL) -I/usr/include/eigen3 
CPPFLAGS += -I/usr/include/vtk-5.6 -I/usr/include/vtk-5.8 -I/usr/include/vtk-5.2
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -lpcl_visualization \
          -L/usr/lib/vtk-5.6 -lboost_system-mt -lboost_iostreams-mt -lpcl_io -lpcl_common
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED -DEIGEN_USE_NEW_STDVECTOR -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET -DFLANN_STATIC
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fpermissive -Wno-deprecated -fPIC $(DEFINES) -std=gnu++0x

include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
sourcescpp = ufuncview.cpp unavview.cpp
#
sourcesh = $(sourcescpp:.cpp=.h)
OBJECTS = $(sourcescpp:.cpp=.o)

# shared library file name (version 0)
shlib = $(libname).0

all: $(shlib)
# compile all - all objects depend on all used files
$(shlib): $(OBJECTS) Makefile $(sources)
	c++ -o $(shlib) $(OBJECTS) $(LDFLAGS)

.o: $(sources) $(sourcesh)
	c++ $@



.PHONY : clean install
clean :
	rm -f $(shlib) $(OBJECTS)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
