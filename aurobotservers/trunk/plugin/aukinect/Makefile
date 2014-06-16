# makefile $Id: Makefile 257 2013-10-06 12:12:56Z jcan $
#
# libraryname (for shared library)
libname = aukinect.so
# link flags
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -L../../lib 
LDFLAGS += -lboost_system-mt
#CPPFLAGS += -lboost_system-mt -lboost_iostreams-mt
#-ugen4o
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
#
CPPFLAGS  = -I../../include -I/usr/local/include/libusb-1.0 -I/usr/include/libusb-1.0
CPPFLAGS += -I/usr/include/pcl-$(USE_PCL) -I/usr/include/eigen3 
CPPFLAGS += -I/usr/include/vtk-5.6 -I/usr/include/vtk-5.8 -I/usr/include/vtk-5.2
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
LIBFREENECT := $(shell ls -d ../../lib/libfreenect.a 2>/dev/null)
ifeq ($(LIBFREENECT),)
else
  CPPFLAGS += -D USE_KINECT
  LDFLAGS += -lusb-1.0 -lfreenect
endif
# linker flags like e.g. -lpthread and -L/usr/local/lib
#
# extra preprocessor defines (mainly for module testcode)
DEFINES := -D LIBRARY_OPEN_NEEDED
ifeq ($(USE_PCL),)
  # no PCL support installed
else
  # pt. USE_PCL must be defined as something like 1.4
  DEFINES += -D USE_PCL
endif
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = ufunckinect.o

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
