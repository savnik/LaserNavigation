# makefile $Id: Makefile 257 2013-10-06 12:12:56Z jcan $
#
# libraryname (for shared library)
libname = autracking.so
#
CXX = g++
#
ifeq ($(USE_PCL),)
  # find version PCL 1.4 or 1.5 include directory, and use if it is there
  PCL14 := $(shell ls -d /usr/include/pcl-1.7 2>/dev/null)
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
CPPFLAGS = -I../../include 
CPPFLAGS += -I/usr/include/pcl-$(USE_PCL) -I/usr/include/eigen3 
CPPFLAGS += -I/usr/include/vtk-5.6 -I/usr/include/vtk-5.8 -I/usr/include/vtk-5.2
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -lpcl_segmentation -lpcl_filters \
          -lpcl_visualization -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_octree \
          -lflann_cpp -lpcl_sample_consensus -lpcl_features
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES) -Wno-deprecated -std=c++0x
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = ufunctracking.o

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

######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@cc $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#'

.deps/%.d: %.cpp
	@$(CXX) $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects) )
