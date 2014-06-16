# makefile $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
#
# libraryname (for shared library)
libname = auv4lgst.so
#
PKG_CONFIG_PATH=/usr/lib/i386-linux-gnu/pkgconfig
CPPFLAGS = -I../../include  -I/usr/include/libxml2
# -I/usr/include/gstreams-0.1/gst
LDFLAGS  += `pkg-config glib-2.0 --libs`
#LDFLAGS  += `pkg-config gstreamer-0.1 --libs`
CPPFLAGS += `pkg-config glib-2.0 --cflags`
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -pthread -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lxml2 -lglib-2.0
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS += -shared -Wl,-soname,$(libname)
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif
#
# Object files to produce before link
sourcescpp = ufuncv4lgst.cpp
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

######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@cc $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' 

.deps/%.d: %.cpp
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects))
