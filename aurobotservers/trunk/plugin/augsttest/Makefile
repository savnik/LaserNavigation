# makefile $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
#
# libraryname (for shared library)
#
# preprocessor flags like -I(include dir)
PKG_CONFIG_PATH=/usr/lib/i386-linux-gnu/pkgconfig
CPPFLAGS = -I../../include -I/usr/include/gstreams-0.1/gst -I/usr/include/libxml2
LDFLAGS  += `pkg-config glib-2.0 --libs`
#LDFLAGS  += `pkg-config gstreamer-0.1 --libs`
CPPFLAGS += `pkg-config glib-2.0 --cflags`
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -pthread -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lxml2 -lglib-2.0
# -shared -Wl,-soname,$(libname) 
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
objects = ufuncgst.o

# shared library file name (version 0)
exe = augsttest

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
