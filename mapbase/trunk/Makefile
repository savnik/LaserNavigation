# Makefile for mapbase plugin
#
# libraryname (for shared library)
libname = mapbase.so
#
AUROBOTSERVERPATH = ../../
DGLIBPATH = dglib
CC = gcc
CXX = g++
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -I../../aurobotservers/trunk/include -I$(AUROBOTSERVERPATH)/include -I$(DGLIBPATH)
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# C compiler flags
CFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -lexpat
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../aurobotservers/trunk/include/opencv.mk
include ../../aurobotservers/trunk/include/opencv_flags.mk
# 
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# Object files to produce before link
objects = ufunctionmapbase.o uresmapbase.o mapline.o mappoint.o ExpatXMLParser.o graphmap.o mapcontainers.o

#DIRTY HACK!
#libdgl object files
LIBDGLOBJ := $(DGLIBPATH)/avl.o $(DGLIBPATH)/tavl.o $(DGLIBPATH)/tree.o
LIBDGLOBJ := $(LIBDGLOBJ) $(DGLIBPATH)/heap.o $(DGLIBPATH)/graph.o $(DGLIBPATH)/helpers.o
LIBDGLOBJ := $(LIBDGLOBJ) $(DGLIBPATH)/graph_v1.o $(DGLIBPATH)/graph_v2.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects) $(LIBDGLOBJ)
	@echo "Linking mapbase plugin"
	c++ -o $(shlib) $(objects) $(LIBDGLOBJ) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps
	$(MAKE) -C $(DGLIBPATH) clean

install:
	cp $(shlib) lib/

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

-include $(patsubst %.o, .deps/%.d, $(objects) $(LIBDGLOBJ))
