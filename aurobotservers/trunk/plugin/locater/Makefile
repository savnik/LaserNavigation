# Makefile for rowfinder plugin
#
# libraryname (for shared library)
libname = locater.so
#
CC = gcc
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -I../ulmsscanfeatures -I../mapbase/ -fPIC
# -I../aulibextractfeatures -I../  
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -L../../lib -lauextractfeatures -liau_mat 
#IAU MATRIX BIB:  /usr/local/lib/libiau_mat.a (on nyquist)
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
objects = ufunctionlocater.o ureslocater.o linreg.o tools.o

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
