######################################################################
#
# Makefile for Robot Manager AUServer plugin
#
# $Id: Makefile 55 2009-03-31 22:02:17Z s021786 $

######################################################################

# libraryname (for shared library)
libname = aumanager.so
#
# AU Robot Server location (if this is placed outside aurobotservers folder
AUROBOTDIR = ../../../aurobotservers/trunk
MAPBASEDIR = ../../../mapbase/trunk
BINDIR = ../bin
#
# preprocessor flags like -I(include dir)
INCLUDES = -I$(AUROBOTDIR)/include -I$(MAPBASEDIR)
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g -shared -Wl,-soname,$(libname) -lpthread -lexpat -lm
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g -O0 -Wall -pedantic -fPIC $(DEFINES) 
#CXXFLAGS += -Wno-unused-but-set-variable
include ../../../aurobotservers/trunk/include/opencv.mk
include ../../../aurobotservers/trunk/include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif
#
#Source files
SRC :=        ufuncmanager.cpp uresmanager.cpp 
SRC := $(SRC) robotmanager.cpp smrclgenerator.cpp smrclinterface.cpp
SRC := $(SRC) rulesmrclgenerator.cpp
#
#Create list of objects
OBJ=$(SRC:.cpp=.o)

# shared library file name (version 0)
shlib = $(libname).0

###### .c to .o #####
%.o: %.cpp
	@echo "   * Compiling $<..  "
	g++ $(CXXFLAGS) $(INCLUDES) -c $<

# compile all - all objects depend on all other (objects)
all: $(OBJ)
	@echo -n "   * Linking aumanager plugin..  "
	g++ -o $(shlib) $(OBJ) $(LDFLAGS)
	@echo "Done"
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(OBJ)
	-rm -f *~ *.o

install: all 
	@echo -n "   * Copying robotmanager plugin..   "
	@mkdir -p $(BINDIR)
	@cp $(shlib) $(BINDIR)
	@echo "Done"
