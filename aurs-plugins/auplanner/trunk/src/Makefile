######################################################################
#
# Makefile for Robot Planner AuServer plugin
#
# $Id: Makefile 1919 2012-05-28 15:25:57Z jca $

######################################################################

# libraryname (for shared library)
libname = auplanner.so
#
# AU Robot Server location (if this is placed outside aurobotservers folder
AUROBOTDIR = ../../../../aurobotservers/trunk
MAPBASEDIR = ../../../../mapbase/trunk
BINDIR = ../bin
#
# preprocessor flags like -I(include dir)
INCLUDES = -I$(AUROBOTDIR)/include -I$(AUROBOTDIR) -I$(MAPBASEDIR) 
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
include ../../../../aurobotservers/trunk/include/opencv.mk
include ../../../../aurobotservers/trunk/include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif
#
#Source files
SRC :=        ufuncplanner.cpp uresplanner.cpp
#SRC := $(SRC) robotmanager.cpp
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
	@echo -n "   * Linking auplanner plugin..  "
	g++ -o $(shlib) $(OBJ) $(LDFLAGS)
	@echo "Done"
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(OBJ)
	-rm -f *~ *.o

install: all 
	@echo -n "   * Copying robotplanner plugin..   "
	@mkdir -p $(BINDIR)
	@cp $(shlib) $(BINDIR)
	@cp $(shlib) $(AUROBOTDIR)/bin
	@echo "Done"
