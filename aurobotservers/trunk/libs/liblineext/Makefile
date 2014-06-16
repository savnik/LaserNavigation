######################################################################
#
# Makefile for "iau_mat" module.
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $

CC=g++
LFLAGS=-L.
#CFLAGS=-Wall -g3 -I. -O1
CXXFLAGS:= -Wall -g3 -I. -O1

# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -g3 -I. -O1 -I/usr/local/include/opencv
#CXXFLAGS = -I../../include -g3 -I. -O1
#
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
#   LDFLAGS += -lcxcore -lcv
#   CPPFLAGS += -I/usr/include/opencv
# else
#   LDFLAGS  += `pkg-config opencv-$(OPENCV) --libs`
#   CPPFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif

SOURCES=LEL_cvModCanny.cpp LEL_main.cpp LEL_ransac.cpp LEL_tables.cpp LEL_hough.cpp\
	LEL_quadtree.cpp LEL_structures.cpp LEL_utilities.cpp 
#localizationutils.cpp

OBS=$(SOURCES:.cpp=.o)

INSTALL_PATH=/usr/local/smr

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	libiau_lel.a($(OBS))

######################################################################
#
# Remove backup, object, dependency, test and core files

clean:	
	rm -f *~ *.o *.d core core.* *.a

######################################################################
#
# Documentation

doc: 
	doxygen Doxyfile
   
help: 
	echo OPENCV $(OPENCV)
	echo CPPFLAGE $(CPPFLAGS)
	echo LDFLAGS $(LDFLAGS)
	echo CXXFLAGS $(CXXFLAGS)

######################################################################
#
# Installation of the library in INSTALL_PATH/lib and the include
# file in INSTALL_PATH/include. This must be run as root.

install: 
	-cp -f libiau_lel.a ../../lib/
#	cp -f libiau_lel.h $(INSTALL_PATH)/include/.

######################################################################
#
# .c to .o

%.o:	%.c
	$(CC) $(CPPLAGS) $(CXXFLAGS) -c $< -fPIC 
%.o:	%.cpp
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -fPIC 

######################################################################
#
# Automatic dependencies

%.d:	%.cpp
	@$(SHELL) -ec '$(CC) -MM $(CFLAGS) $< \
        | sed '\''s/\($*\)\.o[ :]*/\1.o $@ : /g'\'' > $@; \
          [ -s $@ ] || rm -f $@'

######################################################################
#
# Include automatic dependencies

-include $(SOURCES:.cpp=.d)
