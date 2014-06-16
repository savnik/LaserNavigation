######################################################################
#
# Makefile for "iau_mat" module.
# $Id: Makefile 182 2013-04-14 16:39:25Z jcan $

CC=gcc
LFLAGS=-L.
CFLAGS=-Wall -g3 -I. -O0

# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -g3 -I. -O0 
#CXXFLAGS = -I../../include -g3 -I. -O1
#
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
CPPFLAGS += $(CXXFLAGS)
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

SOURCES=localizationutils.cpp 
#localizationutils.cpp

OBS=$(SOURCES:.cpp=.o)

INSTALL_PATH=/usr/local/smr

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	libutils.a($(OBS))

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
   
######################################################################
#
# Installation of the library in INSTALL_PATH/lib and the include
# file in INSTALL_PATH/include. This must be run as root.

install: 
	-cp -f libutils.a ../../lib/
#	cp -f libiau_lel.h $(INSTALL_PATH)/include/.

######################################################################
#
# .c to .o

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -fPIC
%.o:	%.cpp
	g++ $(CPPFLAGS) -c $< -fPIC

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
