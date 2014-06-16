# makefile
# $Rev: 59 $
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# preprocessor flags like -I(include dir)
CPPFLAGS = 
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info)
LDFLAGS =
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

ifeq ($(OPENCV),)
  # for opencv version 2.1
  LDFLAGS += -lhighgui
endif

# Object files to produce before link
objects = cameracalibrate.o
# executable file name
exe = cameracalibrate

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps

install:
	cp $(exe) ../../build/bin
help:
	echo OPENCV $(OPENCV) version

include ../../include/dependencies4objects.mk
