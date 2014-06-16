# makefile
# $Rev: 197 $
# $Id: Makefile 197 2013-05-20 16:43:25Z jcan $
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 -L../../lib -lumap4o -lurob4o -lugen4o \
          -lcurses -lreadline -liau_mat -lgstreamer-0.10 -lgobject-2.0 -lglib-2.0 -lpthread -lpng12 -ldl
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
ifeq ($(OPENCV),)
  # for opencv version 2.1
  #LDFLAGS += -lhighgui
endif
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
#   LDFLAGS += -lcxcore -lcv -lhighgui
# else
#   LDFLAGS  += `pkg-config opencv-$(OPENCV) --libs`
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif

# Object files to produce before link
objects = ucamcompclient.o uclientfuncimggui.o uclientfunclaser.o \
          uclientfunclasergui.o ulaserdataset.o uclientfuncobstgrp.o \
	  uobstgrp.o uclientfuncsf.o uclientfuncpath.o usfpool.o
# executable file name
exe = uclient

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	cp $(exe) ../../build/bin

include ../../include/dependencies4objects.mk
