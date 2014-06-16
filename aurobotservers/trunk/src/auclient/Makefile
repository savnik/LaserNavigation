# makefile
# $Rev: 197 $
# $Id: Makefile 197 2013-05-20 16:43:25Z jcan $
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -I.
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3  \
          -L../../lib -lurob4o -lugen4o -lumap4o \
	  -lpng -ldl -rdynamic -lcurses -lreadline -lurob4o -liau_mat -lgstreamer-0.10 -lgobject-2.0 -lglib-2.0 -lpthread
# extra preprocessor defines (mainly for module testcode)
# change NO_USE_HIGHGUI to USE_HIGHGUI for X-display direct
# if NO_USE_HIGHGUI, then -lhighgui may be removed from LDFLAGS
#DEFINES = -D USE_HIGHGUI
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
ifeq ($(OPENCV),)
  # for opencv version 2.1
  #LDFLAGS += -lhighgui
endif

# Object files to produce before link
objects = auclient.o           ufunctiondisp.o     ureslaserif.o     \
	uclientcamifgmk.o      ufunctionlaserif.o  ureslaserifobst.o \
	uclientcamifpath.o     ugmkpool.o          ureslaserifroad.o \
	uclientfuncgmk.o       ulaserdataset.o     ureslaserifscan.o \
	uclientfuncpath.o      unavpaint.o         ureslaserifvar.o  \
	uclienthandlercamif.o  uobstaclehist.o     userverstatic.o   \
	ufeaturepool.o         urescamifimg.o      uvisdata.o        \
	ufunctioncamif.o       uresdisp.o          urescamifcam.o    \
	uresnavifman.o         urespoly.o          ufuncpoly.o \
        ureslaserifsf.o        ufunctionlaserifscan.o

# executable file name
exe = auclient

BIN_PATH := ../../build/bin
INSTALL_BIN_PATH := ../../build/bin

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
	cp -f $(exe) $(BIN_PATH)
  
.PHONY : clean install
clean :
	rm -f $(exe) $(objects)
	-@rm -fr *~ .deps


install: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)
	cp -f $(exe) $(INSTALL_BIN_PATH)

include ../../include/dependencies4objects.mk

