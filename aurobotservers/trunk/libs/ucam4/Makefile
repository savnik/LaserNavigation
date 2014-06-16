# makefile
# $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
# $Rev: 174 $
# preprocessor flags like -I(include dir)
#CPPFLAGS = -I/usr/src/linux/drivers/usb/media
#CPPFLAGS = -I/usr/src/linux/drivers/usb/media/pwc
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 -lpng -lpthread \
          -L../../lib -lumap4o -lurob4o -lumap4o -lugen4o -lgstreamer-0.10
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED -D USE_GUPPY -D USE_NO_IEEE1394 -D USE_NO_GIGE
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects = ucammount.o \
          ucamrad.o  uimagelog.o ucampantiltdevice.o \
          ucambase.o ucamdevbase.o ucampwc.o \
	  ucamdevieee1394.o ucamdevgrabber.o ucamdevreplay.o \
          ufunctioncambase.o ufunctioncam.o ucampool.o ucampush.o \
          ufunctionimage.o ucomcam.o ucamdevgige.o ucamdevguppy.o
# executable file name
exe = ucam4
lib = libucam4o.a

# compile all - all objects depend on all other
all: $(objects) 
#	c++ -o $(exe) $(objects) $(LDFLAGS)
#	c++ -c $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	-rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps

install: 
	ar -r $(lib) $(objects)
	ranlib $(lib)
	cp $(lib) ../../lib
	-rm ../../include/ucam4/*.h
	cp *.h ../../include/ucam4


include ../../include/dependencies4objects.mk
