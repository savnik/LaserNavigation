# makefile
# $Id: Makefile 174 2013-03-30 06:41:59Z jcan $'
# $Rev: 174 $
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
LDFLAGS = -g3 -L../../lib -lugen4o -lumap4o -lpng -liau_mat
# extra preprocessor defines
DEFINES = 
CXXFLAGS = -O0 -g3 -Wall $(DEFINES)
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects =  ucmdexe.o ucomimana.o ucommoncomp.o\
           ufunctionbase.o  \
	   ufunctionimgbase.o ufunctionimgpool.o \
	   upush.o usmltag.o usmlsource.o usmlfile.o usmlstring.o \
	   umsgqueue.o ueventtrap.o uevents.o \
	   userverclient.o userverport.o userverpush.o userverqueue.o \
	   uclientport.o uclienthandler.o uclientfuncbase.o \
	   uclientfuncimage.o uobstacle.o uclientfunccam.o \
	   uclientcams.o uimgpush.o \
           ureslink.o uimagepool.o uresbase.o urespool.o \
	   usmrcl.o usockclient.o usockserv.o \
	   ufunctionposehist.o uresposehist.o \
           ufunctionvarpool.o uresvarpool.o uresclientifvar.o \
	   uvarmethod.o uvarcalc.o uvarpool.o uvariable.o \
	   uresifbase.o ufunctionif.o uresif.o ulogfile.o \
           ufuncplugbase.o uvarlog.o ureplay.o
# urob4.o
# executable file name
exe = urob4
lib = liburob4o.a

# compile all - all objects depend on all other
all: $(objects) 
#	c++ -c  $(objects) $(LDFLAGS)
# 	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	-rm -f $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	-rm ../../include/urob4/*.h
	cp *.h ../../include/urob4
	ar -r $(lib) $(objects)
#	-ar -d $(lib) $(exe).o
	ranlib $(lib)
	cp $(lib) ../../lib

include ../../include/dependencies4objects.mk

