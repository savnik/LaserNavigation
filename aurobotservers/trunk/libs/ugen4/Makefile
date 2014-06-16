# makefile
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# $Rev: 59 $
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info
CPPFLAGS := -I../../include 
# extra preprocessor defines
DEFINES=
#export aa = `pkg-config --exists opencv-2.3.2 --print-errors`
CXXFLAGS := -O0 -g3 -Wall $(DEFINES) -fPIC
LDFLAGS := -g3 -lpng
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
# Object files to produce before link
objects = conf.o     ucommon.o \
ufuzzysplit.o  uimg3dpoint.o  umatrix.o  \
usmltagin.o u2dline.o  udatabase.o      ugbmbmp.o   \
ugmk.o     uimgstat.o     uplane.o    utime.o    \
u3d.o      ufuzzyelement.o  ugbmhelp.o     uimage2.o  \
uline.o        upolygon.o ucampar.o  ufuzzypixel.o   \
ugbm.o         uimage.o   ulock.o        uposrot.o crc.o
# executable file name
exe = ugen4
lib = libugen4o.a

.PHONY : clean install pkgpath

# compile all - all objects depend on all other
all: $(objects) 

clean :
	-rm -f $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	-rm ../../include/ugen4/*.h
	cp *.h ../../include/ugen4
	ar -r $(lib) $(objects)
#	-ar -d $(lib) $(exe).o
	ranlib $(lib)
	cp $(lib) ../../lib

help:
	echo $(OPENCV) $(OCV2)

include ../../include/dependencies4objects.mk
