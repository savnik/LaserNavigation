# makefile
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# $Rev: 59 $
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 -L../../lib -lugen4o -lurob4o -lugen4o
CXXFLAGS = -O0 -g3 -Wall -Wno-deprecated
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects =  uposev.o upose.o \
           uprobpoly.o uprobpolyqueue.o \
           umanoeuvre.o umanarc.o umanline.o \
	   upose2pose.o umanseq.o umanppseq.o
# executable file name
exe = umap4
lib = libumap4o.a


# compile all - all objects depend on all other
all: $(objects) 
#	c++ -c $(objects) $(LDFLAGS)
#	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	-rm -f $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	-rm ../../include/umap4/*.h
	cp *.h ../../include/umap4
	ar -r $(lib) $(objects)
#	-ar -d $(lib) $(exe).o
	ranlib $(lib)
	cp $(lib) ../../lib

include ../../include/dependencies4objects.mk
