# makefile
# $Rev: 239 $
# $Id: Makefile 239 2013-09-15 17:17:35Z jcan $
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include 
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`

# linker flags like e.g. -lpthread and -L/usr/local/lib/include 
#                  and -O0 (no optimize and -g3 full debug info)
LDFLAGS = -g3 -L../../lib -lurob4o -lugen4o -lumap4o -ldl -rdynamic \
           -lcurses -lreadline -lpng -lulms4o -liau_mat -lgstreamer-0.10 -lgobject-2.0 -lglib-2.0 -lpthread
# uncomment next line to use python 
#PYTHON="2.7"
ifeq ($(PYTHON),)
else
  CPPFLAGS +=  -I/usr/include/python2.7
  LDFLAGS +=   -lpython2.7 -lboost_python
  CXXFLAGS = -D USE_PYTHON 
endif
# C++ compiler flags
CXXFLAGS += -O0 -g3 -Wall
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk

# Object files to produce before link
objects = auservertest.o urawserverport.o
# executable file name
exe = auservertest

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
