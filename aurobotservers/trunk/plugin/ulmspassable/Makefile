# makefile $Id: Makefile 218 2013-08-06 08:15:55Z jcan $
#
CC = c++
# libraryname (for shared library?)
libname = ulmspassable.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname)
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC -D LIBRARY_OPEN_NEEDED
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = ufunctionpassable.o  ulaserpoint.o    uresobstacle.o  \
          uresrobot.o          ulaserobst.o     ulaserscan.o    \
          urespassable.o       uroadline.o      ulaserpi.o      \
          uobstaclepool.o      uresroadline.o


# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	$(CC) -o $(shlib) $(objects) $(LDFLAGS)
	cp $(shlib) ../../build/lib
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps


install:
	cp $(shlib) ../../build/lib

include ../../include/dependencies4objects.mk
