# makefile
#
# libraryname (for shared library)
libname = aucamlocalize.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -L../../lib -liau_mat -liau_lel -lutils
#
# extra preprocessor defines
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES)

ifeq ($(OPENCV),2.1)
  export OPENCV=
endif
ifeq ($(OPENCV),)
else
  CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
endif

#
# Object files to produce before link (all *.cpp files)
objects = ufunccamlocalize.o 

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-rm -f *~

install:
	cp $(shlib) ../../build/lib
