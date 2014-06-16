# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library)
libname = audispv.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -I/usr/hertil/mangler/qt -I/usr/include/qt4 -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtCore
CPPFLAGS += `pkg-config QtCore --cflags-only-I`
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname)
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES)
CXXFLAGS += `pkg-config QtCore --cflags-only-other`
LDFLAGS += `pkg-config QtGui --libs`
# CXXFLAGS += -Wno-unused-but-set-variable
ifeq ($(OPENCV),2.1)
  export OPENCV=
endif
ifeq ($(OPENCV),)
else
  CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
endif
#
# Object files to produce before link
objects = ufuncdispv.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@cc $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#'

.deps/%.d: %.cpp
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects) )
