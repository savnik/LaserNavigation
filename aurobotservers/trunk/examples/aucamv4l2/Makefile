# makefile
#
# libraryname (for shared library)
libname = auv4l2.so
#
# preprocessor flags like -I(include dir)
# $(pkg-config --cflags --libs gstreamer-0.10)
CPPFLAGS = -I../../include -I/usr/include/gstreamer-0.10 \
           -I/usr/include/glib-2.0 \
           -I/usr/lib/i386-linux-gnu/glib-2.0/include \
           -I/usr/include/libxml2
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) \
          -L/usr/lib/i386-linux-gnu \
          -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 \
          -lxml2 -lgthread-2.0 -lrt -lglib-2.0
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
#
# Object files to produce before link
objects = ufunccamv4l2.o

# shared library file name (version 0)
shlib = $(libname).0

BIN_PATH := ../../bin
INSTALL_BIN_PATH := ../../bin

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)
	cp $(shlib) $(BIN_PATH)
  
.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-@rm -fr *~ .deps

install:
	c++ -o $(shlib) $(objects) $(LDFLAGS)
	cp $(shlib) $(INSTALL_BIN_PATH)

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
