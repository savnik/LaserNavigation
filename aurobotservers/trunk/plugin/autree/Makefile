# makefile $Id: Makefile 174 2013-03-30 06:41:59Z jcan $
#
# libraryname (for shared library)
libname = autree.so
# Trap for jensen, where g++ is too old to compile the Kincet library
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include 
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname) -L../../lib -lusb-1.0 
#
# extra preprocessor defines (mainly for module testcode)
DEFINES := -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -fPIC $(DEFINES)
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
objects = ufunctree.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	$(CXX) -o $(shlib) $(objects) $(LDFLAGS)

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
	@$(CXX) $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects) )
