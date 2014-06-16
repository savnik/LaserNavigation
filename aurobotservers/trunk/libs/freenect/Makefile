# makefile
CC = gcc
CXX = g++
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include  -I/usr/local/include/libusb-1.0 -fPIC
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 -lpthread \
          -L../../lib
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D USE_KINECT -D BUILD_NOT_AUDIO
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall $(DEFINES)
CFLAGS := -O0 -g -Wall -export-dynami $(DEFINES)
# Object files to produce before link
objects = usb_libusb10.o core.o cameras.o tilt.o  registration.o
# audio.o loader.o
# executable file name
lib = libfreenect.a

# compile all - all objects depend on all other
all: $(objects)

.PHONY : clean install
clean :
	-rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	-ar -r $(lib) $(objects)
	-ranlib $(lib)
	-cp $(lib) ../../lib
	-@rm ../../include/libfreenect.h
	cp libfreenect.h ../../include/
	cp libfreenect.hpp ../../include/
	cp registration.h ../../include/

######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#'

.deps/%.d: %.cpp
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects))
