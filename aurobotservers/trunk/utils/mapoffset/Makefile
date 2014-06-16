# makefile
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# $Rev: 59 $
# preprocessor flags like -I(include dir)
CPPFLAGS =
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info)
LDFLAGS =
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall

# Object files to produce before link
objects = mapoffset.o
# executable file name
exe = mapoffset

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(exe) $(objects) $(lib)
	-@rm -fr *~ .deps

install:
	cp $(exe) ../../build/bin

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

-include $(patsubst %.o, .deps/%.d, $(objects))
