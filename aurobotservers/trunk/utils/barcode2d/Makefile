# makefile
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
# linker flags like e.g. -lpthread and -L/usr/local/lib/include
#                  and -O0 (no optimize and -g3 full debug info
LDFLAGS = -g3 -L../../lib -lugen4o
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall

# Object files to produce before link
objects = barcode2d.o
# executable file name
exe = barcode2d

# compile all - all objects depend on all other
all: $(objects)
	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(exe) $(objects)
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

-include $(patsubst %.o, .deps/%.d, $(objects) )
