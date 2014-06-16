# makefile
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
# linker flags like e.g. -lpthread 
# and -O0 (no optimize and -g3 full debug info)
LDFLAGS = -g3 -L../../lib 
# C++ compiler flags
CXXFLAGS = -O0 -g3 -Wall -fPIC

# Object files to produce before link
objects =  aucircle.o aufeature.o auextractfeatures.o \
           auline.o circlefit.o \
           polarlinefit.o rangedata.o
# executable file name
exe = libauextractfeatures
lib = libauextractfeatures.a

# compile all - all objects depend on all other
all: $(objects)
#	c++ -c  $(objects) $(LDFLAGS)
# 	c++ -o $(exe) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	-rm -f $(objects) $(lib)
	-@rm -fr *~ .deps


install:
	-rm ../../include/auef/*.h
	cp *.h ../../include/auef
	ar -r $(lib) $(objects)
	ranlib $(lib)
	cp $(lib) ../../lib


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
