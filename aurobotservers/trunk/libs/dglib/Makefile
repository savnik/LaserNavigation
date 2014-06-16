# makefile
MAKE = make
lib = libdgl.a
LIBSUBDIRS = src

.PHONY : $(LIBSUBDIRS) clean install

# compile all - all objects depend on all other
all: $(LIBSUBDIRS)

$(LIBSUBDIRS):
	$(MAKE) -C $@

clean:
	$(MAKE) --silent -C src clean

install:
	cp src/$(lib) ../../lib
