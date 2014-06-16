######################################################################
#
# Makefile for "iau_mat" module.
# $Id: Makefile,v 1.1.1.1 2006-12-12 13:48:14 lvm Exp $

CC=gcc
LFLAGS=-L.
CFLAGS=-Wall -g -I. -O1

SOURCES=matinv.c matcore.c matio.c
OBS=$(SOURCES:.c=.o)

TESTPROG=demomat
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTSOURCES:.c=.o)

INSTALL_PATH=/usr/local/smr

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	testsmart libiau_mat.a($(OBS))

######################################################################
#
# Remove backup, object, dependency, test and core files

clean:	
	rm -f *~ *.o *.d $(TESTPROG) core core.* *.a

######################################################################
#
# test programs

test: $(TESTOBS)
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTPROG) $(TESTOBS)

testsmart: smartmattest ;

smartmattest: smartmattest.o $(OBS)
	g++ $(CFLAGS) $(LFLAGS) -o smartmattest smartmattest.o $(OBS)

smartmattest.o: smartmat.hpp

######################################################################
#
# Documentation

doc: 
	doxygen Doxyfile
   
######################################################################
#
# Installation of the library in INSTALL_PATH/lib and the include
# file in INSTALL_PATH/include. This must be run as root.

install: 
	cp -f libiau_mat.a $(INSTALL_PATH)/lib/.
	cp -f iau_mat.h $(INSTALL_PATH)/include/.

######################################################################
#
# .c to .o

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -fPIC
%.o:	%.cpp
	g++ $(CFLAGS) -c $< -fPIC

######################################################################
#
# Automatic dependencies

%.d:	%.c
	@$(SHELL) -ec '$(CC) -MM $(CFLAGS) $< \
        | sed '\''s/\($*\)\.o[ :]*/\1.o $@ : /g'\'' > $@; \
          [ -s $@ ] || rm -f $@'

######################################################################
#
# Include automatic dependencies

-include $(SOURCES:.c=.d)
