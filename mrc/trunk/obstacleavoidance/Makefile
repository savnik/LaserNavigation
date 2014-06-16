######################################################################
#
# Makefile for "smrdemo" module.
# $Id: Makefile,v 1.2 2009/05/05 10:03:46 naa Exp $

CC=gcc
LFLAGS= -L../
LIBS=	-lrobot -lm  -lpthread  
CFLAGS=-Wall -g  -I../robotinc -I/usr/local/smr/include 

SOURCES=obstacle1.c
OBS=$(SOURCES:.c=.o)

TESTPROG= test
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTSOURCES:.c=.o)

SIMLIBS= ../smrsim/smrsim.o -lrobot -lm  -lpthread  -lncurses  ../guidemark/ucamlibc.a

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	test


######################################################################
#
# Remove backup, object, dependency, test and core files

clean:
	rm -f *~ *.o *.d $(TESTPROG) core core.*



obstacle1: $(OBS)  
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) $(OBS)  $(LIBS)

obstaclesim: obstaclesim.o $(OBS) 
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) obstaclesim.o $(OBS)  $(LIBS)

testlaserdata: testlaserdata.o $(OBS) 
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) testlaserdata.o $(OBS)  $(LIBS)




######################################################################
#
# test programs

test: $(TESTOBS)
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTPROG) $(TESTOBS)  $(LIBS)
       
          
######################################################################
#
# Doxggen documentation

doc:
	doxygen Doxyfile_lin
       

######################################################################
#
# .c to .o

%.o:	%.c
	$(CC) $(CFLAGS) -c $<

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
