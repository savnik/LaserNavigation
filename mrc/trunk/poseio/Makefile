######################################################################
#
# Makefile for "posereader" module.
# $Id: Makefile,v 1.1.1.1 2005/11/22 21:39:59 naa Exp $

CC=gcc

LFLAGS=
CFLAGS=-Wall -g

SOURCES=poseio.c
OBS=$(SOURCES:.c=.o)

TESTPROG=posetest
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTPROG).o $(OBS)

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	$(OBS)

../polyline/polyline.o:
	make -C ../polyline

######################################################################
#
# Remove backup, object, dependency, test and core files

clean:	
	rm -f *~ *.o *.d $(TESTPROG) core core.*

######################################################################
#
# test programs

test: $(TESTOBS)
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTPROG) $(TESTOBS)

######################################################################
#
# .c to .o

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -o $@

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
