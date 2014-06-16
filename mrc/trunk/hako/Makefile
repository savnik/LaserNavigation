######################################################################
#
# Makefile for "motioncontrol" module.
# $Id: Makefile,v 1.2 2005/11/30 08:35:47 ex05 Exp $

CC=gcc
LFLAGS=
CFLAGS=-Wall -g -I../robotinc

SOURCES=libhako.c
OBS=$(SOURCES:.c=.o)

TESTPROG= test
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTSOURCES:.c=.o)

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	../librobot.a($(OBS))

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
