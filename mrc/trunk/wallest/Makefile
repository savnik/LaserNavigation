######################################################################
#
# Makefile for "lineest" module.
# $Id: Makefile,v 1.1.1.1 2005/11/22 21:39:59 naa Exp $

CC=gcc
LFLAGS=
CFLAGS=-Wall -g

SOURCES=wallest.c
OBS=$(SOURCES:.c=.o)

TESTPROG=
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
