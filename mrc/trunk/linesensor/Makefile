######################################################################
#
# Makefile for "linesensor" module.
# $Id: Makefile,v 1.2 2008/04/21 10:05:13 naa Exp $

CC=gcc
LFLAGS=-lm
CFLAGS=-Wall -g

SOURCES=linesensor.c
OBS=$(SOURCES:.c=.o)

TESTPROG=linesensortest
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
