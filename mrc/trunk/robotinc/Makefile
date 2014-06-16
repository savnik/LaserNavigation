######################################################################
#
# Makefile for "irmodule" module.
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $

CC=gcc
LFLAGS=
CFLAGS=-Wall -g

SOURCES=ir2dist.c
OBS=$(SOURCES:.c=.o)

TESTPROG=
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTSOURCES:.c=.o)

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	
	./copyh

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
