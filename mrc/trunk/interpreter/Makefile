######################################################################
#
# Makefile for "interpreter" module.
# $Id: Makefile,v 1.4 2007/07/03 14:30:00 naa Exp $

CC=gcc
LFLAGS=-lm
CFLAGS=-Wall -g -I../robotinc  -I../../../aumat/trunk
#-I/usr/local/smr/include

SOURCES=interp.tab.c loader.c
OBS=$(SOURCES:.c=.o)

TESTPROG= test
TESTSOURCES=$(TESTPROG).c $(SOURCES)
TESTOBS=$(TESTSOURCES:.c=.o)

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	../librobot.a($(OBS))

interp.tab.c:	interp.y
	bison interp.y
######################################################################
#
# Remove backup, object, dependency, test and core files

clean:	
	rm -f *~ *.o *.d $(TESTPROG) core core.*
	rm interp.tab.c

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
