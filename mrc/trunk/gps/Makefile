######################################################################
#
# Makefile for "UTMgpsd-client" module.
# $Id: Makefile,v 1.1 2006/06/14 11:26:54 ex30 Exp $

CC=gcc
LFLAGS=
CFLAGS=-Wall -g -I../robotinc

SOURCES=libgps.c
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
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

lib:	./libgps.a($(OBS))

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
# test programs

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
