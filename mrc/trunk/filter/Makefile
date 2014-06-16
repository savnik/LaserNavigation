######################################################################
#
# Makefile for "kalman_odometry" module.
# $Id: Makefile,v 1.1 2006/06/14 11:30:32 ex30 Exp $

CC=gcc
#LFLAGS=-L../iau_mat -liau_mat  -lm
LFLAGS= -lm

CFLAGS= -Wall -g -I../robotinc -I../../../aumat/trunk/ -I/usr/local/include
LIBS=/usr/local/lib/libiau_mat.a

SOURCES=filter.c
OBS=$(SOURCES:.c=.o)

TESTPROG=test
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
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTPROG) $(TESTOBS) $(LIBS)


######################################################################
#
# Documentation

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
