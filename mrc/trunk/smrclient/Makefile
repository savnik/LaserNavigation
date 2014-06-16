######################################################################
#
# Makefile for "smrclient" module.
# $Id: Makefile,v 1.1 2008/12/22 11:07:24 naa Exp $

CC=gcc
LFLAGS= -L../
LIBS=	-lrobot -lm  -lpthread -lncurses -lexpat 
CFLAGS=-Wall -g  -I../robotinc -I/usr/local/smr/include 

SOURCES=dxl_server.c 
OBS=$(SOURCES:.c=.o)

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	dxl_server


######################################################################
#
# Remove backup, object, dependency, test and core files

clean:
	rm -f *~ *.o *.d $(TESTPROG) core core.*



smrclient: $(OBS)  ../librobot.a
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) $(OBS)  $(LIBS)

dxl_server: $(OBS)  ../librobot.a
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) $(OBS)  $(LIBS)



######################################################################
#
# test programs

test: $(TESTOBS)
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTPROG) $(TESTOBS)
       
          
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
