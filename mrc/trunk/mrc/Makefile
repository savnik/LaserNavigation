######################################################################
#
# Makefile for "smrdemo" module.
# $Id: Makefile,v 1.2 2009/05/05 10:03:46 naa Exp $

CC=gcc
LFLAGS= -g2  -L../
# -L../../../aumat/trunk -Llibsmr -L/usr/local/lib
LIBS=	-lpthread -lncurses -lexpat  -lrobot -lm \
         ../../../rhd/trunk/build/lib/librhd.a \
         ../../../aumat/trunk/libiau_mat.a 
# -liau_mat
#         ../libsmr/libsmr.a
#         ../../../rhd/branches/rhd2.0/build/lib/librhd.a \
# /usr/local/smr/lib/librhd.a  /vhome/naa/linux/smr/libsmr/libsmr.a
CFLAGS=-Wall -g2  -I../robotinc -I../smrsim \
       -I../../../aumat/trunk
#-I/usr/local/smr/include 

SOURCES=mrc.c robotinit.c administration.c
OBS=$(SOURCES:.c=.o)

#SIMLIBS= ../smrsim/smrsim.o -lrobot -lm  -lpthread  -lncurses  ../guidemark/ucamlibc.a
SIMLIBS=  -lrobot -lm  -lpthread  -lncurses  ../guidemark/ucamlibc.a

######################################################################
#
# Main target. Compiles object files as this is not a stand alone
#              program, but a module.

all:	mrc
sim:	smrdemosim

######################################################################
#
# Remove backup, object, dependency, test and core files

clean:
	rm -f *~ *.o *.d $(TESTPROG) core core.*



mrc: $(OBS)  ../librobot.a
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) $(OBS)  $(LIBS)

smrdemosim: $(OBS) ../librobot.a  ../smrsim/smrsim.o
	$(CC)  $(CFLAGS) $(LFLAGS) -o $(@) $(OBS)  $(SIMLIBS)




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
