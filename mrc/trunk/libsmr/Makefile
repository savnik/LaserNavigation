CC     = gcc
CFLAGS = -Wall -O2 -I../include

HDRS   = ../include/smr.h
OBJS   = libsmr.o

all:	libsmr.a(libsmr.o)

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
