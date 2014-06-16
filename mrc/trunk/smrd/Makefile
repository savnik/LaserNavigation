
CC      = gcc -pthread
LD      = ${CC}
CFLAGS  = -Wall -O2 -I../include

HDRS   = ../include/smr.h
OBJS   = smrd.o

smrd: ${OBJS}
	${LD} -o ${@} ${OBJS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
