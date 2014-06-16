/***************************************************************************
                          urobformat.h  -  description
                             -------------------
    begin                : Thu Jun 5 2003
    copyright            : (C) 2003 by (Jens) Christian Andersen,326/030,3548,
    email                : jca@iau.dtu.dk
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef UROBFORMAT_H
#define UROBFORMAT_H


//#include "../../gen/gen/utime.h"
#include <sys/time.h>

/**
  *@author (Jens) Christian Andersen,326/030,3548,
  */

// Suppport functions for communication

/**
Convert unsigned char array to long integer. first byte is LSB. */
extern unsigned long getLong(unsigned char * buf, int length);

/**
Convert unsigned char array to long integer. first byte is LSB. */
extern long getLongSigned(unsigned char * buf, int length);

/**
Convert unsigned long integer to char buffer. first byte is LSB. */
extern int setLong(unsigned char * buf, int length, unsigned long value);

/**
Convert time to unsigned char byte buffer.
Returns used number of bytes. */
extern int setTime(unsigned char * buf, struct timeval time);

/**
Get time type from unsigned char buffer.
Returns the time structure. */
extern struct timeval getTime(unsigned char * buf);


#endif
