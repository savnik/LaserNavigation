/***************************************************************************
                          urobformat.cpp  -  description
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

#include "urobformat.h"

unsigned long getLong(unsigned char * buf, int length)
{
  int i;
  unsigned long result = 0;
  //
  for (i = length - 1; i >= 0; i--)
    result = (result << 8) + buf[i];
  //
  return result;
}

/**
Convert unsigned char array to long integer. first byte is LSB. */
long getLongSigned(unsigned char * buf, int length)
{
  int i;
  long result;
  // copy sign and first byte
  result = (char)buf[length - 1];
  for (i = length - 2; i >= 0; i--)
    result = (result << 8) + buf[i];
  //
  return result;
}

/**
Convert unsigned long integer to char buffer. first byte is LSB. */
int setLong(unsigned char * buf, int length, unsigned long value)
{
  int result = 1; // true;
  //
  int i;
  //
  for (i = 0; i < length; i++)
  {
    buf[i] = (unsigned char) value & 0xff;
    value = value >> 8;
  }
  if (value > 0)
    // there should be no bits left over
    result = 0; // false;
  //
  return result;
}

/**
Convert time to unsigned char byte buffer.
Returns used number of bytes. */
int setTime(unsigned char * buf, struct timeval time)
{ //  Convert time to unsigned char byte buffer.
  // Returns used number of bytes.
  int result = 8;
  //
  //setLong(buf    , 4, time.GetSec());      // seconds since 1 jan 1970
  //setLong(&buf[4], 4, time.GetMicrosec()); // microseconds
  setLong(buf    , 4, time.tv_sec);      // seconds since 1 jan 1970
  setLong(&buf[4], 4, time.tv_usec); // microseconds
  //
  return result;
}

/**
Get time type from unsigned char buffer.
Returns the time structure. */
struct timeval getTime(unsigned char * buf)
{ // Get time type from unsigned char buffer.
  // Returns the time structure. */
  struct timeval result;
  //
  result.tv_sec = getLong(buf     , 4);
  result.tv_usec = getLong(&buf[4], 4);
  //
  return result;
}



