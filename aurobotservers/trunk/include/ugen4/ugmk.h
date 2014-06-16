/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UGMK_H
#define UGMK_H

#include "uposrot.h"

/**
Holds information of a guidemark
@author Christian Andersen
*/

class UGmk : public UPosRot
{
public:
  /**
  Constructor */
  UGmk();
  /**
  Destructor */
  ~UGmk();
  /**
  Clear guidemark */
  void clear();
  /**
  Set code */
  void set(const char * newCode);
  /**
  Set time */
  inline void setTime(UTime atTime)
  { time = atTime; };
  /**
  Convert a string code to a long unsigned integer.
  \param newCode is a string with the values of the 4-bit codes - up to max length of square code block
  \param okCrc is set to true, if the 8-bit code CRC is validated successfut
  \returns the gmk code as a signed long integer (no test for overflow), if crc fails, then the value includes
  the full code (to be compatible with old function) */
  static unsigned long getCodeInt(const char * newCode, bool * okCrc = NULL);
  /**
  Get the integer (unsigned long) code value */
  inline unsigned long getCodeInt()
  {  return codeInt; };
  /**
  Get long code string */
  inline const char * getCode()
  { return code; };
  /**
  Set time and position */
  void set(UPosition pos, URotation rot);
  /**
  Get gmk time */
  inline UTime getTime()
  { return time; };
  /**
  Get valid flag (for code) */
  inline bool isValid()
  { return valid; };
  /**
  Get valid flag (for code) */
  inline bool isCrcValid()
  { return crcOK; };
  /**
  Get is position valid flag */
  inline bool isPosValid()
  { return validPos; };
  /**
  Set is position valid flag */
  inline void setPosValid(bool value)
  { validPos = value; };

public:
  /**
  Max code length */
  static const int MAX_CODE_LENGTH = 30;

protected:
  /**
  Full code as found in image */
  char code[MAX_CODE_LENGTH];
  /**
  Short version of code - first 8 values */
  unsigned long codeInt;
  /**
  Time for guidemark position position */
  UTime time;
  /**
  Validity of position information. true (1) if valid. */
  bool validPos;
  /**
  Validity of code data. */
  bool valid;
  /**
   * CRC is OK */
  bool crcOK;
};

#endif
