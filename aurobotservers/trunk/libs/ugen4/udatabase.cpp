/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)                        *
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
#include "udatabase.h"

// UDataBase::UDataBase()
// {
// }
//
//
// UDataBase::~UDataBase()
// {
// }

bool UDataBase::isAlsoA(const char * typeString)
{
  bool result;
  result = (strcmp(UDataBase::getDataType(), typeString) == 0);
  if (not result)
      // ask parent type
    result = false; // UDataBase::isAlsoA(typeString);
  return result;
}


//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////

UDataString::UDataString()
{
  str = NULL;
  strCnt = 0;
  strAlloc = false;
}

//////////////////////////////////////////

UDataString::~UDataString()
{
  if (strAlloc) free(str);
}

//////////////////////////////////////////

char * UDataString::makeString(int length)
{
  if (strAlloc) free(str);
  str = (char *)malloc(length);
  strAlloc = (str != NULL);
  if (strAlloc)
    strCnt = length;
  else
    strCnt = 0;
  return str;
}

//////////////////////////////////////////

char * UDataString::makeString(const char * source)
{
  if (strAlloc) free(str);
  strCnt = strlen(source) + 1;
  str = (char *)malloc(strCnt);
  strAlloc = (str != NULL);
  if (strAlloc)
    strcpy(str, source);
  else
    strCnt = 0;
  return str;
}

/////////////////////////////////////////

bool UDataString::isAlsoA(const char * typeString)
{
  bool result;
  result = (strcmp(UDataString::getDataType(), typeString) == 0);
  if (not result)
      // ask the ancestor if type is known
    result = UDataBase::isAlsoA(typeString);
  return result;
}
