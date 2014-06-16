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
#ifndef UDATABASE_H
#define UDATABASE_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/**
* A base class for data, that allows data type checking using one virtual method, but otherwise has no data content.
* Is intended as base class for all datatypes that may be returned by a var-pool method.

	@author Christian Andersen <chr@oersted.dtu.dk>
*/
class UDataBase
{
public:
  /**
  Constructor */
  UDataBase() {;};
  /**
  Destructor */
  virtual ~UDataBase() {;};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "empty";
  };
  /**
  Function to test if the class is of a specific type */
  inline bool isA(const char * typeString)
  {
    return (strcmp(getDataType(), typeString) == 0);
  };
  /**
  Function to test if the class or one of its ancestors is of a specific type */
  virtual bool isAlsoA(const char * typeString);
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt)
  {
    snprintf(buff, buffCnt, "%s a %s\n", preString, getDataType());
  };

};

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

class UDataString : public UDataBase
{
public:
  /**
  Constructor */
  UDataString();
  /**
  Destructor */
  virtual ~UDataString();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "string";
  };
  /**
  Function to test if the class or one of its ancestors is of a specific type */
  virtual bool isAlsoA(const char * typeString);
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt)
  {
    snprintf(buff, buffCnt, "%s %s\n", preString, str);
  };
  /**
  Set string buffer */
  inline void setStr(char * toStr, const int toStrCnt)
  {
    if (strAlloc)
      free(str);
    str = toStr;
    strCnt = toStrCnt;
    strAlloc = false;
  }
  /**
  Get string (buffer) length */
  inline int getStrCnt()
  { return strCnt; };
  /**
  Get string (buffer) - may be NULL. */
  inline char * getStr()
  { return str; };
  /**
  Allocate string space on heap.
  Returns pointer to new string.
  The string will be freed when a new string is allocated or assigned. */
  char * makeString(int length);
  /**
  Allocate string space on heap and copy the source string data.
  Returns pointer to new string.
  The string will be freed when a new string is allocated or assigned. */
  char * makeString(const char * source);

protected:
  /**
  Pointer to a string buffer */
  char * str;
  /**
  Length of string buffer */
  int strCnt;
  /**
  Is string allocated - and thus must be freed*/
  bool strAlloc;
};

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

class UDataDouble : public UDataBase
{
  public:
  /**
    Constructor - initializes the value to 0 */
    UDataDouble()
    {
      value = 0;
    };
  /**
    Destructor */
    virtual ~UDataDouble(){};
  /**
    Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "double";
    };
  /**
    Print status for this structure */
    virtual void snprint(const char * preString, char * buff, const int buffCnt)
    {
      snprintf(buff, buffCnt, "%s %f\n", preString, value);
    };
  /**
    Set double value */
    inline void setVal(double in)
    {
      value = in;
    }
  /**
    Get double value */
    inline double getVal()
    { return value; };

  protected:
  /**
    Double value */
    double value;

};

#endif
