/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <string.h>

#include <ugen4/ucommon.h>
#include "usmlstring.h"

/////////////////////////////////////////////

USmlString::USmlString()
 : USmlSource()
{
  line = 0;
  source = NULL;
  fullSource = NULL;
  verbose = false;
}

/////////////////////////////////////////////

USmlString::~USmlString()
{
}

/////////////////////////////////////////////

int USmlString::getMoreData(char * buffer, int bufferSize, int pollTimeoutMs)
{
  int n = 0;
  const char * p1;
  if (source != NULL)
  {
    p1 = strchr(source, '\n');
    if (p1 == NULL)
    {
      n = strlen(source);
      p1 = &source[n];
    }
    else
      // advance past new-line
      p1++;
    // get length of line - including new-line
    n = mini(bufferSize - 1, p1 - source);
    // copy to buffer
    strncpy(buffer, source, n);
    // terminate
    buffer[n] = '\0';
    if (n == p1 - source)
      // a full line is detected
      line++;
    // advance source pointer
    source = p1;
  }
  return n;
}

///////////////////////////////////////////

bool USmlString::doVerboseMessages()
{
  return verbose;
}

///////////////////////////////////////////

bool USmlString::isSourceAvailable()
{
  bool result = false;
  if (source != NULL)
    // source available until end of string
    result = (*source != '\0');
  return result;
}

///////////////////////////////////////////

bool USmlString::setSourceString(const char * editStr)
{
  fullSource = editStr;
  source = editStr;
  line = 0;
  return true;
}

///////////////////////////////////////////////////////

void USmlString::syntaxError(const char * message)
{
  const int MSL = 300;
  char s[MSL];
  //
  snprintf(s, MSL, "line %d: %s", line, message);
  USmlSource::syntaxError(s);
}

///////////////////////////////////////////////////////


