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

#include "usmlfile.h"


USmlFile::USmlFile()
{
  fh = NULL;
  fn[0] = '\0';
  verbose = true;
  errorBufferCnt = 0;
  line = -1;
}

///////////////////////////////////////////////

USmlFile::~USmlFile()
{
  if (fh != NULL)
    fclose(fh);
}

///////////////////////////////////////////////

int USmlFile::getMoreData(char * buffer, int bufferSize, int pollTimeoutMs)
{
  int n = 0;
  const char * p1;
  if (fh != NULL)
  {
    p1 = fgets(buffer, bufferSize, fh);
    n = strlen(buffer);
    line++;
    if ((feof(fh) and n == 0) or (p1 == NULL))
      n = -1;
  }
  return n;
}

///////////////////////////////////////////

bool USmlFile::doVerboseMessages()
{
  return verbose;
}

///////////////////////////////////////////

bool USmlFile::isSourceAvailable()
{
  return fh != NULL;
}

///////////////////////////////////////////

bool USmlFile::openSmlFile(const char * filename)
{
  const int MSL = MAX_FILENAME_LENGTH;
  char s[MSL];
  const char * f;
 // bool fullName;
  //
  if (fh != NULL)
    fclose(fh);
  strncpy(fn, filename, MAX_FILENAME_LENGTH);
  // should path be appended?
  f = getFullFilename(dataPath, filename, s, MSL);
/*  fullName = (filename[0] == '/') or
      (filename[0] == '.' and (filename[1] == '/' or filename[1] == '.'));
  //
  if (fullName)
    f = fn;
  else
  {
    snprintf(s, MSL, "%s/%s", dataPath, filename);
    f = s;
  }*/
  fh = fopen(f, "r");
  line = 0;
  return fh != NULL;
}

///////////////////////////////////////////////////////

void USmlFile::closeSmlFile()
{
  if (fh != NULL)
  {
    fclose(fh);
    fh = NULL;
    line = -1;
  }
}

///////////////////////////////////////////////////////

void USmlFile::syntaxError(const char * message)
{
  const int MSL = 300;
  char s[MSL];
  //
  snprintf(s, MSL, "line %d: %s", line, message);
  USmlSource::syntaxError(s);
}

///////////////////////////////////////////////////////

