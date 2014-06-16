/***************************************************************************
 *   Copyright (C) 2009 by Christian Andersen   *
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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


using namespace std;

bool logRename(const char * name);

int main(int argc, char *argv[])
{
  int result = 0;
  int i;
  bool ask4help;
  //
  ask4help = (argc == 1);
  for (i = 1; i < argc; i++)
  { // all options result in help
    if (*argv[i] == '-')
      ask4help = true;
  }
  if (ask4help)
  {
    printf("\n The AU Log Rotate is used to rename a (log) file\n");
    printf("   It will append the modified date and time (from stat mtime) to the filename\n");
    printf("   e.g.:\n\n");
    printf("   $ aulogrotate svs.log\n");
    printf("   $ aulogrotate abc\n");
    printf("   $ aulogrotate abc.dat\n");
    printf("   $ aulogrotate *.log\n\n");
    printf("   the first will rename the file to something like svs20090522_165922.logg\n");
    printf("   the 2nd will rename the file to something like abc20090522_165924.logg\n");
    printf("   the 3rd will rename the file to something like abc20090522_165926.datg\n");
    printf("   the third will rename all the matching files and new names end with an extra g\n");
    printf("   Returns 0 on succes (on all files)\n");
  }
  else
  {
    for (i = 1; i < argc; i++)
    {
      if (*argv[i] != '-')
        result += logRename(argv[i]);
    }
  }
  return result;
}

//////////////////////////////////

bool logRename(const char * name)
{ /// make log-rotate here
  struct stat attrib;                   // create a file attribute structure
  struct tm* clock;                     // create a time structure
  const int MFL = 1000;
  char f[MFL];
  int err;
  int y, mdr, d, h, min, n, sec;
  const char * p1;
  //
  //
  err = stat(name, &attrib);      // get the attributes of afile.txt
  if (err == 0)
  { // Get the last modified time and put it into the time structure
    //clock = gmtime(&(attrib.st_ctime));
    // // Get the last modified time and put it into the time structure
    clock = localtime(&(attrib.st_mtime));
    y   = clock->tm_year + 1900; // returns the year (since 1900)
    mdr = clock->tm_mon + 1; // returns the month (January = 0)
    d   = clock->tm_mday; // returns the day of the month
    h   = clock->tm_hour;
    min = clock->tm_min;
    sec = clock->tm_sec;
    p1 = strrchr(name, '.');
    if (p1 == NULL)
      // no extension, so add also an extension '.logg'
      snprintf(f, MFL, "mv %s %s%d%02d%02d_%02d%02d%02d.logg", name, name, y, mdr, d, h, min, sec);
    else
    { // insert time and add a 'g'
      snprintf(f, MFL, "mv %s ", name);
      n = strlen(f);
      strncat(f, name, MFL-1);
      n += p1 - name;
      snprintf(&f[n], MFL, "%d%02d%02d_%02d%02d%02d%sg", y, mdr, d, h, min, sec, p1);
    }
    err = system(f);
  }
  //
  return err;
}
