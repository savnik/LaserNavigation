/***************************************************************************
 *   Copyright (C) 2009 by Christian,,,   *
 *   chr@saturn   *
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
#include <cstdlib>
#include <math.h>
#include <string.h>
#include <stdio.h>


using namespace std;



// convert function
/**
 * Open source and destination files */
bool openSourceFile(const char * src, FILE ** fs);
/**
 * Open source and destination files */
bool openDestFile(const char * dst, FILE ** fd);
/**
 * make modified file with modified x (and y) values */
bool processFiles(FILE * fs, FILE * fd, double xoffset, double yoffset);

//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions

/// zone 32 UTM offset for pometet i Tåstrup
# define OX 708000.0             // UTM offset in x direction
# define OY 6174000.0            // UTM offset in y direction

void printHelp()
{ // print help
  printf("\nUsage: mapoffset [-h] [sourcefile [destinationFile [utmPoseFile]]] [-xX] [-yY]\n");
  printf("Offsets XML attributes with name x by adding the X value, and offsetting attributes named y by adding the Y value\n");
  printf("Sourcefile is e.g. pometet_map.xml\n");
  printf("Option -h          prints this help text\n\n");
  printf("Option -xX         source file attributes like x=\"-137.45\" are offset by adding the value X\n");
  printf("Option -yY         source file attributes like y=\"122.00\" are offset by adding the value Y\n");
  printf("All other values and attributes are left as is in the source file.\n");
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// main
int main(int argc, char *argv[])
{
  int i;
  const char * src = NULL;
  const char * dst = NULL;
  bool isHelp = false;
  char * p1;
  FILE *fs = NULL, *fd = NULL;
  bool isOK = false;
  double xoffset = 0.0;
  double yoffset = 0.0;
  //
  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] > ' ' and argv[i][0] != '-')
    { // assume a legal argument
      if (src == NULL)
      { // first argument is source
        src = argv[i];
      }
      else if (dst == NULL)
      { // destination filename
        dst = argv[i];
      }
      else
        printf("Too many non-option arguments 1:%s, 2:%s 3:%s\n", src, dst, argv[i]);
    }
    else if (argv[i][0] > ' ')
    {
      p1 = &argv[i][1];
      if (*p1 == 'x')
        xoffset = strtof(++p1, NULL);
      else if (*p1 == 'y')
        yoffset = strtof(++p1, NULL);
      else
      {
        isHelp = true;
        printf("Unknown option %s\n\n", p1);
      }
    }
  }
  if (isHelp)
  { // print help
    printHelp();
  }
  else
  { // do some conversion
    isOK = openSourceFile(src, &fs);
    if (isOK)
    { // source and destination file is OK
      isOK = openDestFile(dst, &fd);
    }
    if (isOK)
    {
      isOK = processFiles(fs, fd, xoffset, yoffset);
    }
    if (fs != NULL)
      fclose(fs);
    if (fd != NULL)
      fclose(fd);
  }
  if (isOK)
    printf("Finished OK\n");
  else
    printf("Failed\n");
  //

  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

bool openSourceFile(const char * src, FILE ** fs)
{
  const char * dsrc = "utmPose.log";
  const char * sfn = src;
  bool result = false;
  //
  if (sfn == NULL)
    sfn = dsrc;
  *fs = fopen(sfn, "r");
  if (*fs == NULL)
    printf("sourcefile not found: %s\n", src);
  else
  {
    result = true;
    printf("Reading from file '%s'\n", sfn);
  }
  return result;
}

/////////////////////////////////////////////////

bool openDestFile(const char * dst, FILE ** fd)
{
  const char * ddest = "utmPose.kml";
  const char * dfn = dst;
  bool result = false;
  //
  if (dfn == NULL)
    dfn = ddest;
  *fd = fopen(dfn, "w");
  if (*fd == NULL)
    printf("Could not create destination file: %s\n", dfn);
  else
  {
    result = true;
    printf("Created destination file '%s'\n", dfn);
  }
  return result;
}

/////////////////////////////////////////////////////////

bool processFiles(FILE * fs, FILE * fd, double xoffset, double yoffset)
{
  const int MSL = 10000;
  char s[MSL];
  char *p0, *p1, *p1x, *p1y, *p2;
  double v, ofs = 0.0;
  bool note = false;
  //
  while (not feof(fs))
  {
    p0 = fgets(s, MSL, fs);
    if (p0 == NULL)
      break;
    if (not note and p0[0] == '<' and p0[1] >= 'a')
    { // write note
      fprintf(fd, "<!-- offset by x+=%.3f and y+=%.3f by mapoffset utility -->\n", xoffset, yoffset);
      note = true;
    }
    while (strlen(p0) > 0)
    {
      p1x = strstr(p0, "x=");
      p1y = strstr(p0, "y=");
      if (p1x == NULL)
        p1 = p1y;
      else if (p1y == NULL)
        p1 = p1x;
      else if (p1x < p1y)
        p1 = p1x;
      else
        p1 = p1y;
      // get right offset
      if (p1 == p1x)
        ofs = xoffset;
      if (p1 == p1y)
        ofs = yoffset;
      //
      if (p1 != NULL)
      { // replace value
        p1 += 2;
        *p1++ = '\0';
        v = strtod(p1, &p2);
        fprintf(fd, "%s\"%.3f", p0, v + ofs);
        p0 = p2;
      }
      else
      { // just copy
        fprintf(fd, "%s", p0);
        break;
      }
    }
  }
  return true;
}
