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
#include <cstdlib>
#include <limits.h>
#include <string.h>
#include <stdio.h>

using namespace std;

/**
 * Open source and destination files */
bool openSourceFile(const char * src, FILE ** fs);
/**
 * Open source and destination files */
bool openDestFile(const char * dst, FILE ** fd);
/**
 * make kml file format and process all lines in source file */
bool processFiles(FILE * fs, FILE * fd,
                  unsigned int minLng,  int secStart, int secEnd);

/////////////////////////////////////

int main(int argc, char *argv[])
{
  int i;
  const char * src = NULL;
  const char * dst = NULL;
  bool isHelp = false;
  char * p1;
  FILE *fs = NULL, *fd = NULL;
  bool isOK = false;
  unsigned int minLng = 43;
  int secStart =0, secEnd = INT_MAX;
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
    }
    else if (argv[i][0] > ' ')
    {
      p1 = &argv[i][1];
      if (*p1 == 's' and strlen(p1) > 8)
        secStart = strtol(&p1[1], NULL, 0);
      else if (*p1 == 'e' and strlen(p1) > 8)
        secEnd = strtol(&p1[1], NULL, 0);
      else if (*p1 == 'h')
        isHelp = true;
      else if (*p1 == '-')
        isHelp = true;
      else
      {
        isHelp = true;
        printf("Unknown option %s\n\n", p1);
      }
    }
  }
  if (isHelp)
  { /* print help */
    printf("\nUtility to strip short lines from a logfile\n");
    printf("Use:\n");
    printf(" %s [-lN] [-h] sourcefile destinationfile\n", argv[0]);
    printf(" where N (default=%d) is minimum length of a line in the destinationfile\n\n", minLng);
  }
  else
  { // do some conversion
    isOK = openSourceFile(src, &fs);
    if (isOK)
    { // source and destination file is OK
      isOK = openDestFile(dst, &fd);
      if (isOK)
        isOK = processFiles(fs, fd, minLng, secStart, secEnd);
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


bool openSourceFile(const char * src, FILE ** fs)
{
  const char * dsrc = "smrcl.log";
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
  const char * ddest = "smrcl-part.log";
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

bool processFiles(FILE * fs, FILE * fd,
                  unsigned int minLng,  int secStart, int secEnd)
{
  const int MSL = 10000;
  char s[MSL];
  char *p2;
  const char * p1;
  bool started = secStart < 1234567890;
  int t;
  int n = 0;
  bool inc;
  //
  while (not feof(fs))
  {
    p1 = fgets(s, MSL, fs);
    inc = strlen(s) > minLng;
    if (not inc)
      inc = strstr(s, "userevent") > NULL;
    if (not inc)
      inc = strstr(s, "syntaxerror") > NULL;
    if (inc)
    {
      t = strtol(s, &p2, 0);
      if (p2 != s)
      {
        if (not started and t > secStart)
          started = true;
        if (started and t > secEnd)
          break;
      }
      if (started)
      {
        fprintf(fd, "%s", s);
        n++;
      }
    }
  }
  printf("wrote %d lines\n", n);
  return n > 0;
}

