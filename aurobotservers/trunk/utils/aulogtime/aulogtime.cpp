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
#include <ugen4/ucommon.h>
#include <ugen4/utime.h>
#include <math.h>
#include <string.h>

using namespace std;

void listTimes(FILE * tf, const int argc, char * argv[]);
UTime copyTimes(FILE * tf, FILE * fl, UTime tLast);

// main
int main(int argc, char *argv[])
{
  int i;
  const char * dst = NULL;
  bool isHelp = false;
  FILE * tf;
  const int MSL = 500;
  char s[MSL];
  //
  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] > ' ' and argv[i][0] != '-')
    { // assume a legal argument
      { // use last name as destination filename
        dst = argv[i];
      }
    }
    else if (argv[i][0] > ' ')
    {
      isHelp = true;
    }
  }
  if (isHelp or dst == NULL)
  { // print help
    printf("Usage: aulogtime [-h] sourcefiles\n");
    printf("Makes a new logfile based on first filename, with an extra '.time' extension\n");
    printf("Lists in this file the start and end-time of each logfile - as double, and in human readable form\n");
    printf("Option -h --help    prints this help text\n");
  }
  else
  {
    snprintf(s, MSL, "%s.time", dst);
    tf = fopen(s, "w");
    if (tf != NULL)
    {
      listTimes(tf, argc, argv);
      fprintf(tf, "\n\n");
      fclose(tf);
      printf("---- saved timelist to '%s'\n", s);
    }
    else
      printf("**** failed to create destination file: '%s'\n", s);
  }
  cout << "Finished" << endl;
  return EXIT_SUCCESS;
}

//////////////////////////////////////////

void listTimes(FILE * tf, const int argc, char * argv[])
{
  FILE * fl;
  int i;
  UTime t1, t2;
  //
  for (i = 1; i < argc; i++)
  {
    if (*argv[i] != '-')
    { // not an option, so a file name
      fl = fopen(argv[i], "r");
      if (fl != NULL)
      {
        if (not feof(fl))
        {
          fprintf(tf, "file:  %s\n", argv[i]);
          printf("file:  %s\n", argv[i]);
          t2 = copyTimes(tf, fl, t1);
          if (not t2.valid)
          {
            fprintf(tf, "   ... is empty\n\n");
            printf("   ... is empty\n\n");
          }
          else
          {
            fprintf(tf, "\n");
            printf("\n");
            t1 = t2;
          }
        }
      }
    }
  }
}


UTime copyTimes(FILE * tf, FILE * fl, UTime tLast)
{
  int i;
  const int MSL = 10000;
  char s[MSL];
  UTime t1, t2, t;
  bool hasFirst = false;
  bool isOK;
  double d;
  char * p1;
  //
  i = 0;
  while (not feof(fl))
  {
    p1 = fgets(s, MSL, fl);
    if (not hasFirst)
    {
      t1.setTimeTod(s);
      hasFirst = t1.valid and t1.getSec() > 1e9;
      if (hasFirst)
      { // found first timestamp
        if (tLast.valid)
        { // pause without logfile
          d = t1 - tLast;
          fprintf(tf, "pause: %.1fsec  (%d:%02d min)\n", d, int(d/60.0), roundi(d - int(d/60.0)*60.0));
          printf("pause: %.1fsec  (%d:%02d min)\n", d, int(d/60.0), roundi(d - int(d/60.0)*60.0));
        }
        fprintf(tf, "start: %lu.%06lu %s\n", t1.getSec(), t1.getMicrosec(),
                t1.getDateTimeAsString(s, true));
        printf("start: %lu.%06lu %s\n", t1.getSec(), t1.getMicrosec(),
                t1.getDateTimeAsString(s, true));
        t2 = t1;
      }
    }
    else
    {
      t.setTimeTod(s);
      if (t.valid and t.getSec() > 1e9)
      {
        if (t2 - t > 15.0)
        { // pause inside logfile
          fprintf(tf, "pause: %.1fsec line %d\n", t2 - t, i);
          printf("pause: %.1fsec line %d\n", t2 - t, i);
        }
        t2 = t;
      }
    }
    i++;
  }
  isOK =  t1.valid and t2.valid;
  if (isOK)
  {
    fprintf(tf, "end:   %lu.%06lu %s %d lines\n", t2.getSec(), t2.getMicrosec(),
          t2.getDateTimeAsString(s, true), i);
    printf("end:   %lu.%06lu %s %d lines\n", t2.getSec(), t2.getMicrosec(),
            t2.getDateTimeAsString(s, true), i);
    d = t2 - t1;
    fprintf(tf, "lasts: %.1fsec  (%d:%02d min)\n", d, int(d/60.0), roundi(d - int(d/60.0)*60.0));
    printf("lasts: %.1fsec  (%d:%02d min)\n", d, int(d/60.0), roundi(d - int(d/60.0)*60.0));
  }
  return t2;
}
