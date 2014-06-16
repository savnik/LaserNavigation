/***************************************************************************
 *   Copyright (C) 2008 by Lars Valdemar Mogensen   *
 *   rse@oersted.dtu.dk   *
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



#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "rangedata.h"
#include "auextractfeatures.h"

#include "auline.h"
#include "aucircle.h"

/* This test program runs the extractfeature math on a file saved by scansave */

#define TRUE 1
#define FALSE 0

using namespace std;

// beware, unsafe function:
void loadbearings(FILE *f, RangeData *rd) 
{
  char c;
  char buffer[50];
  int pi = -1;  // ignore first value
  int bi = 0;
  char instring;
 
  instring = 0;

  c = fgetc(f);
  while ((c != '\n') && !feof(f)) {
    if (instring) {
      if ((c == '\n') || (c == ' ')) {
        // string finished
        if (pi<0) {
          pi++;
        }
        else {
          buffer[bi++] = 0;
          rd->point_th[pi++] = atof(buffer);
        }
        bi = 0;
        instring = FALSE;
      }
      else {
        // continue assembling string
        buffer[bi++] = c;
      }
    }
    else {
      if ((c == '\n') || (c == ' ')) {
        // still not instring
      }
      else {
        // start assembling string
        buffer[bi++] = c;
        instring = TRUE;
      }
    }
    c = fgetc(f);
  }
  
  rd->points = pi;
  
}

// beware, unsafe function:
int loadscan(FILE *f, RangeData *rd, double *ts) 
{
  char c;
  char buffer[50];
  int bi = 0;
  int pi = -1; // first value is timestamp
  char instring;
  
  instring = 0;

  c = fgetc(f);
  while ((c != '\n') && !feof(f)) {
    if (instring) {
      if ((c == '\n') || (c == ' ')) {
        // string finished
        if (pi<0) {
          // timestamp
          *ts = atof(buffer);
          pi++;
        }
        else {
          buffer[bi++] = 0;
          rd->point_r[pi++] = atof(buffer);
        }
        bi = 0;
        instring = FALSE;
      }
      else {
        // continue assembling string
        buffer[bi++] = c;
      }
    }
    else {
      if ((c == '\n') || (c == ' ')) {
        // still not instring
      }
      else {
        // start assembling string
        buffer[bi++] = c;
        instring = TRUE;
      }
    }
    c = fgetc(f);
  }
  
  return pi;
}

void simulate(FILE *logfile, FILE *datafile)
{
  RangeData rd;
  AUExtractFeatures ef;
  double ts; // timestamp
  int segs;
  const int MRL = 100;
  char reply[MRL];
  
  // Load bearing values
  loadbearings(datafile, &rd);
  
  while (loadscan(datafile, &rd, &ts) == rd.points) {
    // We have a scan, process it:
    ef.extractFeatures(&rd);
          
    segs = ef.getSegmentCount();
      
    snprintf(reply, MRL, "<sf timestamp=\"%f\">\n", ts);
    fprintf(logfile, "%s", reply);
    
    for (int i=0; i<segs; i++) {
      // ef.segmentToStringPolar(i, reply);
      ef.segmentToStringCart(i, reply);
      fprintf(logfile, "%s", reply);
    }
    
    fprintf(logfile, "</sf>\n");
  }
  
}

int main (int argc, char *argv[])
{
  FILE *logfile;
  FILE *datafile;
  AULine line;
  AUCircle circle;
  AUEFReturnStruct res;
  
  if ((argc != 2) && (argc != 3)) {
    printf("Testsf. Scanfeatures test program.\n\n");
    printf("Syntax: testsf <datafile> [<outputfile>]\n\n");
    printf("<datafile>   must be a file created by scansave\n");
    printf("<outputfile> will be created with feature extraction result in XML. \n");
    printf("Default outputfile is testsf.xml\n\n");
    printf("\n\n\n");
    line.print();
    circle.print();
    res.print();
    
    //printf("\n\n");
    //printf("%d %s\n",line.r0,line.cartValid);
    //cout << line.r0 << boolalpha << line.cartValid << endl;
    return 0;
  }
  
  // Open input data file
  datafile = fopen(argv[1],"r");
  
  if (datafile == NULL) {
    printf("ERROR: Could not open datafile: %s\n",argv[1]);
    return 0;
  }

  // Open output data file
  if (argc == 3 )
    logfile = fopen(argv[2],"w");
  else
    logfile = fopen("testsf.xml","w");
  
  if (logfile == NULL) {
    printf("ERROR: Could not open output file\n");
    fclose(datafile);
    return 0;
  }
  
  // Do it
  fprintf(logfile,"<sfroot>\n");
  simulate(logfile, datafile);
  fprintf(logfile,"</sfroot>\n");
  
  fclose(logfile);
  fclose(datafile);
  
}
