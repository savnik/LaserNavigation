#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "xfigreader.h"

#define MAXCHARS 1000

void xfr_readcomments(FILE *fp) {
  int c = ' ', comment = 1;
  char line[MAXCHARS];

  while (comment) {
    while ((c == ' ') || (c == '\t')) {
      c = fgetc(fp);
    }
    if (c != '#') {
      ungetc(c,fp);
      comment = 0;
    } else {
      fgets(line,MAXCHARS,fp);
      c = ' ';
      comment = 1;
    }
  }
}

void xfr_parsecolor(FILE *fp, char *startline) {
  // Do nothing - all data is on startline and i automatically removed

}

void xfr_parsearc(FILE *fp, char *startline) {
  int forwardarrow, backwardarrow;
  char line[MAXCHARS];
  sscanf(startline,"%*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*f %*d %*d %d %d",
	 &forwardarrow,&backwardarrow);
  if (forwardarrow) {
    xfr_readcomments(fp);
    fgets(line,MAXCHARS,fp);
  }
  if (backwardarrow) {
    xfr_readcomments(fp);
    fgets(line,MAXCHARS,fp);
  }
}

void xfr_parseellipse(FILE *fp, char *startline) {
  // Do nothing - all data is on startline and i automatically removed
  
}

void xfr_parsespline(FILE *fp, char *startline) {
  int forwardarrow, backwardarrow, npoints, readpoints = 0;
  char line[MAXCHARS];
  char *token;
  char *tokens = (char *) malloc(MAXCHARS);
  char delim[] = " \t";

  sscanf(startline,"%*d %*d %*d %*d %*d %*d %*d %*d %*d %*f %*d %d %d %d",
	 &forwardarrow,&backwardarrow,&npoints);
  if (forwardarrow) {
    xfr_readcomments(fp);
    fgets(line,MAXCHARS,fp);
  }
  if (backwardarrow) {
    xfr_readcomments(fp);
    fgets(line,MAXCHARS,fp);
  }
  while (readpoints < npoints*3) {
    xfr_readcomments(fp);
    fgets(line,MAXCHARS,fp);
    tokens = line;
    while ((token = strsep(&tokens,delim))) {
      readpoints++;
    }
  }
}

void xfr_parsetext(FILE *fp, char *startline) {
  // Do nothing - all data is on startline and i automatically removed

}

void xfr_parsecompound(FILE *fp, char *startline) {
  fprintf(stderr,"Error: compound objects not supported\n");
  exit(1);
}

polylinetype xfr_parsepolyline(FILE *fp, char *startline) {
  int i,n,npoints,readpoints = 0, subtype;
  double resolution = 1200.0;
  char currentline[MAXCHARS];
  char *tokens;
  char *token;
  char delim[] = " \t";
  polylinetype p = NULL;

  tokens = startline;

  // Get subtype
  token = strsep(&tokens,delim);
  token = strsep(&tokens,delim);
  subtype = atoi(token);
  // Skip unimportant info (line style etc)
  for (i = 0; i < 11; i++)
    token = strsep(&tokens,delim);
  // Read arrow information and find number of arrow lines to skip
  n = 0;
  for (i = 0; i < 2; i++) {
    token = strsep(&tokens,delim);
    if (atoi(token) != 0)
      n++;
  }
  token = strsep(&tokens,delim);
  npoints = atoi(token);

  // Skip any arrowlines
  for (i = 0; i < n; i++) {
    xfr_readcomments(fp);
    fgets(currentline,MAXCHARS,fp);
  }

  // Read points lines and add points to polyline
  while (readpoints < npoints) {
    int x,y;
    xfr_readcomments(fp);
    fgets(currentline,MAXCHARS,fp);
    tokens = currentline;
    while ((token = strsep(&tokens,delim))) {
      x = atoi(token);
      token = strsep(&tokens,delim);
      y = atoi(token);
      polyline_addpoint(&p,x/resolution,-y/resolution);
      readpoints++;
    }
  }
  p->subtype = subtype;
  if (subtype == PLT_PICTUREBOX) {
    xfr_readcomments(fp);
    fgets(currentline,MAXCHARS,fp);
  }
  if ((subtype == PLT_POLYLINE) || (subtype == PLT_POLYGON))
    return p;
  else 
    return NULL;
}

polylinestype xfr_parsexfig(char *filename) {
  FILE *fp;
  int i;
  char currentline[MAXCHARS];
  char *token;
  polylinetype p = NULL;
  polylinestype lines = polyline_new();

  // Open XFIG file
  if (!(fp = fopen(filename,"rb"))) {
    fprintf(stderr,"xfr_init: unable to open file %s\n",filename);
    exit(1);
  }

  // Read XFIG header and verify it
  // Does not check version number!
  fgets(currentline,MAXCHARS,fp);
  i = 0;
  while ((currentline[i] == ' ') ||
	 (currentline[i] == '\t')) {
    i++;
  }
  token = (char *) malloc(4 * sizeof(char));
  strncpy(token,currentline+i+1,3);
  token[3] = '\0';

  if ((currentline[i] != '#') || !(strcmp(token,"FIG") == 0)) {
    fprintf(stderr, "Error: unable to find XFIG header. Found %s in %s\n",token,currentline);
    exit(1);
  }
  free(token);

  // Read global xfig options, not important to us
  for (i = 0; i < 8; i++) {
    xfr_readcomments(fp);
    fgets(currentline,MAXCHARS,fp);
  }

  while (1) {
    // Read comments...
    xfr_readcomments(fp);
    // Read first line of first object
    fgets(currentline,MAXCHARS,fp);
    //    tokens = currentline;

    if (feof(fp))
      break;

    switch(currentline[0]) {
    case '0': // Color object
      //      printf("Color found\n");
      xfr_parsecolor(fp,currentline);
      break;
    case '1': // Ellipse object
      //      printf("Ellipse found\n");
      xfr_parseellipse(fp,currentline);
      break;
    case '2': // Polyline object
      //      printf("Polyline found\n");
      if ((p = xfr_parsepolyline(fp,currentline))) {
	polyline_addline(lines,p);
      }
      break;
    case '3': // Spline object
      //      printf("Spline found\n");
      xfr_parsespline(fp,currentline);
      break;
    case '4': // Text object
      //      printf("Text found\n");
      xfr_parsetext(fp,currentline);
      break;
    case '5': // Arc object
      //      printf("Arc found\n");
      xfr_parsearc(fp,currentline);
      break;
    case '6': // Compound object
      //      printf("Compound found\n");
      xfr_parsecompound(fp,currentline);
      break;
    default:
      break;
    }
  }
  fclose(fp);
  return lines;
}

polylinestype xfr_init(char *filename) {
  polylinestype tmp = xfr_parsexfig(filename);

  return tmp;
}

