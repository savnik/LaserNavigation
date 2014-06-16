// $Id: polyline.h,v 1.1.1.1 2005/11/22 21:39:58 naa Exp $
#ifndef POLYLINE_H
#define POLYLINE_H

enum polylinetype {
  PLT_NONE = 0,
  PLT_POLYLINE,
  PLT_BOX,
  PLT_POLYGON,
  PLT_ARCBOX,
  PLT_PICTUREBOX
};

typedef struct polyline * polylinetype;
struct polyline {
  int numpoints, maxpoints;
  enum polylinetype subtype;
  double *x,*y,*ex,*ey,*l,*c;
};

typedef struct polylines * polylinestype;
struct polylines {
  int numlines, maxlines;
  polylinetype *lines;
};


void polyline_addpoint(polylinetype *p, double x, double y);
void polyline_addline(polylinestype p, polylinetype line);
void polyline_print(polylinestype lines);
void polyline_free(polylinestype *lines);
polylinestype polyline_new();


#endif
