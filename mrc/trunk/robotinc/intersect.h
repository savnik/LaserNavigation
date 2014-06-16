#ifndef INTERSECT_H
#define INTERSECT_H

#include "smrsim.h"
#include "polyline.h"

#ifndef BACKGROUNDCOLOUR
#define BACKGROUNDCOLOUR 255
#endif

#ifndef FOREGROUNDCOLOUR
#define FOREGROUNDCOLOUR 0
#endif

#ifndef MAXINTERSECTIONS
#define MAXINTERSECTIONS 8
#endif

#ifndef INTERSECTTOLERANCE
#define INTERSECTTOLERANCE 1e-6
#endif

typedef struct {
  int intersections; // How many intersections were found
  int types[MAXINTERSECTIONS]; // Types of intersections (1=intersection,2=coincident)
  int lines[MAXINTERSECTIONS]; // What line numbers in approriate polylinestype are intersected
  int points[MAXINTERSECTIONS]; // For each line, which segment number (ie points[i] - points[i+1]
  double c[MAXINTERSECTIONS]; // Coefficients for linesegments (only if types[i] == I_CROSS)
  double lc[MAXINTERSECTIONS]; // Coefficients for linesensor
  double angles[MAXINTERSECTIONS]; // Angle between linesensor and line (currently unused
} intersectionstype;

enum intersectiontype {
  I_NONE = 0,
  I_CROSS,
  I_COMPLETE,
  I_LEFTEDGE,
  I_RIGHTEDGE,
  I_WITHIN
};

/* linesensordist: distance from pose reference point to linesensor
 * linesensorwidth: width of the linesensor, assumes symmetrical sensor
 * *line: first line number in lines that intersects with linesensor
 * *point: first point in *line that intersects
 * *coef: intersection is at coef * ((*point + 1) - (*point))
 * linesensor[] is filled with BACKGROUNDCOLOUR or FOREGROUNDCOLOR depending
 * on wether there is a line beneath each sensor
 * Return value: 2 if lines are coincident and linesegments are overlapping
 * 1 if intersection is found, 0 otherwise
 * if 0 is returned, *line, *point and *coef are NULL.
 */
int intersect_linesensor(polylinestype lines, posetype pose,
			 double linesensordist, double linesensorwidth,
			 intersectionstype *is, int linesensor[SMR_LS_N]);


#endif
