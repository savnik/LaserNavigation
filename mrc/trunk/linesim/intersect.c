#include <stdlib.h>
#include <math.h>

#include "intersect.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#endif

/* Check if the two lines are overlapping
 * Not yet implemented!
 * P1 = linesensor midpoint
 * P2 = linesensor left point
 * P3 = Pn in polyline
 * P4 = Pn+1 in polyline
 * Return values:
 * I_NONE      : linesegment completely outside linesensor
 * I_COMPLETE  : linesegment completely covers linesensor
 * I_LEFTEDGE  : linesegment crosses left edge of linesensor
 * I_RIGHTEDGE : linesegment crosses right edge of linesensor
 * I_WITHIN    : linesegment completely within/beneath linesensot
 */
int intersect_overlap(double x1, double x2, double x3, double x4,
		      double y1, double y2, double y3, double y4,
		      double *ul, double *ur) {

  double ua,ub,lengthsqr;
  
  lengthsqr = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  ua = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / lengthsqr;
  ub = ((x4 - x1) * (x2 - x1) + (y4 - y1) * (y2 - y1)) / lengthsqr;

  if ((fabs(ua) > 1.0) && (fabs(ub) > 1.0) && (ua*ub < 0.0)) {
    // Polyline end points completely outside linesensor, so
    // left position is 1.0 and right position is -1.0
    *ul = 1.0;
    *ur = -1.0;
    return I_COMPLETE;
  } else if (((ua > 1.0) && (ub < 1.0)) || ((ua < 1.0) && (ub > 1.0))) {
    // Polyline covers part of the linesensor, including left edge
    // so left position is 1.0 and right position is whatever is smaller
    *ul = 1.0;
    *ur = (ua <= ub)?ua:ub;
    return I_LEFTEDGE;
  } else if (((ua < -1.0) && (ub > -1.0)) || ((ua > -1.0) && (ub < -1.0))) {
    // Polyline covers part of the linesensor, including right edge
    // so right position is -1.0 and left position is whatever is larger
    *ur = -1.0;
    *ul = (ua >= ub)?ua:ub;
    return I_RIGHTEDGE;
  } else if ((fabs(ua) < 1.0) && (fabs(ub) < 1.0)) {
    // Both ends of polyline is within the linesensor so left position is
    // assigned to the greater value and right position to the smaller
    *ul = (ua >= ub)?ua:ub;
    *ur = (ua < ub)?ua:ub;
    return I_WITHIN;
  } else {
    // Polyline is completely outside linesensor
    return I_NONE;
  }
  
  return -1;
}

/* Intersection algorithm from:
 * http://astronomy.swin.edu.au/~pbourke/geometry/lineline2d/
 */
int intersect_do(double x1, double x2, double x3, double x4,
		 double y1, double y2, double y3, double y4,
		 double *ua, double *ub) {
  double denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
  double nominator0,nominator1;
  int overlap;

  nominator0 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
  nominator1 = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

  if (fabs(denominator) < INTERSECTTOLERANCE) {
    //    printf("Lines are parallel\n");
    //    printf("Nom0: %f\tNom1: %f\n",nominator0,nominator1);
    // Denominator approximately 0, lines are parallel
    if ((fabs(nominator0) < INTERSECTTOLERANCE) || (fabs(nominator1) < INTERSECTTOLERANCE)) {
      //      printf("Lines are coincident\n");
      // Lines are coincident
      if ((overlap = intersect_overlap(x1,x2,x3,x4,y1,y2,y3,y4,ua,ub))) {
	// Line segments overlap, left linesensor position is stored in
	// ua, right position in ub
	return overlap;
      }
    }
  } else {
    // Lines are not parallel
    *ua = nominator0 / denominator;
    *ub = nominator1 / denominator;
    //    printf("denom: %f\n",denominator);
    //    printf("ua: %f\tub: %f\n",*ua,*ub);
    if ((fabs(*ua) <= 1.0) && (*ub >= 0.0) && (*ub <= 1.0)) {
      // We know *ua is the parameter for the linesensor vector
      return I_CROSS;
    }
  }
  // Otherwise
  return 0;
}

int intersect_linesensor(polylinestype lines, posetype pose,
			 double linesensordist, double linesensorwidth,
			 intersectionstype *is, int linesensor[SMR_LS_N]) {

  int i,j,intersecttype;
  double sensor_x,sensor_y,sensor_dx,sensor_dy,sensor_lx,sensor_ly;
  double ua,ub;
  double ls_positions[SMR_LS_N], ls_limits[SMR_LS_N+1];
  double sensor_space;

  sensor_x = pose.x + linesensordist * cos(pose.th);
  sensor_y = pose.y + linesensordist * sin(pose.th);
  sensor_dx = linesensorwidth / 2.0 * cos(pose.th + M_PI_2);
  sensor_dy = linesensorwidth / 2.0 * sin(pose.th + M_PI_2);
  sensor_lx = sensor_x + sensor_dx;
  sensor_ly = sensor_y + sensor_dy;

  // Reset output struct's counter
  is->intersections = 0;

  //  printf("Sensor: ( %f , %f ) , ( %f , %f )\n",sensor_x,sensor_y,sensor_dx,sensor_dy);
  for (i = 0; i < lines->numlines; i++) {
    if (lines->lines[i]->subtype != PLT_POLYLINE)
      break;
    for (j = 0; j < lines->lines[i]->numpoints-1; j++) {
      //      printf("Polyline %d (point %d)\n",i,j);
      intersecttype = intersect_do(sensor_x,sensor_lx,
				   lines->lines[i]->x[j],lines->lines[i]->x[j+1],
				   sensor_y,sensor_ly,
				   lines->lines[i]->y[j],lines->lines[i]->y[j+1],
				   &ua,&ub);
      // if we have an intersection, return appropriate values
      if (intersecttype) {
	//	printf("Got intersection: %d\n",intersecttype);
	is->types[is->intersections] = intersecttype;
	is->lines[is->intersections] = i;
	is->points[is->intersections] = j;
	is->lc[is->intersections] = ua;
	is->c[is->intersections] = ub;
	is->angles[is->intersections] = 0.0;
	if (++is->intersections > MAXINTERSECTIONS-1) {
	  fprintf(stderr,"intersect_linesensor: Error: too many intersections (%d) found\n",MAXINTERSECTIONS);
	  exit(1);
	}
      }
    }
  }

  // This is not optimal. There should be no need to calculate all this
  // static stuff every time we check for an intersection!
  sensor_space = linesensorwidth / (double)(SMR_LS_N - 1);
  for (i = 0; i < SMR_LS_N; i++) {
    linesensor[i] = BACKGROUNDCOLOUR;
    ls_positions[i] = linesensorwidth / 2.0 - i * sensor_space;
    ls_limits[i] = ls_positions[i] + sensor_space / 2.0;
  }
  ls_limits[SMR_LS_N] = ls_positions[SMR_LS_N - 1] - sensor_space / 2.0;


  for (i = 0; i < is->intersections; i++) {
    switch (is->types[i]) {
    case I_CROSS:
      // If the linesensor and the polyline crosses each other we
      // set the appropriate sensor value to FOREGROUNDCOLOUR
      for (j = 0; j < SMR_LS_N; j++) {
	if ((is->lc[i] * linesensorwidth / 2.0 <= ls_limits[j]) &&
	    (is->lc[i] * linesensorwidth / 2.0 >= ls_limits[j+1])) {
	  linesensor[j] = FOREGROUNDCOLOUR;
	}
      }
      break;
    case I_COMPLETE:
      // If the linesensor is completely covered by the polyline all
      // linesensor values are set to FOREGROUNDCOLOUR
      for (j = 0; j < SMR_LS_N; j++) {
	linesensor[j] = FOREGROUNDCOLOUR;
      }
      break;
    case I_LEFTEDGE:
      for (j = 0; j < SMR_LS_N; j++) {
	// "Mark" all linesensors that has a higher right limit than
	// the right position of the polyline. Note that is->c[] is used
	// as the right position of the overlapping line segment
	if (is->c[i] * linesensorwidth / 2.0 <= ls_limits[j]) {
	  linesensor[j] = FOREGROUNDCOLOUR;
	}
      }
      break;
    case I_RIGHTEDGE:
      for (j = 0; j < SMR_LS_N; j++) {
	// "Mark" all linesensors that has a smaller left limit than
	// the left position of the polyline. Note that is->lc[] is used
	// as the left position of the overlapping line segment
	if (is->lc[i] * linesensorwidth / 2.0 >= ls_limits[j+1]) {
	  linesensor[j] = FOREGROUNDCOLOUR;
	}
      }
      break;
    case I_WITHIN:
      for (j = 0; j < SMR_LS_N; j++) {
	// "Mark" all linesensors where the left position of the overlapping
	// line is larger than the sensor's right limit and the right position
	// is smaller than the left limit
	if ((is->lc[i] * linesensorwidth / 2.0 >= ls_limits[j+1]) &&
	    (is->c[i] * linesensorwidth / 2.0 <= ls_limits[j])) {
	  linesensor[j] = FOREGROUNDCOLOUR;
	}
      }
      break;
    default:
      break;
    }
  }
  return is->intersections;
}
