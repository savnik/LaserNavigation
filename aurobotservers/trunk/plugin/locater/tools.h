# ifndef TOOLS_DOT_H
# define TOOLS_DOT_H

# include <math.h>

// Defines functions as external C code. For use in C++ programs.
#ifdef __cplusplus
 extern "C" {
#endif

/**
  Calculates distance from point to line. 
  Inputs: A, B, C (line parameters: Ax+By+C=0)  
          x, y (point) 
*/
double dist2line(double, double, double, double, double);

/**
  Calculate shortest distance between points
  Inputs: x1, y1 first point. x2, y2 second point
*/
double pointdist(double x1, double y1, double x2, double y2);

/**
  Normalize angle to be within -pi and pi.
*/
double angle_normalize(double);

/**
  Angle between to lines normalized to be within -pi/2 and pi/2.
*/
double angle_deviation(double, double, double, double);

#ifdef __cplusplus
 }
#endif

# endif //TOOLS_DOT_H
