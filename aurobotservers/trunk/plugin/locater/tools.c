# include "tools.h"

double dist2line(double A, double B, double C, double x, double y)
{ // Perpendicular distance from point to line
  //
  return((A * x + B * y + C) / sqrt(pow(A, 2) + pow(B, 2)));
}

double pointdist(double x1, double y1, double x2, double y2)
{ // Shortest distance between points
  //
  double dx = x2 - x1;
  double dy = y2 - y1;
  
  return(sqrt(pow(dx, 2) + pow(dy, 2)));
}

double angle_normalize(double ang)
{ // Keeps the angle within the usual band: -pi to pi
  //
  
  while(ang > M_PI)
    ang -= 2 * M_PI;
  while(ang < -M_PI)
    ang += 2 * M_PI;  
  return(ang);
}

double angle_deviation(double Am, double  Bm, double Ap, double Bp)
{ // Angle between to lines normalized to be within -pi/2 and pi/2 
  //
  
  double out;
  
  out = atan2(-Ap, Bp) - atan2(-Am, Bm);

  while(out > M_PI / 2.0)
    out -= M_PI;
  while(out < -M_PI / 2.0)
    out += M_PI;
  
  return(out);
}

//
//  Rotational matrix 2D:   [  cos   sin ]
//                          [ -sin   cos ]
//
