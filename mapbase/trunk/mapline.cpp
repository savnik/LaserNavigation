# include "mapline.h"

/**
    Maptype used in mapbase module. Implements line features.

*/

mapline::mapline()
{
  isObstacle = true;
  number[0] = '\0';
}

mapline::~mapline()
{
}

void mapline::display()
{
  printf("Start     : x = %f, y = %f\n", x_s, y_s);
  printf("End       : x = %f, y = %f\n", x_e, y_e);
  printf("Line      : A = %f, B = %f, C = %f\n", A, B, C);
  printf("Perimeter : p = %f\n", perimeter);
  printf("Nodename  : %s\n", nodename);
}

/////////////////////////////////////

void mapline::fill_ABC()
{ // so that Ax+By+C=0, and sqrt(A^2+B^2) = 1
  double dx = x_s - x_e;
  double dy = y_s - y_e;
  double lng;

  if(fabs(dx) > fabs(dy))
  {
    A = dy / dx;
    B = -1;
    C = y_s - A * x_s;
  }
  else
  {
    A = -1;
    B = dx / dy;
    C = x_s - B * y_s;
  }
  // normalize line, so that hypot(A,B) == 1.0
  lng = hypot(A,B);
  A /= lng;
  B /= lng;
  C /= lng;
}

/////////////////////////////////////////

void mapline::xml_export(string *appxml, char *tmp, const char *spacer)
{
  sprintf(tmp, "%s  <line number=\"%s\" perimeter=\"%.3f\" isObstacle=\"%d\">\n", spacer, number, perimeter, isObstacle);
  *appxml += tmp;
  sprintf(tmp, "%s    <start x=\"%.3f\" y=\"%.3f\" />\n", spacer, x_s, y_s);
  *appxml += tmp;
  sprintf(tmp, "%s    <end x=\"%.3f\" y=\"%.3f\" />\n", spacer, x_e, y_e);
  *appxml += tmp;
  sprintf(tmp, "%s  </line>\n", spacer);
  *appxml += tmp;
}
