# include "mappoint.h"

/**
    Maptype used in rowfinder module. Implements point features.

*/

mappoint::mappoint()
{
}

mappoint::~mappoint()
{
}

void mappoint::display()
{
  printf("Values    : x = %f, y = %f\n", x, y);
  printf("Perimeter : p = %f\n", perimeter);
  printf("Nodename  : %s\n", nodename);
}

void mappoint::xml_export(string *appxml, char *tmp, const char *spacer)
{
  sprintf(tmp, "  %s<point perimeter=\"%.3f\">\n", spacer, perimeter);
  *appxml += tmp;
  sprintf(tmp, "%s    <value x=\"%.3f\" y=\"%.3f\" />\n", spacer, x, y);
  *appxml += tmp;
  sprintf(tmp, "%s  </point>\n", spacer);
  *appxml += tmp;
}
