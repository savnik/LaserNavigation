#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "xfigreader.h"
#include "smrsim.h"
#include "intersect.h"

int main(int argc, char **argv) {
  polylinestype lines = NULL;
  char filename[64];
  posetype pose;
  int intersection = -1,line = -1,point = -1,i;
  double coef = -1.0;
  intersectionstype is;
  int ls[SMR_LS_N];

  // Userfriendliness: allow command line arguments... wow!
  if (argc == 2)
    strncpy(filename,argv[1],64);
  else
    strcpy(filename,"test.fig");

  // Read polylines from "filename"
  lines = xfr_init(filename);
  
  // Print the polylines
  polyline_print(lines);

  
  pose.x = 1.0;
  pose.y = 1.0;
  pose.th = 0.0;

  intersection = intersect_linesensor(lines,pose,0.3,0.3,&is,ls);
  printf("intersections: %d ( %d )\n",intersection,is.intersections);
  for (i = 0; i<is.intersections; i++)
    printf("Intersection type: %d ( %d , %d , %f )\n",is.types[i],is.lines[i],is.points[i],is.c[i]);
  for (i = 0; i < SMR_LS_N; i++)
    printf("linesensor[%d] = %d\n",i,ls[i]);

  // Free memory occupied by lines
  polyline_free(&lines);

  return 0;
}
