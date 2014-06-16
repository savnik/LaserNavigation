#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "xfigreader.h"

int main(int argc, char **argv) {
  polylinestype lines = NULL;
  char filename[64];

  // Userfriendliness: allow command line arguments... wow!
  if (argc == 2)
    strncpy(filename,argv[1],64);
  else
    strcpy(filename,"test.fig");

  // Read polylines from "filename"
  lines = xfr_init(filename);
  
  // Print the polylines
  polyline_print(lines);
  
  // Free memory occupied by lines
  polyline_free(&lines);

  
  return 0;
}
