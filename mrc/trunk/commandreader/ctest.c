// $Id: ctest.c,v 1.1.1.1 2005/11/22 21:39:58 naa Exp $
#include <string.h>

#include "commandreader.h"

int main(int argc, char **argv) {
  char filename[64];
  int route[4][6];
  int lines;

  // Userfriendliness: allow command line arguments... wow!
  if (argc == 2)
    strncpy(filename,argv[1],64);
  else
    strcpy(filename,"test.smr");

  commandreader_readfile(filename,route,4,&lines);
  commandreader_print(route,lines);

  return 0;
}
