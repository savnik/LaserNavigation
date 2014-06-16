#include <stdlib.h>

#include "poseio.h"

int main(int argc, char **argv) {

  posetype p = NULL;

  poseio_readfile("test.pose",PFT_ASCII,&p);
  //  poseio_readfile("test.pose.dat",PFT_BINARY,&p);
  poseio_print(p);
  poseio_writefile("test.pose.dat",PFT_BINARY,p);
  //  poseio_writefile("test.pose",PFT_ASCII,p);

  return 0;
}
