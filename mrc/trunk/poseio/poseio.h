#ifndef POSEIO_H
#define POSEIO_H

typedef struct pose * posetype;
struct pose {
  int numpoints, maxpoints; // "Accounting" variables
  double *x, *y, *theta; // Pointers meant for (x,y,theta) arrays
};

enum posefiletype {
  PFT_DEFAULT,
  PFT_ASCII,
  PFT_BINARY
};

void poseio_readfile(char *filename, enum posefiletype filetype, posetype *p);
void poseio_writefile(char *filename, enum posefiletype filetype, posetype p);
void poseio_print(posetype p);

#endif
