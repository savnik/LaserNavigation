#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "poseio.h"

static void poseio_doublepoints(posetype pose) {
  double *tmpx,*tmpy,*tmptheta;
  int oldmaxpoints = pose->maxpoints;

  tmpx = (double *) malloc(pose->maxpoints * sizeof(double));
  tmpy = (double *) malloc(pose->maxpoints * sizeof(double));
  tmptheta = (double *) malloc(pose->maxpoints * sizeof(double));
  
  memcpy(tmpx,pose->x,pose->maxpoints * sizeof(double));
  free(pose->x);

  memcpy(tmpy,pose->y,pose->maxpoints * sizeof(double));
  free(pose->y);

  memcpy(tmptheta,pose->theta,pose->maxpoints * sizeof(double));
  free(pose->theta);

  pose->maxpoints *= 2;

  pose->x = (double *) malloc(pose->maxpoints * sizeof(double));
  pose->y = (double *) malloc(pose->maxpoints * sizeof(double));
  pose->theta = (double *) malloc(pose->maxpoints * sizeof(double));

  if ((pose->x == NULL) || (pose->y == NULL) || (pose->theta == NULL) ||
      (tmpx == NULL) || (tmpy == NULL) || (tmptheta == NULL)) {
    fprintf(stderr,"Error: malloc returned NULL\n");
    exit(1);
  }
  memcpy(pose->x,tmpx,oldmaxpoints * sizeof(double));
  memcpy(pose->y,tmpy,oldmaxpoints * sizeof(double));
  memcpy(pose->theta,tmptheta,oldmaxpoints * sizeof(double));

  free(tmpx);
  free(tmpy);
  free(tmptheta);
}


void poseio_addpoint(posetype *p, double x, double y, double theta) {

  if (*p == NULL) {
    *p = (posetype) malloc(sizeof(struct pose));
    (*p)->numpoints = 0;
    (*p)->maxpoints = 1;
    (*p)->x = (double *) malloc(sizeof(double));
    (*p)->y = (double *) malloc(sizeof(double));
    (*p)->theta = (double *) malloc(sizeof(double));
  }

  (*p)->x[(*p)->numpoints] = x;
  (*p)->y[(*p)->numpoints] = y;
  (*p)->theta[(*p)->numpoints] = theta;
  (*p)->numpoints++;
  if ((*p)->numpoints == (*p)->maxpoints) {
    poseio_doublepoints(*p);
  }
}

void poseio_print(posetype p) {
  int i;

  printf("Poselist with %d of %d points used\n",p->numpoints,p->maxpoints);
  for (i = 0; i < p->numpoints; i++) {
    printf("%f\t%f\t%f\n",p->x[i],p->y[i],p->theta[i]);
  }
}

static void poseio_readascii(FILE *fp, posetype *p) {
  unsigned char line[1000];
  float x,y,theta;

  while(fgets(line,1000,fp)) {
    sscanf(line,"%f %f %f",&x,&y,&theta);
    poseio_addpoint(p,x,y,theta);
  }
}

static void poseio_readbinary(FILE *fp, posetype *p) {
  int i;
  unsigned long filelength, numpoints, readpoints;
  double *x, *y, *theta;
  unsigned char doublesize;

  fseek(fp,0L,SEEK_END);
  filelength = ftell(fp) - 1;
  rewind(fp);
  
  if (1 != fread(&doublesize, 1, 1, fp)) {
    fprintf(stderr,"poseio: Error: unable to read first byte in binary pose file\n");
    fclose(fp);
    exit(1);
  }

  if (doublesize != sizeof(double)) {
    fprintf(stderr,"poseio: Error: sizeof(double) does not match stored value in binary posefile\n");
    fclose(fp);
    exit(1);
  }

  if (filelength % (3 * sizeof(double))) {
    fprintf(stderr,"Error: binary posefile does not contain 3*N*sizeof(double) bytes, but 3*N+%ld\n",filelength%3);
    fclose(fp);
    exit(1);
  }

  numpoints = filelength / (3 * sizeof(double));

  x = (double *) malloc(numpoints * sizeof(double));
  y = (double *) malloc(numpoints * sizeof(double));
  theta = (double *) malloc(numpoints * sizeof(double));

  if (numpoints != (readpoints = fread(x,sizeof(double),numpoints,fp))) {
    fprintf(stderr,"Error: poseio_readbinary could not read %ld x coordinates only %ld\n",numpoints, readpoints);
    exit(1);
  }
  if (numpoints != fread(y,sizeof(double),numpoints,fp)) {
    fprintf(stderr,"Error: poseio_readbinary could not read %ld x coordinates\n",numpoints);
    exit(1);
  }
  if (numpoints != fread(theta,sizeof(double),numpoints,fp)) {
    fprintf(stderr,"Error: poseio_readbinary could not read %ld x coordinates\n",numpoints);
    exit(1);
  }

  for (i = 0; i < numpoints; i++) {
    poseio_addpoint(p,x[i],y[i],theta[i]);
  }
}

void poseio_readfile(char *filename, enum posefiletype filetype, posetype *p) {

  FILE *fp;

  if (*p) {
    fprintf(stderr,"Warning: poseio_readfile got non-empty pose list, appending...\n");
  }

  if (!(fp = fopen(filename,"rb"))) {
    fprintf(stderr,"poseio: unable to open file %s for reading\n", filename);
    exit(1);
  }

  switch (filetype) {
  case PFT_DEFAULT:
  case PFT_ASCII:
    poseio_readascii(fp,p);
    break;
  case PFT_BINARY:
    poseio_readbinary(fp,p);
    break;
  default:
    fprintf(stderr,"poseio: unknown filetype given: %d\n",filetype);
    fclose(fp);
    exit(1);
  }
  fclose(fp);
}

static void poseio_writeascii(FILE *fp, posetype p) {

  int i;
  
  for (i = 0; i < p->numpoints; i++) {
    fprintf(fp,"%f\t%f\t%f\n",p->x[i],p->y[i],p->theta[i]);
  }

}

static void poseio_writebinary(FILE *fp, posetype p) {
  unsigned char doublesize = sizeof(double);
  fwrite(&doublesize, 1, 1, fp);
  fwrite(p->x, sizeof(double), p->numpoints, fp);
  fwrite(p->y, sizeof(double), p->numpoints, fp);
  fwrite(p->theta, sizeof(double), p->numpoints, fp);
}

void poseio_writefile(char *filename, enum posefiletype filetype, posetype p) {
  FILE *fp;

  if (!(fp = fopen(filename,"wb"))) {
    fprintf(stderr,"Error: poseio_writefile could not open file %s\n",filename);
    exit(1);
  }
  if (!p) {
    fprintf(stderr,"Warning: poseio_writefile got NULL pose\n");
  } else {
    switch(filetype) {
    case PFT_DEFAULT:
    case PFT_ASCII:
      poseio_writeascii(fp, p);
      break;
    case PFT_BINARY:
      poseio_writebinary(fp, p);
      break;
    default:
      fprintf(stderr,"Warning: poseio_writefile got invalid output type: %d\n",filetype);
      break;
    }
  }
  fclose(fp);
}
