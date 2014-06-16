#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
#include "obstacle.h"

int main(int argc,char *argv[]){
double d_th[1000][2];
double pos[1000][2];
double res[1000][2];
double radii[1000];
double R;
double timestamp,resolution,startangle,th,wc,vel;
int scanno,nmeas;
int i,j;
FILE *f,*f1;
char line[10000];
char *p;
int n=0;
posetype robotpose, tgtpose;
robotsizetype robotdim;
//struct timeval tv,tv1;
//struct timezone tz,tz1;
robotdim.b=0.22;
robotdim.c=0.25;
robotdim.d=0.05;

if (argc==2){
  f1=fopen("scan","w");
  f=fopen(argv[1],"r");
  if ( !f) exit(1);
  while ( fgets(line,10000-1,f)){
  timestamp=strtod(line,&p);

  scanno=strtol(p,&p,10);
  resolution=strtod(p,&p);
  startangle=strtod(p,&p);
  nmeas=strtol(p,&p,10);
    printf("%lf %d \n",timestamp, nmeas);
    j=0;
  for (i=0;i< nmeas;i++){
     radii[i]=strtod(p,&p);
     if (radii[i]>0.02){
       th=(startangle+i*resolution)/180*M_PI;
       pos[j][0]=cos(th)*radii[i]+0.28;
       pos[j][1]=sin(th)*radii[i];
       fprintf(f1,"%lf %lf \n",  pos[j][0],  pos[j][1]);
       j++;
     }       
  }
 robotpose.x=0;
 robotpose.y=0;
 robotpose.th=0;
 tgtpose.x=3;
 tgtpose.y=0;
 tgtpose.th=0;
 vel=0.2;
 findobstaclecourse(&wc,j,pos,tgtpose,robotpose,vel,0.05,41,robotdim);
 printf("wc %lf\n",wc);
 }
 fclose(f1);
}

 return 0;
}
