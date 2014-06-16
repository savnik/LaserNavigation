#include <stdio.h>
#include <math.h>
#include "laserdefs.h"
#include <stdlib.h>

int main(void){
scantype sc;
posetype startline,optline,tmpline;
double wmin,wmint;
int i;
sc.x=malloc(10*sizeof(double));
sc.y=malloc(10*sizeof(double));

sc.x[0]=4;
sc.x[1]=4;
sc.y[0]=2;
sc.y[1]=4;
sc.x[2]=8;
sc.x[3]=8;
sc.y[2]=2;
sc.y[3]=4;


wmin=0;
startline.x=3;
startline.y=0;
startline.th=M_PI/2;
for (i=-10;i<10;i++){
  startline.th=M_PI/2-0.13+i*0.02;
  wmint=centreline(sc,startline,0,1,2,3,&tmpline);
  if (wmint > wmin){
    wmin=wmint;
    optline=tmpline;
  }
}    
printf("wmin %f  x %f y %f th %f\n",wmin,optline.x,optline.y,optline.th);
return 0;
}
