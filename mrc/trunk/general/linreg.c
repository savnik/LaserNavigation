#include <math.h>
#include "mobilerobot.h"

void linreg(linregtype *p){
double sx=0,sx2=0,sy=0,sy2=0,sxy=0,nsx2,nsy2,nsxy;
int i;
for (i=0;i<p->N;i++){
  sx+=p->data[i].x;
  sx2+=p->data[i].x*p->data[i].x;
  sy+=p->data[i].y;
  sy2+=p->data[i].y*p->data[i].y;
  sxy+=p->data[i].x*p->data[i].y;
}
nsx2=p->N*sx2-sx*sx;
nsy2=p->N*sy2-sy*sy;
nsxy=p->N*sxy-sx*sy;
if (nsx2 >= nsy2){
  p->line.th=atan2(nsxy,nsx2);
  p->line.x=0;
  p->line.y=(sy-nsxy/nsx2*sx)/p->N;
}
else {
  p->line.th=atan2(nsy2,nsxy);
  p->line.x=(sx-nsxy/nsy2*sy)/p->N;
  p->line.y=0;
}
}
