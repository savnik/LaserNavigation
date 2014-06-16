#include  <math.h>
#include "mobilerobot.h"

void normalizeangle(double *angle){
while(*angle > M_PI) *angle-=2*M_PI;
while(*angle < -M_PI) *angle+=2*M_PI;
}

double limit(double v,double limv_hi,double limv_lo){
if( v > limv_hi) v=limv_hi;
if( v < limv_lo) v=limv_lo;
return(v);
}

double sign(double x){
if (x < 0 )
  return(-1.0);
else
  return(1.0);
}

posetype posediff(posetype p1,posetype p2){
  posetype r;
  r.x=p1.x-p2.x;
  r.y=p1.y-p2.y;
  r.th=p1.th-p2.th;
  return(r);
}
