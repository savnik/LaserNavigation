#include "regulators.h"


void pd_init(pd_type * p){
 p->b=(2*p->tau+p->ts)/(2*p->alfa*p->tau+p->ts);
 p->a=(2*p->ts)/(2*p->alfa*p->tau+p->ts);
 p->state=0;
 p->o=0;
}

void pd_out(pd_type *p){
  p->o=( p->k * p->e - p->state )*p->b + p->state;
}

void pd_update(pd_type *p){
  p->state=(p->k*p->e - p->state)*p->a+p->state;
}
