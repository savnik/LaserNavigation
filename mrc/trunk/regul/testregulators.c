#include "regulators.h"
#include <stdio.h>

int main(void){
  pd_type pd;
  int i;
  pd.k=1;
  pd.tau=7;
  pd.alfa=0.2;
  pd.ts=0.01;
  pd_init(&pd);
  pd.e=-1;
  for (i=0;i<1000;i++){
    pd_out(&pd);
    printf("%lf \n",pd.o);
    pd_update(&pd);
  }
  return(0);
}