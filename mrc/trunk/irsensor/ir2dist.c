#include "ir2dist.h"

int calc_ir_param(irparamtype param[],double d1[],double ir1[],double d2[],double ir2[],int N){
  int i;
  for (i=0;i<N;i++){
    param[i].k2=(d1[i]*ir1[i]-d2[i]*ir2[i])/(d1[i]-d2[i]);
    param[i].k1=d1[i]*(ir1[i]-param[i].k2);
  }
  return(0);
}

void ir2dist(double dist[],unsigned char irval[],irparamtype param[],int N){
  int i,h;
  for (i=0;i<N ;i++){
    h=irval[i]-param[i].k2;
    if (h < 1.0)
      dist[i]=1.0;
    else
      dist[i]=param[i].k1/h;
  }
}

void ir2dist1(double dist[],unsigned char irval[],irparamtype param[],irsamptype samp[],int N){
  int i,h;
  for (i=0;i<N ;i++){
    // Storing the previous value
    samp[i].lval=dist[i];
    // Calculating the value
    h=irval[i]-param[i].k2;
    if (h < 1.0)
      dist[i]=1.0;
    else
      dist[i]=param[i].k1/h;
    // Checking if the value has been updated
    if((samp[i].lval != dist[i]) || (samp[i].repeated >=  (REFRESH_TIME - 1))){
      samp[i].newval=1;
      samp[i].repeated=0;
    }else{
      samp[i].newval=0;
      samp[i].repeated++;
    }
  }
}


