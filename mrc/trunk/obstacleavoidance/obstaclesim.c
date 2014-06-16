#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "obstacle.h"


int main(int argc,char *argv[]){
double pos[300][2];
double obstpos[300][2];
posetype robotpose[1001],tgtpose;
double wc,vel,tsam,deltad,th,ts;
int i,j,nsam;
FILE *f;
char line[256];
int Nobst=0;
robotsizetype robotdim;
struct timeval tv,tv1;
struct timezone tz,tz1;
if (argc==1){
  robotdim.b=0.22;
  robotdim.c=0.25;
  robotdim.d=0.05;
  for ( i=0;i<20;i++){
    pos[i][0]=i*0.05;
    pos[i][1]=0.30;
    pos[i+20][0]=1.00;
    pos[i+20][1]=0.3-0.03*i;
    pos[i+40][0]=i*0.02;
    pos[i+40][1]=-0.30;
    pos[i+60][0]=0.4;
    pos[i+60][1]=-0.3-0.03*i;
    pos[i+80][0]= 0.4+i*0.05;
    pos[i+80][1]=-0.90;    
  }
  Nobst=100;
  #if(0)
  pos[0][0]=0.1;	pos[0][1]=0.3;
  pos[1][0]=0.4;	pos[1][1]=0.3;
  pos[2][0]=0.7; 	pos[2][1]=0.3;
  pos[3][0]=0.9;	pos[3][1]=0.3;
  pos[4][0]=1.0; 	pos[4][1]=0.3;
  pos[5][0]=1.0;	pos[5][1]=0.0;
  pos[6][0]=1.0; 	pos[6][1]=-0.3;
  pos[7][0]=0.1;	pos[7][1]=-0.3;
  pos[8][0]=0.4; 	pos[8][1]=-0.3;
  pos[9][0]=0.4;	pos[9][1]=0.3;
  pos[10][0]=0.4; 	pos[10][1]=-0.6;
  pos[11][0]=0.7;	pos[11][1]=-0.9;
  pos[12][0]=0.7;	pos[12][1]=-0.9;
  
  Nobst=13;
  #endif
  
  robotpose[0].x=0;
  robotpose[0].y=0;
  robotpose[0].th=0;
  gettimeofday(&tv,&tz);
  wc=0;
  vel=0.2;
  ts=0.01;
  nsam=20;
  tsam=nsam*ts;
  tgtpose.x=3;
  tgtpose.y=-0.6;
  tgtpose.th=0;
  f=fopen("simdata","w");
  for(i=0;i<1000;i++){
    if (i % nsam==0){ 
    for (j=0;j<Nobst;j++){
      double thr,xh,yh;
      thr=robotpose[i].th;
      xh=pos[j][0]-robotpose[i].x;
      yh=pos[j][1]-robotpose[i].y;
      obstpos[j][0]=cos(thr)*xh+sin(thr)*yh;
      obstpos[j][1]=-sin(thr)*xh+cos(thr)*yh;
    }
    findobstaclecourse(&wc,Nobst,obstpos,tgtpose,robotpose[i],vel,0.05,41,robotdim);
    }
    deltad=vel*ts;
    th=robotpose[i].th;
    robotpose[i+1].x=robotpose[i].x+deltad*cos(th);
    robotpose[i+1].y=robotpose[i].y+deltad*sin(th);
    robotpose[i+1].th=robotpose[i].th+wc*ts;
    fprintf(f,"%lf %lf %lf %lf \n",robotpose[i].x,robotpose[i].y,robotpose[i].th,wc);
  }

 }
 fclose(f);
gettimeofday(&tv1,&tz1);
printf("dt %lf\n",(double) tv1.tv_usec -tv.tv_usec); 
return 0;
}
