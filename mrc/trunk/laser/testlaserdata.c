#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
#include "laserdefs.h"

int main(int argc,char *argv[]){

double timestamp,resolution,startangle,th,wc,vel;
int scanno,nmeas,finished=0;
int i,j,type,edgeleft,edgeright;
FILE *f,*f1;
char line[10000];
char *p;
int n=0;
scantype sc;
posetype startline,optline,tmpline;
double wmin,wmint,wrob;
sc.x=malloc(1000*sizeof(double));
sc.y=malloc(1000*sizeof(double));
sc.r=malloc(1000*sizeof(double));
sc.th=malloc(1000*sizeof(double));

posetype lineguess, lineres;
//struct timeval tv,tv1;
//struct timezone tz,tz1;
wrob=0.4;
if (argc==2){
  //f1=fopen("scan","w");
  f=fopen(argv[1],"r");
  if ( !f) exit(1);
  while ( fgets(line,10000-1,f)&& !finished){
  timestamp=strtod(line,&p);

  scanno=strtod(p,&p);
  resolution=strtod(p,&p);
  startangle=strtod(p,&p);
  nmeas=strtod(p,&p);
//    printf("%lf %d \n",timestamp, nmeas);
    j=0;
    sc.nactual=nmeas;
  for (i=0;i< nmeas;i++){
     sc.r[i]=strtod(p,&p);
     if (sc.r[i]>0.4 && sc.r[i] < 2.0){
       th=(startangle+i*resolution)/180*M_PI;
       sc.x[i]=cos(th)*sc.r[i];
       sc.y[i]=sin(th)*sc.r[i];
      // fprintf(f1,"%lf %lf \n",  pos[j][0],  pos[j][1]);
     }
     else {
       sc.r[i]=-1;
     }      
  }
  finished=1;
 
 }
 fclose(f);
 }
 else {
 sc.r[0]=1;
 sc.r[1]=1.2;
 sc.r[2]=1.5;
 sc.th[0]=0;
 sc.th[1]=0.01;
 sc.th[2]=0.02;
 sc.nactual=3;
 for (i=0;i<sc.nactual;i++){
  th=(startangle+i*resolution)/180.0*M_PI;
       sc.x[i]=cos(th)*sc.r[i];
       sc.y[i]=sin(th)*sc.r[i];
 }
 }
 #if(0)
 { int istart,p1,p2;
 istart=0;
 while (1){
 
  type=findhole(&sc,wrob,istart,&edgeright,&edgeleft);
  //printf("type %d   edgeleft %d  edgeright %d \n",type,edgeleft,edgeright);
  getchar();
  if (searchback(&sc,edgeright,edgeleft,wrob,&wmin,&p1)){
     if(searchfwd(&sc,p1,edgeleft,wrob,&wmin,&p2)){
         printf("hole %d %d \n",p1,p2);
	 istart=p2;
     }
     else {
        istart=edgeleft;
     }
  }	
  else 
    istart=edgeleft;
  getchar();
 }
 #endif
 type=findbesthole(&sc,wrob,0,&edgeright,&edgeleft);
 printf("%d %d \n",edgeright,edgeleft);
 //printf("type %d e1 %d e2 %d \n",type,edgeright,edgeleft);

 lineguess.x=(sc.x[edgeright]+sc.x[edgeleft])/2;
 lineguess.y=(sc.y[edgeright]+sc.y[edgeleft])/2;
 lineguess.th=atan2(-sc.x[edgeright]+sc.x[edgeleft],-sc.y[edgeleft]+sc.y[edgeright]);
 wmin=bestline(sc,lineguess,&lineres,0.2,0.02);
 printf(" %f %f %f  %f\n",lineres.x,lineres.y,lineres.th,wmin);
 
 
 return 0;
}
