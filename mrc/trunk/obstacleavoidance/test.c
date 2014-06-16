#include <stdio.h>
#include <sys/time.h>

typedef struct {double b,c,d;} robotsizetype;

void obstacle(int nobs,double d_th[][2],double pos[][2],double R,robotsizetype dim);

int main(int argc,char *argv[]){
double d_th[300][2];
double pos[300][2];
double res[300][2];
double R;
int i,j;
FILE *f;
char line[256];
int n=0;
robotsizetype robotdim;
struct timeval tv,tv1;
struct timezone tz,tz1;
if (argc==1){
  robotdim.b=0.15;
  robotdim.c=0.25;
  robotdim.d=0.05;
  R=0.2;
  for ( i=0;i< 150;i++){
    pos[i][0]=i*0.05-0.20;
    pos[i][1]=0.17;
    pos[i+150][0]=i*0.05-0.20;
    pos[i+150][1]=-0.17;
  }
  n=300;
  gettimeofday(&tv,&tz);
 for (j=1;j<51;j++){
 R=0.2/(j*0.04);
 obstacle(n,d_th,pos,-R,robotdim);
 }
gettimeofday(&tv1,&tz1);
printf("dt %lf\n",(double) tv1.tv_usec -tv.tv_usec); 

}
else{
  f=fopen(argv[1],"r");
  if ( !f) exit(1);
  fgets(line,255,f);
  sscanf(line,"%lf %lf %lf %lf",&R,&robotdim.b,&robotdim.c,&robotdim.d);
  printf("%lf %lf %lf %lf \n",R,robotdim.b,robotdim.c,robotdim.d);
  while (fgets(line,255,f)){
    sscanf(line,"%lf %lf %lf %lf",&pos[n][0],&pos[n][1],&res[n][0],&res[n][1]);
    n++;
  }
 fclose(f);
 for (i=0;i<n;i++){
   printf("%lf %lf %lf %lf\n",pos[i][0],pos[i][1],res[i][0],res[i][1]);
 }

 gettimeofday(&tv,&tz);
 obstacle(n,d_th,pos,R,robotdim);
  gettimeofday(&tv1,&tz1);
  printf("dt %lf\n",(double) tv1.tv_usec -tv.tv_usec); 
  for (i=0;i<n;i++){
   printf("dd %lf   dth %lf    \n",d_th[i][0]-res[i][0],d_th[i][1]-res[i][1]);
  }
 }
return 0;
}
