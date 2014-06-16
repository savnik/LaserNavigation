#include <math.h>
#include <stdio.h>
#include "obstacle.h"


#define min(x,y)	((x<y)?(x):(y))



void obstacle(int nobs,double d_th[][2],double pos[][2],double R,robotsizetype dim){
int i;
double dmax=DMAX;
double thmax=2*M_PI;
double rmin,ra,rb,rc,rd;
double b,c,d,x,y,r;
double signy;
b=dim.b;
c=dim.c;
d=dim.d;
if (R< 0) signy=1; else signy=-1;
R=fabs(R);
if (R>100000){
  for(i=0;i<nobs;i++){
    d_th[i][1]=0;
    if(fabs(pos[i][1])>b)
      d_th[i][0]=dmax;
    else{
       if (pos[i][0]>c){
         d_th[i][0]=pos[i][0]-c;
         d_th[i][0]=min(d_th[i][0],dmax);
       }
       else{
         d_th[i][0]=dmax;
       }
    }
  }
}

else{
rmin=R-b;
ra=hypot(c,(R+b));
rb=hypot(c,(R-b));
rc=hypot(d,(R-b));
rd=hypot(d,(R+b)); 
if (R > b){
for (i=0;i<nobs;i++){
      x=pos[i][0];
      y=pos[i][1]*signy;
      r=hypot(x,y+R);
      if (y>b && r < rd){
          // thad
          //printf("case1\n");
          d_th[i][1]=M_PI-asin((R+b)/r)-atan2(y+R,x);
      }
      else{
        if (r<rb ){
              if (r >rmin){
                   //thbc
                   //  printf("case 2 <n");
                   d_th[i][1]=acos((b-R)/r)-atan2(x,-y-R);
		   if (d_th[i][1] < 0)
		        d_th[i][1]+=2*M_PI;
              }
	      else{
               //    printf("'case 3' \n");
                   d_th[i][1]=thmax;
              }
	}
        else{
               if (r < ra){
                    //thab
                 //   printf("'case4' \n");
                    d_th[i][1]=M_PI-asin(c/r)-atan2(x,-y-R);
                    if (d_th[i][1] <0)
                       d_th[i][1]+=2*M_PI;
               }
               else{
               // printf("'case5'\n");
                    d_th[i][1]=thmax;
               }
        }
      }
      d_th[i][0]=r*d_th[i][1];
  }
}
else{ 
    for (i=0;i<nobs;i++){
      x=pos[i][0];
      y=pos[i][1]*signy;
      r=hypot(x,y+R);
      if (y>b && r < rd){
             // thad
             // printf("'case 6 \n");
             d_th[i][1]=M_PI-asin((R+b)/r)-atan2(y+R,x);
      }
      else {
          if(x < -d && r <rc){
                   //thdc
                 // printf("case7 \n");
                  d_th[i][1]=-asin(d/r)-atan2(x,-y-R);
          }
	  else{
              if (r>ra){
                  // printf("'case8'\n");
                  d_th[i][1]=thmax;
	      }
              else{
                  if (r > rb){
                      //thab
                     //printf(" 'case 9'\n");
                      d_th[i][1]=M_PI-asin(c/r)-atan2(x,-y-R);
		      if (d_th[i][1] < 0)
		        d_th[i][1]+=2*M_PI;
                  }
		  else{
                      //thbc
                   // printf"  'case10'\n");
                      d_th[i][1]=acos((b-R)/r)-atan2(x,-y-R);
		      if (d_th[i][1] < 0)
		        d_th[i][1]+=2*M_PI;
                  }
              }
          }
      }
       d_th[i][0]=r*d_th[i][1];
    }
 } 
             
}             
    
}


int findobstaclecourse(double *wc,int Nobst, double obstpos[][2],posetype tgtpose,posetype robpose,double vel,
			double deltaw,int Nradii,robotsizetype robotdim){
posetype deltagoal;
int imean,i,j,objmaxi;
double d_th[NOBSTMAX][2];
double R,w,dmin,th1;
double obj1,obj2,obj,objmax;
  deltagoal.x=tgtpose.x-robpose.x;
  deltagoal.y=tgtpose.y-robpose.y;
  deltagoal.th=tgtpose.th-robpose.th;
  th1=atan2(deltagoal.y,deltagoal.x);
  imean=Nradii/2;
  objmax=0;
  for (i=0;i<Nradii;i++){ 
    w=(i-imean)*deltaw; 
    if (i==imean){
       R=1000000;
    }
    else {
      R=vel/w;
    }
    obstacle(Nobst,d_th,obstpos,R,robotdim);
    dmin=d_th[0][0];
    for (j=1;j<Nobst;j++){
      if (d_th[j][0] <dmin){
        dmin=d_th[j][0];
      }
    }
    
    
    dmin=min(dmin,DMAX);
    printf("R %lf dmin %lf \n",R,dmin); 
    obj1=dmin/DMAX;
    obj2=1-fabs(th1-robpose.th-w*TSAMOBJ)/M_PI;
    obj=obj1+0.2*obj2;
     
    if (obj >objmax){
      objmax=obj;
      objmaxi=i;
    }
} 

*wc=(objmaxi-imean)*deltaw;
//printf("objmax %lf  objmaxi %d  wc %lf \n",objmax,objmaxi,*wc);
//     getchar(); 
return 0;
}
