#include "laserdefs.h"
#include <math.h>
#include "mobilerobot.h"
#include <stdio.h>

// type
// 0 no hole    1  open  2 open right 3 open left 4 hole

int findhole(scantype * sc,double w,int start,int *edgeright,int*edgeleft){
int i=start,result;
double r,r0;

if ((r=sc->r[i]) < 0){
  i++;
  while (i < sc->nactual && (r= sc->r[i]) < 0 ) i++;
  if(i==sc->nactual)
     result=1;    
  else{
    result = 2;
    *edgeleft=i;
  }
 }
 else {
   r0=r;
   i++;
   while (i < sc->nactual && (r= sc->r[i]) >= 0 && fabs(r-r0)<w){ i++;r0=r;}
   if(i==sc->nactual){
     result=0;
   }
   else {
    *edgeright=i-1;
    if (r < 0){
      i++;
      while (i < sc->nactual && (r= sc->r[i]) < 0 ) i++;
      if(i==sc->nactual)
        result=3; 
      else{
        result=4;
	*edgeleft=i;
      }
    }
    else {
      result=4;
      *edgeleft=i;
    }
   }
 }        
return result;
}
    
 

// returns 0 if win < wrob else 1
int searchfwd(scantype *sc,int edge1,int edge2,double wrob, double *wmin, int *pmin){
   int i,finished=0,result=1;
   double dist,x0,y0;
   i=edge2;
   *wmin=1e38;
   x0=sc->x[edge1];
   y0=sc->y[edge1];
   while (!finished && i< sc->nactual){
     if (sc->r[i] > 0){
       dist= hypot(sc->x[i]-x0,sc->y[i]-y0);
	 if (dist < *wmin){
	   *wmin=dist;
	   *pmin=i;
	   if (dist < wrob){
	     result=0;
	     finished=1;
	   }
	}
      }
      i++;
    }
  return result;
}

// returns 0 if win < wrob else 1
int searchback(scantype *sc,int edge1,int edge2,double wrob, double *wmin, int *pmin){
   int i,finished=0,result=1;
   double dist,x0,y0;
   i=edge1;
   *wmin=1e38;
   x0=sc->x[edge2];
   y0=sc->y[edge2];
   while (!finished && i> 0){
     if (sc->r[i] > 0){
       dist= hypot(sc->x[i]-x0,sc->y[i]-y0);
	 if (dist < *wmin){
	   *wmin=dist;
	   *pmin=i;
	   if (dist < wrob){
	     result=0;
	     finished=1;
	   }
	}
      }
      i--;
    }
  return result;
}

int findbesthole(scantype * sc,double wrob,int start,int *edgeright,int*edgeleft){
  int istart,p1,p2,finished=0,h1,h2,result,type;
  double yhole=1e38,wmin;
   istart=0;
   result=0;
   type=findhole(sc,wrob,istart,edgeright,edgeleft);
   if (type==0)
      result=0;
   else if (type==1)
      result=1;
   else
     while (!finished){
       type=findhole(sc,wrob,istart,edgeright,edgeleft);
       if (type==0 || type==3)
         finished=1;
       else if (type==2)
          istart=*edgeleft; 
       else{  
         if (searchback(sc,*edgeright,*edgeleft,wrob,&wmin,&p1)){
         if(searchfwd(sc,p1,*edgeleft,wrob,&wmin,&p2)){
            result=4;
	    if (fabs((sc->y[p1]+sc->y[p2])/2) < yhole){
	        yhole=fabs((sc->y[p1]+sc->y[p2]))/2;
		h1=p1;
		h2=p2;
	    }
	    istart=p2 ;
          } 
          else {
           istart=*edgeleft;
          }
  	}
        else 
          istart=*edgeleft;
      }
  }
  *edgeleft=h2;
  *edgeright=h1;
  return result;
  
}

double centreline(scantype sc,posetype startline,int minpleft,int maxpleft,int minpright,int maxpright,posetype *line){
double a,b,c,d,dd,minpos,maxneg;
int i;
a=-sin(startline.th);
b=cos(startline.th);
c=-(a*startline.x+b*startline.y);
//printf("a %f b %f c %f\n",a,b,c);
minpos=1e38;
for (i=minpleft;i<=maxpleft;i++){
  d=a*sc.x[i]+b*sc.y[i]+c;
  if (d < minpos)
    minpos=d;
}

maxneg=-1e38;
for (i=minpright;i<=maxpright;i++){
  d=a*sc.x[i]+b*sc.y[i]+c;
  if (d > maxneg)
    maxneg=d;
}
//printf("minpos %f maxneg %f \n",minpos,maxneg);
if (maxneg<minpos){
  dd=(minpos+maxneg)/2.0;
  line->x=startline.x+dd*a;
  line->y=startline.y+dd*b;
  line->th=startline.th;
}
return 0.5*(minpos-maxneg);
}

double centreline1(scantype sc,posetype startline,posetype *line){
double a,b,c,d,dd,minpos,maxneg;
int i;
a=-sin(startline.th);
b=cos(startline.th);
c=-(a*startline.x+b*startline.y);
//printf("a %f b %f c %f\n",a,b,c);
minpos=1e38;
maxneg=-1e38;
for (i=0;i<sc.nactual;i++){
  if (sc.r[i]>0){
    d=a*sc.x[i]+b*sc.y[i]+c;
    if (d >= 0){
      if (d < minpos) minpos=d;
    }
    else {
      if (d > maxneg) maxneg=d;
    }
  }    
}
//printf("minpos %f maxneg %f \n",minpos,maxneg);
  dd=(minpos+maxneg)/2.0;
  line->x=startline.x+dd*a;
  line->y=startline.y+dd*b;
  line->th=startline.th;
  return 0.5*(minpos-maxneg);
}


double bestline(scantype sc,posetype startline,posetype *line,double dth,double resolution){
double maxw,th,th0,w;
posetype tmpline;
maxw=0;
th0=startline.th;
for (th=th0-dth;th<th0+dth;th+=resolution){
  startline.th=th;
  w=centreline1(sc,startline,&tmpline);
  if (w>maxw){
    maxw=w;
    *line=tmpline;
  }    
}

  return maxw;
}
