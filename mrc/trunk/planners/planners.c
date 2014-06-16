
#include <stdio.h>
#include <math.h>
#include "planners.h"


double findroutexy(graphtype *G, double startx, double starty, double goalx, double goaly,posetype *route,int * routestart){
  int sp,spa,gp,gpa,scon, gcon,goalp,i,p,startp,arcgoal=0;
  double cost,sxa,sya,sxp,syp,gxa,gya,gyp,gxp,dminp,dmina;
  dminp=findnearestpoint(G,startx,starty,&sxp,&syp,&sp);
  dmina=findarc(G,startx,starty,&sxa,&sya,&spa,&scon);
  if (dmina <dminp){
    addp(G,0,sxa,sya);
    addcon(G,0,spa,hypot(G->nodes[spa].x-G->nodes[0].x,G->nodes[spa].y-G->nodes[0].y));
    addcon(G,0,scon,hypot(G->nodes[scon].x-G->nodes[0].x,G->nodes[scon].y-G->nodes[0].y));
    printf("p1 %d  p2 %d \n",spa,scon);
    startp=0;
  }
  else {
    startp=sp;
    sxa=sxp;
    sya=syp;
  }
  dminp=findnearestpoint(G,goalx,goaly,&gxp,&gyp,&gp);
  dmina=findarc(G,goalx,goaly,&gxa,&gya,&gpa,&gcon);
  if (dmina < dminp){
    goalp=G->Nnodes-1;
    addp(G,goalp,gxa,gya);
    addcon(G,gpa,goalp,hypot(G->nodes[gp].x-G->nodes[goalp].x,G->nodes[gp].y-G->nodes[goalp].y));
    addcon(G,gcon,goalp,hypot(G->nodes[gcon].x-G->nodes[goalp].x,G->nodes[gcon].y-G->nodes[goalp].y));
    arcgoal=1;
  }
  else {
    goalp=gp;
    gxa=gxp;
    gya=gyp;
  } 
  printf("startpoint %f %f %d \n",sxa,sya,startp); 
  printf("goalpoint %f %f %d \n",gxa,gya,goalp);
  #if (0)
    {
      int i;
      for (i=0;i<5;i++){
      printf("%d %d %f %f\n" , G->nodes[i].conlist[0],G->nodes[i].conlist[1],G->nodes[i].costlist[0],G->nodes[i].costlist[1]);
      }
    }
  #endif

  cost=findroute(startp,goalp,G);
  //printf("after findroute\n");

  i=0;
  p=goalp;
  route[0].x=gxa;route[0].y=gya;

  while ((p=G->nodes[p].fromnode)!=-1)
  { // copy found route to user structure
    route[i+1].x=G->nodes[p].x;
    route[i+1].y=G->nodes[p].y;
    route[i].th=atan2(route[i].y-route[i+1].y,route[i].x-route[i+1].x);
    i++;
  }
  route[i].th=route[i-1].th;
  // report first node
  *routestart=i;  
  // remove extra start and nodes - if used
  if (arcgoal){
    // extra node has beed added at goal position,
    // remove it
    removecon(G,gpa,goalp);
    removecon(G,gcon,goalp);
    removep(G,goalp);
  }
  if (spa > 0)
  { // extra start connections has beed added to node 0
    // remove it
    removecon(G,0,spa);
    removecon(G,0,scon);
    removep(G,0);
  }
  return cost;
}



void calculatecost(graphtype *G){
int i,j,c;
double x0,y0,x1,y1;
for (i=0;i< G->Nnodes;i++){
  if (G->nodes[i].initialized){
    j=0;
    x0=G->nodes[i].x;
    y0=G->nodes[i].y;
    while ((c=G->nodes[i].conlist[j])!=-1){
      x1=G->nodes[c].x;
      y1=G->nodes[c].y;
      G->nodes[i].costlist[j]=hypot(x1-x0,y1-y0);
      j++;
    }	
  }
 }
}	


double findarc(graphtype *G,double xr, double yr, double *xa, double *ya,int *p,int *con){
int i,j,c;
double d2min=1e38,x0,y0,x1,y1,vx,vy,d2,t,v2;
*p=-1;
for (i=0;i< G->Nnodes;i++){
  if (G->nodes[i].initialized){
    j=0;
    x0=G->nodes[i].x;
    y0=G->nodes[i].y;
    while ((c=G->nodes[i].conlist[j])!=-1){
      x1=G->nodes[c].x;
      y1=G->nodes[c].y;
      vx=(x1-x0);
      vy=(y1-y0);
      v2=(vy*vy+vx*vx);
      d2=fabs((-vy*(xr-x0)+vx*(yr-y0)))/sqrt(v2);
       
   //   printf("i %d j %d c %d d %f t %f %f %f %f %f %f %f \n",i,j,c,(d2),t,x0,y0,x1,y1,xr,yr);
      if ((d2 < d2min) ){
        t=-((x0-xr)*vx+(y0-yr)*vy)/v2;
	if ((t>=0) && (t <=1)){
	   d2min=d2;
	   *xa=x0+t*vx;
	   *ya=y0+t*vy;
	   *p=i;
	   *con=c;
	}
      }
      j++;
    }	
  }
 }
 return d2min;
}	

double findnearestpoint(graphtype *G,double xr, double yr, double *xa, double *ya,int *p){
int i;
double dmin=1e38,x0,y0,d;

for (i=0;i< G->Nnodes;i++){
  if (G->nodes[i].initialized){    
    x0=G->nodes[i].x;
    y0=G->nodes[i].y;
    d=hypot((x0-xr),(y0-yr));
    if (d < dmin){
      dmin=d;
      *p=i;
   }
  }
 }
 *xa=G->nodes[*p].x;
 *ya=G->nodes[*p].y;	
 return dmin;	
} 
     
void removep(graphtype * G,int id){
  G->nodes[id].initialized=0;
}

void removecon(graphtype *G, int id, int conid){
int j,c;
j=0;
while (((c=G->nodes[id].conlist[j])!=conid) && (c!=-1))
  j++;
if (c==conid){
  do {
    G->nodes[id].conlist[j]= G->nodes[id].conlist[j+1];
    G->nodes[id].costlist[j]=G->nodes[id].costlist[j+1];
    j++;
  }while( G->nodes[id].conlist[j]!=-1);
}
}



void addp(graphtype * G,int id, double x, double y){
if (id >-1 && id < G->Nnodes){
	G->nodes[id].x=x;
	G->nodes[id].y=y;
	G->nodes[id].initialized=1;
   }
}

void addcon(graphtype *G, int id, int conid,double cost){
int j;
if (id >-1 && id < G->Nnodes && G->nodes[id].initialized &&
	id >-1 && conid < G->Nnodes && G->nodes[conid].initialized){
	j=0;
	while (G->nodes[id].conlist[j]!=-1)
  		j++;
	G->nodes[id].conlist[j]=conid;
	G->nodes[id].conlist[j+1]=-1;
	G->nodes[id].costlist[j]=cost;
    }
}

void insert(int node,graphtype *G){
    if (G->slisthead==-1){
      G->slisthead=node;
      G->nodes[node].slistprenode=-1;
      G->nodes[node].slistnextnode=-1;
      G->nodes[node].inlist=1;
    }
    else {
      if (G->nodes[node].inlist==0){
        G->nodes[node].inlist=1;
        G->nodes[node].slistprenode=-1;
        G->nodes[node].slistnextnode= G->slisthead; 
        G->nodes[G->slisthead].slistprenode=node;     
        G->slisthead=node;
      }
    }
}


int getmin(graphtype * G){
  int mnode,c,p,p1;
  double mincost;
  if ((mnode=G->slisthead)==-1)
     return -1;
  else {
    mincost=G->nodes[mnode].nodecost;
    c=mnode;
    while ((c=G->nodes[c].slistnextnode)!=-1){
      if (mincost > G->nodes[c].nodecost){
         mincost=G->nodes[c].nodecost;
	 mnode=c;
      }
    }
    if ((p=G->nodes[mnode].slistprenode)==-1){
      G->slisthead= G->nodes[mnode].slistnextnode;
      if (G->slisthead!=-1)
        G->nodes[G->slisthead].slistprenode=-1;
    }
    else {
      p1= G->nodes[mnode].slistnextnode;
      G->nodes[p].slistnextnode=p1;
      if (p1 !=-1)
        G->nodes[p1].slistprenode=p;
    }
    G->nodes[mnode].inlist=0;
    return mnode;
  }
}
  


double findroute(int startnode, int targetnode,graphtype * G){
int i,v,w,j;
double newcost;
// initialize nodecost
for (i=0;i<G->Nnodes;i++){
  G->nodes[i].nodecost=1.0e6;
  G->nodes[i].inlist=0;
}
G->nodes[startnode].nodecost=0;
G->nodes[startnode].fromnode=-1;
G->slisthead=-1;
insert(startnode,G);
while ( (v=getmin(G))!=targetnode && v!=-1 ){
   j=0;
  // printf("v %d \n",v);
   while ((w=G->nodes[v].conlist[j])!=-1){
     newcost=G->nodes[v].nodecost+G->nodes[v].costlist[j];
     if (newcost < G->nodes[w].nodecost){
        G->nodes[w].nodecost=newcost;
	G->nodes[w].fromnode=v;
	insert(w,G); 
	//printf("w %d\n",w);
     }
    j++;
   }
  // printf("cost %f %f %f %f %f\n",G->nodes[1].nodecost,G->nodes[2].nodecost,G->nodes[3].nodecost,
  //         G->nodes[4].nodecost,  G->nodes[5].nodecost );
  // printf("lh %d \n",G->slisthead);
  // for (k=1;k<6;k++)
  //   printf("p %d n%d \n",G->nodes[k].slistprenode,G->nodes[k].slistnextnode);
}
if (v==-1)
  return -1;
 else
   return G->nodes[v].nodecost;
}	
