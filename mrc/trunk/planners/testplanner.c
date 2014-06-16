#include "planners.h"
#include <stdio.h>
#include <stdlib.h>
int main(void){
graphtype G;
graphnode nodes[100];
int i,j,k,routestart;
double cost,xr,yr,gx,gy;
posetype route[100];
posetype p[4];

G.Nnodes=30;
G.nodes=nodes;
G.slisthead=-1;
for (i=0;i<G.Nnodes;i++){
  G.nodes[i].initialized=0;
  G.nodes[i].conlist[0]=-1;
}
#if(0)
addp(&G,1,-0.45,0);
addcon(&G,1,2,2.2);


addp(&G,2,-0.45,2.2);
addcon(&G,2,1,2.2);
addcon(&G,2,3,2.2);
addcon(&G,2,6,1);

addp(&G,3,-0.45,5);
addcon(&G,3,2,2.2);

addp(&G,4,0.45,0);
addcon(&G,4,5,0.5);

addp(&G,5,0.45,0.6);
addcon(&G,5,4,0.5);
addcon(&G,5,6,1.7);

addp(&G,6,0.45,2.2);
addcon(&G,6,5,1.7);
addcon(&G,6,7,1.7);
addcon(&G,6,2,1);
addcon(&G,6,9,1.5);

addp(&G,7,0.45,4.65);
addcon(&G,7,6,1.7);
addcon(&G,7,8,0.35);
addcon(&G,7,13,3);

addp(&G,8,0.45,5.0);
addcon(&G,8,7,0.35);

addp(&G,9,2,2.2);
addcon(&G,9,6,1.5);
addcon(&G,9,12,1.5);

addp(&G,10,3.55,0.0);
addcon(&G,10,11,0.6);

addp(&G,11,3.55,0.6);
addcon(&G,11,10,0.6);
addcon(&G,11,12,1.7);

addp(&G,12,3.55,2.2);
addcon(&G,12,11,1.7);
addcon(&G,12,13,1.7);
addcon(&G,12,16,1.0);
addcon(&G,12,9,1.5);


addp(&G,13,3.55,4.65);
addcon(&G,13,12,1.7);
addcon(&G,13,14,0.35);
addcon(&G,13,7,3.0);

addp(&G,14,3.55,5.0);
addcon(&G,14,13,0.35);

addp(&G,15,4.45,0.0);
addcon(&G,15,16,2.2);

addp(&G,16,4.45,2.2);
addcon(&G,16,15,2.2);
addcon(&G,16,17,2.8);
addcon(&G,16,12,1);

addp(&G,17,4.45,5.0);
addcon(&G,17,16,2.8);
#endif

addp(&G,1,0.38,0.38);

addp(&G,2,1.88,0.38);

addp(&G,3,3.38,0.38);

addp(&G,4,0.38,1.13);

addp(&G,5,1.13,1.13);

addp(&G,6,1.88,1.13);
addp(&G,7,3.38,1.13);
addp(&G,8,0.38,1.88);
addp(&G,9,1.13,1.88);
addp(&G,10,2.63,1.88);
addp(&G,11,0.38,2.63);
addp(&G,12,1.88,2.63);
addp(&G,13,0.38,3.38);
addp(&G,14,1.88,3.38);
addp(&G,15,2.63,3.38);
addp(&G,16,3.38,3.38);

addcon(&G,1,2,1.5);
addcon(&G,1,4,0.75);

addcon(&G,2,1,1.5);
addcon(&G,2,3,1.5);
addcon(&G,2,6,0.75);


addcon(&G,3,2,1.5);

addcon(&G,4,1,0.75);
addcon(&G,4,5,0.75);

addcon(&G,5,4,0.75);
addcon(&G,5,9,0.75);

addcon(&G,6,2,0.75);
addcon(&G,6,7,1.5);

addcon(&G,7,6,1.5);
addcon(&G,7,16,2.25);

addcon(&G,8,9,0.75);
addcon(&G,8,11,0.75);

addcon(&G,9,8,0.75);
addcon(&G,9,5,0.75);
addcon(&G,9,10,1.5);

addcon(&G,10,9,1.5);
addcon(&G,10,15,1.5);

addcon(&G,11,8,0.75);
addcon(&G,11,12,1.5);

addcon(&G,12,11,1.5);
addcon(&G,12,14,0.75);


addcon(&G,13,14,1.5);

addcon(&G,14,12,0.75);
addcon(&G,14,13,1.5);

addcon(&G,15,10,1.5);
addcon(&G,15,16,0.75);

addcon(&G,16,15,0.75);
addcon(&G,16,7,2.25);


p[0].x=3.39;
p[0].y=0.37;

p[1].x=0.37;
p[1].y=0.37;

p[2].x=0.37;
p[2].y=3.38;

p[3].x=3.39;
p[3].y=3.38;
j=0;
k=1;
calculatecost(&G);
while (1){

printf("enter xs and ys  xg   yg\n");
//scanf(" %d %d",&start,&goal);
scanf(" %lf %lf %lf %lf",&xr,&yr,&gx,&gy);

//xr=p[3].x+rand()*0.1/RAND_MAX-0.05;
//yr=p[3].y+rand()*0.1/RAND_MAX-0.05;
//gx=p[0].x+rand()*0.1/RAND_MAX-0.05;
//gy=p[0].y+rand()*0.1/RAND_MAX-0.05;
cost=findroutexy(&G,xr,yr,gx,gy,route,&routestart);

printf("after findroutexy\n");
//getchar();

//cost=findroute(start,goal,&G);
if (cost >0){
  for(i=routestart;i >=0; i--){
    printf("%f %f %f \n",route[i].x,route[i].y,route[i].th);
    }
  printf("cost= %lf \n", cost);
} 
j++;
if (j >3) j=0;
k++;
if (k>3)k=0;
getchar();
#if(0)
for (i=0;i<G.Nnodes;i++){
printf("x %f  y %f  con %d %d %d %d %d cost %f %f %f %f ini %d \n",
       G.nodes[i].x , G.nodes[i].y,  G.nodes[i].conlist[0], G.nodes[i].conlist[1],G.nodes[i].conlist[2], G.nodes[i].conlist[3] ,G.nodes[i].conlist[4],
        G.nodes[i].costlist[0], G.nodes[i].costlist[1],G.nodes[i].costlist[2], G.nodes[i].costlist[3],  G.nodes[i].initialized);
	}
#endif         
}

return 0;
}
