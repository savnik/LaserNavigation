#include "smrsim.h"
#include "smrsdl.h"
#include <stdio.h>
#include <unistd.h>
#include "polyline.h"
#include "xfigreader.h"
#include "intersect.h"
simtype sim;
struct smr simrobot; 
FILE  *fdata;
polylinestype lines=NULL;
intersectionstype is;
int ls[SMR_LS_N];

struct smr* smr_connect(char * hostname, int port){
sim.pose.x=0;
sim.pose.y=0;
sim.pose.th=0;
sim.ts=0.01;
sim.w=0.26;
sim.cr=0.01;
sim.cl=0.01;
sim.encr=10000;
sim.encl=10000;
simrobot.left.encoder=0;
simrobot.right.encoder=0;
fdata=fopen("smrsim.dat","w");
smrsdl_init(300, 300, -2.0, 2.0, -2.0, 2.0);
lines=xfr_init("test.fig");
polyline_print(lines); 
return(&simrobot);
}

void smr_disconnect(struct smr *p){
 // Nils: dette er ikke et optimalt sted til kaldene, men nu er
 // de alstaa her... Mvh Skjalm
 smrsdl_update();
 smrsdl_waitforkeypress();
 fclose(fdata);
 polyline_free(&lines);
}
 

int smr_write(struct smr * p){
 sim.vleft=p->left.speed*sim.cl;
 sim.vright=p->right.speed*sim.cr;
 return(0);
}

int smr_read(struct smr * p){
double dr,dl,d;
int count;
static int j=0;
int intersection = -1,i;
j++;
dl=sim.vleft*sim.ts;
dr=sim.vright*sim.ts; 
sim.lrnd+=dl;
count = sim.lrnd*sim.encl;
p->left.encoder+=count;
sim.lrnd-=count/sim.encl;
sim.rrnd+=dr;
count = sim.rrnd*sim.encr;
p->right.encoder+=count;
sim.rrnd-=count/sim.encr;
d = (dl+dr)/2;
sim.pose.x+=d*cos(sim.pose.th);
sim.pose.y+=d*sin(sim.pose.th); 
sim.pose.th+=(dr-dl)/sim.w;
normalizeangle(&sim.pose.th);

intersection = intersect_linesensor(lines,sim.pose,0.3,0.3,&is,ls);
for (i=0;i<8;i++){
  p->ls[7-i]=ls[i];
  p->ir[i]=30;
}
#if (0)
printf("intersections: %d ( %d )\n",intersection,is.intersections);
for (i = 0; i<is.intersections; i++)
  printf("Intersection type: %d ( %d , %d , %f )\n",is.types[i],is.lines[i],is.points[i],is.c[i]);
for (i = 0; i < SMR_LS_N; i++)
  printf("linesensor[%d] = %d\n",i,ls[i]);
#endif
if (j==5){
  fprintf(fdata," %f %f %f
  \n",sim.pose.x,sim.pose.y,sim.pose.th);
  smrsdl_drawpolylines(lines);
  smrsdl_drawline(&sim);
  j=0;
}
//usleep(10000);
return(0);
}
