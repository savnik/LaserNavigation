#include "smrsim.h"
#include "smrsdl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include "polyline.h"
#include "xfigreader.h"
#include "intersect.h"
#define SIMULATIONTIMEOUT 100000

siminterfacetype siminterface;
simtype sim;
struct smr simrobot; 
FILE  *fdata;
polylinestype lines=NULL;
intersectionstype is;
int ls[SMR_LS_N];
int sockfd,connected;
int runtime=0;

unsigned int tdelay = 1000000;
struct	sockaddr_in serv_adr;
#define BUFLEN	100
char	buffer[BUFLEN];
	

struct smr* smr_connect(char * hostname, int port){
/* Create endpoint */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
        printf(" sock %d \n",sockfd);
    if ( sockfd < 0 )
    {
      	perror(strerror(errno));
      	fprintf(stderr," Can not make  socket\n");
      	exit(errno);
    }
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_port	= htons(31370);
    serv_adr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if ((connected=connect(sockfd, (struct sockaddr *) &serv_adr, sizeof(serv_adr))) <0){
          printf("connect error\n");
    }
siminterface.newcmd=0;
sim.pose.x=0;
sim.pose.y=0;
sim.pose.th=0;
sim.ts=0.01;
sim.w=0.26;
sim.cr=0.01026;
sim.cl=0.01026;
sim.encr=0.0001026;
sim.encl=0.0001026;
simrobot.left.encoder=0;
simrobot.right.encoder=0;

fdata=fopen(siminterface.fname,"w");
//smrsdl_init(300, 300, -2.0, 2.0, -2.0, 2.0);
lines=xfr_init("test.fig");
polyline_print(lines); 
return(&simrobot);
}

void smr_disconnect(struct smr *p){
int len;
 // Nils: dette er ikke et optimalt sted til kaldene, men nu er
 // de alstaa her... Mvh Skjalm
// smrsdl_update();
// smrsdl_waitforkeypress();
 len = sprintf(buffer, "quit\n");
 if ( connected >-1){
   send( sockfd, buffer, len, 0);    
   close(sockfd);
 }
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
//int intersection
int i;
j++;
dl=sim.vleft*sim.ts;
dr=sim.vright*sim.ts; 
sim.lrnd+=dl;
count = sim.lrnd/sim.encl;
p->left.encoder+=count;
sim.lrnd-=count*sim.encl;
sim.rrnd+=dr;
count = sim.rrnd/sim.encr;
p->right.encoder+=count;
sim.rrnd-=count*sim.encr;
d = (dl+dr)/2;
sim.pose.x+=d*cos(sim.pose.th);
sim.pose.y+=d*sin(sim.pose.th); 
sim.pose.th+=(dr-dl)/sim.w;
normalizeangle(&sim.pose.th);

/*intersection =*/ intersect_linesensor(lines,sim.pose,0.3,0.3,&is,ls);
for (i=0;i<8;i++){
  p->ls[7-i]=ls[i];
  p->ir[i]=30;
}
// #if (0)
// printf("intersections: %d ( %d )\n",intersection,is.intersections);
// for (i = 0; i<is.intersections; i++)
//   printf("Intersection type: %d ( %d , %d , %f )\n",is.types[i],is.lines[i],is.points[i],is.c[i]);
// for (i = 0; i < SMR_LS_N; i++)
//   printf("linesensor[%d] = %d\n",i,ls[i]);
// #endif
if (siminterface.newcmd){
  fprintf(fdata,"status %s",siminterface.cmdbuf);
  fprintf(fdata,"pose %d %f %f %f \n",10*runtime,sim.pose.x,sim.pose.y,sim.pose.th);
  siminterface.newcmd=0;
}
if ((runtime % 10) ==0){
  fprintf(fdata,"pose %d %f %f %f \n",10*runtime,sim.pose.x,sim.pose.y,sim.pose.th);
}
if (j==10){
  int len;
 
//  smrsdl_drawpolylines(lines);
//  smrsdl_drawline(&sim);
    len = sprintf(buffer, "%d  %d\n",(int)(100*sim.pose.x)+200,(int)(100*sim.pose.y)+200);
    if (connected > -1)
      send( sockfd, buffer, len, 0);    
    j=0;
}
usleep(100);
runtime++;
// #if (0)
// if (runtime > SIMULATIONTIMEOUT){
//    fprintf(fdata,"status  simulation timeout");
//    exit(1);
// }
// #endif

return(0);
}

