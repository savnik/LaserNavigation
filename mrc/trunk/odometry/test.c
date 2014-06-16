#include <stdio.h>
#include <math.h>
#include "odometry.h"

int main(void){
double xcw,xccw,ycw,yccw,L,dr,wb;
dr=1;
wb=0.26;
printf("enter xcw,xccw,ycw,yccw,L \n");
scanf("%lf %lf %lf %lf %lf",&xcw,&xccw,&ycw,&yccw,&L);

dr=1;
wb=0.26;
UMB_updatexy(&wb,&dr,xcw,ycw,xccw,yccw,L);
printf("wb= %lf  dr= %lf \n",wb,dr);
dr=1;
wb=0.26;
UMB_updatey(&wb,&dr,ycw,yccw,L);
printf("wb= %lf  dr= %lf \n",wb,dr);

return(0);
}
