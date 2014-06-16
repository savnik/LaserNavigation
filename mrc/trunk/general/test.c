#include <math.h>
#include "mobilerobot.h"
#include <stdio.h>

int main(void){
linregtype d;
posetype data[10];
data[0].x=0;data[0].y=1;
data[1].x=1;data[1].y=2;
data[2].x=2;data[2].y=3;
data[3].x=3;data[3].y=4;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI);
data[0].x=1;data[0].y=1;
data[1].x=1;data[1].y=2;
data[2].x=1;data[2].y=3;
data[3].x=1;data[3].y=4;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI);
data[0].x=0;data[0].y=1;
data[1].x=1;data[1].y=3;
data[2].x=2;data[2].y=5;
data[3].x=3;data[3].y=7;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI,atan2(2,1)*180/M_PI);
data[0].x=0;data[0].y=7;
data[1].x=1;data[1].y=5;
data[2].x=2;data[2].y=3;
data[3].x=3;data[3].y=1;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI,atan2(2,1)*180/M_PI);
data[0].x=0;data[0].y=4;
data[1].x=1;data[1].y=3;
data[2].x=2;data[2].y=2;
data[3].x=3;data[3].y=1;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI);
data[0].x=0;data[0].y=0;
data[1].x=.1;data[1].y=-1;
data[2].x=.2;data[2].y=-2;
data[3].x=.3;data[3].y=-3;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI);

data[0].x=0;data[0].y=0;
data[1].x=-.1;data[1].y=-1;
data[2].x=-.2;data[2].y=-2;
data[3].x=-.3;data[3].y=-3;
d.N=4;
d.data=data;
linreg(&d);
printf("x y th %lf %lf %lf \n",d.line.x,d.line.y,d.line.th*180/M_PI);


return 0;


}
