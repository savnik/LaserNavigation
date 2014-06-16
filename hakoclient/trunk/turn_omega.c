#include <math.h>

#include "turn_omega.h"

/* Code moved from "main.c" to separate file 2007jun10. */

int TURN_omega (const pointtype dmp_i,  /* just passed position, angle is from last position */
		const pointtype dmp_ip1,   /* end of turn position, angle is from 'dmp_i' */
		const pointtype dmp_ip2)   /* next row position */
{
  double dist = 0, x=0, x1=0, y=0, y1=0, l=0, anglefrom=0, angleto=0,angleend=0,dl;
  char str[128];
  y = dmp_i.n - dmp_ip1.n;
  x = dmp_i.e - dmp_ip1.e;
  dist = sqrt(pow(x, 2) + pow(y, 2)); /* distance between start and end of turn */
  angleto = dmp_ip1.ang; /* angle along headland */
  anglefrom = dmp_i.ang; /* angle along last row-line */
  angleend=dmp_ip2.ang; /* angle along next row-line */
  
  /* tight omega turn */
  if (dist < 2 * MAXTURNRAD) {
    l = MAXTURNRAD * sin (acos (dist / (2.0 * MAXTURNRAD)));
    y1 = dmp_i.n + l * sin (anglefrom / 180.0 * M_PI);
    x1 = dmp_i.e + l * cos (anglefrom / 180.0 * M_PI);
    dl=(MAXTURNRAD - dist / 2.0 );
    y = y1 - dl* sin (angleto / 180.0 * M_PI);
    x = x1 - dl * cos (angleto / 180.0 * M_PI);    
    sprintf(str,"drive %f %f %f @v%f @a%f : ($targetdist < 0)\n",x,y,anglefrom,TURNSPEED,TURNACC);
    cl_send(str);
    y1 = dmp_i.n + (MAXTURNRAD +l) * sin (anglefrom / 180.0 * M_PI);
    x1 = dmp_i.e + (MAXTURNRAD +l) * cos (anglefrom / 180.0 * M_PI);
    y = y1 + (dist / 2.0) * sin (angleto / 180.0 * M_PI);
    x = x1 + (dist / 2.0) * cos (angleto / 180.0 * M_PI);
    sprintf(str,"drive %f %f %f @v%f @a%f : ($targetdist < 0)\n",x,y,angleto,TURNSPEED,TURNACC);
    cl_send(str);
		/*
		The last line in the same direction as the next row-line is not needed, better to
		let drive take the fastest route to the new row-line to follow. 
    y1 = dmp_i.n + l * sin (anglefrom / 180.0 * M_PI);
    x1 = dmp_i.e + l * cos (anglefrom / 180.0 * M_PI);
    y = y1 + (MAXTURNRAD + dist / 2.0) * sin (angleto / 180.0 * M_PI);
    x = x1 + (MAXTURNRAD + dist / 2.0) * cos (angleto / 180.0 * M_PI);
    sprintf(str,"drive %f %f %f @v%f @a%f : ($targetdist < 0)\n",x,y,angleend,TURNSPEED,TURNACC);
    cl_send(str);
		*/
  } else {
    /* large omega turn */
      y1 = dmp_i.n + MAXTURNRAD * sin (anglefrom / 180.0 * M_PI);
      x1 = dmp_i.e + MAXTURNRAD * cos (anglefrom / 180.0 * M_PI);
      y = y1 + (dist -MAXTURNRAD) * sin (angleto / 180.0 * M_PI);
      x = x1 + (dist -MAXTURNRAD) * cos (angleto / 180.0 * M_PI);
      driveto (y, x, angleto, TURNSPEED, TURNACC);
     }
  
  driveto(dmp_ip1.n, dmp_ip1.e, dmp_ip2.ang, TURNSPEED,TURNACC);
  
  return 0;
}
