#ifndef SMRSIM_H
#define SMRSIM_H

#include "mobilerobot.h"
#include "smr.h"
#include <math.h>
typedef
struct{
	double ts;
	double w;
	double cr;		// convertion from motercmd to m/s
	double cl;             // convertion from motercmd to m/s
	double encr;	       // driven distance pr. encoder tick m/tick	
	double encl; 		// driven distance pr. encoder tick m/tick
	posetype pose;
	double vleft,vright;
        double lrnd,rrnd;
	}simtype;

typedef struct{
        char cmdbuf[256];
        int newcmd;
        char fname[100];
	int simulating;
      }siminterfacetype;

#endif

