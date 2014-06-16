#include  <math.h>

#ifndef MOBILEROBOT_H
#define MOBILEROBOT_H

typedef struct{
		double x,y,th;
  	      }posetype;
	      
typedef struct{
	double x,y,z,roll,pitch,yaw;
	}pose3dtype;	      

void normalizeangle(double *angle);

double limit(double v,double limv_hi,double limv_lo);

double sign(double x);

posetype posediff(posetype p1,posetype p2);

typedef struct{
		posetype *data;
		int N;
		double res;
		posetype line;
		}linregtype;
void linreg(linregtype *p);

#endif //MOBILEROBOT_H
