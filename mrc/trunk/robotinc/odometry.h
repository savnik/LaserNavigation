#include "mobilerobot.h"
#ifndef ODOMETRY_H
#define ODOMETRY_H



typedef
struct{
	//  Parameters ---------------------------------------------------------------
        int 	type;
	double	cr;		// driven distance / encodertick (meter/tick) right wheel
	double	cl;		// driven distance / encodertick (meter/tick) left wheel
	double	w;		// distance between wheels
        int     enctype;        // 1=relative or 2=accumulating
	int     encwheel;	// 0=backwheel   1= frontwheel
	int     gyrotype;
        double  robotlength;
	double  steeringangleoffset;
	double	ts;		// sample time
	int	maxtick;	// max encoderticks / sample
	//  input --------------------------------------------------------------------
	int renc,lenc;		// right and left encoder values
	double  steeringangle;
	double  curvature;
	double  dth;		// angular rate 
	int	control;	//
	double  time;		//current time in seconds
	//  output -------------------------------------------------------------------
	posetype pose;		// current pose
	double	 vel;		// estimated velocity
        double   vell;
        double   velr;
	double	 dist;		// distance driven since reset
	double   distleft;
        double   distright; 
        double   dv;		
	double   dthout;
        int	 enc_error;
	int	 dencr;
	int	 dencl;
	// internal states, dont change ----------------------------------------------
	int renc_old;
	int lenc_old;
	double time_old;
	double dthold;

 	}odotype;

typedef struct updatestruct{

	int lTick;	//Left wheel encoder ticks
	int rTick;	//Right wheel encoder ticks
	
	double lK;	//Left wheel travelled distance per tick
	double rK;	//Right wheel travelled distance per tick
	double E2;	//The variance of the error introduced by K's only for a 1m translation 
	
	double B;	//The average wheel separation
	double A2;	//The variance of the error introduced by B only for a 2pi rotation
	
	double theta;	//The robot orientation change since last reset, it should start as 0
	
	// The relevant elements of the covariance matrix, 1: Theta, 2: x, 3: y
	double p11, p12, p13, p22, p23, p33;
}covtype;

void odo_init(odotype *p);

void odo_update(odotype *p);

void updatecov(covtype * us);

void resetupdatestruct(covtype * us);

void UMB_updatexy(double *wheelbase,double *diameter_ratio,
		double xcw,double ycw,double xccw,double yccw,double L);
void UMB_updatey(double *wheelbase,double *diameter_ratio,
		double ycw,double yccw,double L);



#endif // ODOMETRY_H
