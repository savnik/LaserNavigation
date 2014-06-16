#ifndef OBSTACLE_H
#define OBSTACLE_H

#ifdef __cplusplus
extern "C" {
#endif



#define NOBSTMAX	500
#define DMAX		1
#define TSAMOBJ		0.2
typedef struct {double b,c,d;} robotsizetype;
typedef struct {double x,y,th;}posetype; 

void obstacle(int nobs,double d_th[][2],double pos[][2],double R,robotsizetype dim);

int findobstaclecourse(double *wc,int Nobst, double obstpos[][2],posetype tgtpose,posetype robpose,double vel,
			double deltaw,int Nradii,robotsizetype robotdim);


#ifdef __cplusplus
}
#endif

#endif
