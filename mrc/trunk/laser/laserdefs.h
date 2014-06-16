#include "mobilerobot.h"
#ifndef LASERDEFS_H
#define LASERDEFS_H

/*
 * For C++ compilers, use extern "C"
 */
 
#ifdef __cplusplus
extern "C" {
#endif


typedef struct{
		double *r,*th,*x,*y;
		int nactual,nmax;
		}scantype;
		


int searchfwd(scantype *sc,int edge1,int edge2,double wrob, double *wmin, int *pmin);
int searchback(scantype *sc,int edge1,int edge2,double wrob, double *wmin, int *pmin);
double centreline(scantype sc,posetype startline,int minpleft,int maxpleft,int minpright,int maxpright,posetype *line);
int findhole(scantype * sc,double w,int start,int *edgeright,int*edgeleft);
double centreline1(scantype sc,posetype startline,posetype *line);
int findbesthole(scantype * sc,double wrob,int start,int *edgeright,int*edgeleft);
double bestline(scantype sc,posetype startline,posetype *line,double dth,double resolution);




/*
 * end block for C++
 */
#ifdef __cplusplus
}
#endif


#endif  /* LASERDEFS_H */

