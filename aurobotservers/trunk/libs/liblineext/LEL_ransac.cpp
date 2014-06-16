//============================================================================
// Name        : Ransac.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include "LEL_ransac.h"
using namespace std;

inline LEL_GFLine calculateLine(double x1, double y1, double x2, double y2);
inline LEL_GFLine calculateLine(double *X, double *Y, bool* admit, int pointCount);
inline int randInRange(int l, int h);

const int RANSACMAXNOOFPOINTS = 4000;

void ransac(double* X, double* Y, int pointCount, list<LEL_GFLine> &GFLL ){

	double distThreshold=0.01; // The maximum distance between a line and a point to include the point in the line support
	int minLineSupport=max(pointCount/30,10); // The minimum number of points to constitute a line
	int minNoOfPoints= max(pointCount/10,20); // The minimum number of points to continue iterations
	int lineIter = 50; // The maximum number of line extraction iterations
	int sampleIter = 20; // The number of random pairs drawn to find a line candidate

	bool admit[RANSACMAXNOOFPOINTS];

	if(pointCount>RANSACMAXNOOFPOINTS) {
		printf("RANSAC: Warning, number of points (%d) is larger than the specified maximum = %d",pointCount, RANSACMAXNOOFPOINTS);
		return;
	}

	for(int lIn = 0; lIn < lineIter; lIn++) {
		if(pointCount<minNoOfPoints) {
			break;
		}
		int maxSupport = 0;
		LEL_GFLine bestLine;
		for(int sIn = 0; sIn<sampleIter; sIn++){
	        int in1 = randInRange(0,pointCount);
	        int in2 = randInRange(0,pointCount);
	        while(in1==in2) in2 = randInRange(0,pointCount);
	        LEL_GFLine cl = calculateLine(X[in1],Y[in1],X[in2],Y[in2]);
	        int support = 0;
	        for(int pIn = 0; pIn<pointCount; pIn++) {
	        	double dist = cl.A*X[pIn]+cl.B*Y[pIn]+cl.C;
	        	if(fabs(dist)<distThreshold) support++;
	        }
	        if(support>maxSupport) {
	        	maxSupport = support;
	        	bestLine = cl;
	        }
		}


		if(maxSupport>minLineSupport) {
			int tmpSupport =0;
			for(int pIn = 0; pIn<pointCount; pIn++) {
				double dist = bestLine.A*X[pIn]+bestLine.B*Y[pIn]+bestLine.C;
				admit[pIn]=fabs(dist)<distThreshold;
				if(fabs(dist)<distThreshold) tmpSupport++;
			}
			LEL_GFLine newLine;
			int newLineSupport;
			while(1) {
				newLine = calculateLine(X,Y,admit,pointCount);
				bool changeFlag = false;
				newLineSupport =0;
				for(int pIn = 0; pIn<pointCount; pIn++) {
					double dist = newLine.A*X[pIn]+newLine.B*Y[pIn]+newLine.C;
					bool newAdmit = fabs(dist)<distThreshold;
					if(newAdmit) newLineSupport++;
					if(newAdmit!=admit[pIn]) changeFlag=true;
					admit[pIn]=newAdmit;
				}

				if(!changeFlag) break;
			}

			int startInd = 0, endInd = pointCount-1;

			while(1) {
				while(!admit[startInd]) startInd++;
				while(admit[endInd]) endInd--;
				if(startInd>endInd) break;
				swap(X[startInd], X[endInd]);
				swap(Y[startInd], Y[endInd]);
				startInd++; endInd--;
			}

			newLine.edgeCount=newLineSupport;
			newLine.edgesX=&(X[pointCount-newLineSupport]);
			newLine.edgesY=&(Y[pointCount-newLineSupport]);
			GFLL.push_back(newLine);

			pointCount-=newLineSupport;
		}
	}
}

inline int randInRange(int l, int h) {
	int r = h-l;
	return (l+int(r*(rand()/(RAND_MAX + 1.0))));
}

inline LEL_GFLine calculateLine(double x1, double y1, double x2, double y2) {
	float ex=x1+x2;
	float ey=y1+y2;
	float ex2=x1*x1+x2*x2;
	float ey2=y1*y1+y2*y2;
	float exy=x1*y1+x2*y2;

	double th=atan2(2*ex*ey-4*exy,ex*ex-ey*ey-2*(ex2-ey2))/2;
	double R=(ex/(float)2)*cos(th)+(ey/(float)2)*sin(th);

	double A = cos(th);
	double B = sin(th);

	if(A<0) {
		return LEL_GFLine(-A,-B,R);
	} else {
		return LEL_GFLine(A,B,-R);
	}
}

inline LEL_GFLine calculateLine(double *X, double *Y, bool* admit, int pointCount) {
	float ex=0, ey=0, ex2=0, ey2=0, exy=0, xc, yc, lb, le;
	int n=0, PC =pointCount;
	while(PC--) {
		if(admit[PC]) {
			xc=X[PC];
			yc=Y[PC];
			ex+=xc;
			ex2+=xc*xc;
			ey+=yc;
			exy+=xc*yc;
			ey2+=yc*yc;
			n++;
		}
	}
	double th=atan2(2*ex*ey-2*n*exy,ex*ex-ey*ey-n*(ex2-ey2))/2;
	double R=(ex/(float)n)*cos(th)+(ey/(float)n)*sin(th);

	double A = cos(th);
	double B = sin(th);
	double C;

	PC = pointCount-1;
	lb=le= X[PC]*B-Y[PC]*A;

	if(A<0) {
		A = -A;
		B = -B;
		C = R;
	} else {
		A = A;
		B = B;
		C = -R;
	}

	while(--PC) {
		if(admit[PC]) {
		double l=X[PC]*B-Y[PC]*A;
		if(l>le) le=l;
		else if(l<lb) lb=l;
		}
	}

	return LEL_GFLine(A,B,C,lb,le);

}
