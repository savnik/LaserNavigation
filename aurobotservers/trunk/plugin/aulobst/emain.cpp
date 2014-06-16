/*
 * LEL_main.cpp
 *
 *  Created on: 17/12/2009
 *      Author: eba
 */

#include <list>
#include <cstdlib>
#include <iostream>
using namespace std;

#include "LEL_ransac.h"

int main()
{
	// Testing Ransac
	int pointCount = 600;

	double X[pointCount];
	double Y[pointCount];

	for(int i=0; i<pointCount; i++) {
		X[i] = i/(double)100;
		if(i<pointCount/3)
			Y[i] = X[i];
		else if(i<2*pointCount/3)
			Y[i] = -X[i]+5;
		else
			Y[i] = 4*X[i]-10;
	}

	list<LEL_GFLine> GFLL;

	ransac(X,Y,pointCount,GFLL);
	printf("Ransac results:");
	for(list<LEL_GFLine>::iterator it = GFLL.begin(); it!=GFLL.end(); it++) {
		printf("\nA:%f B:%f C:%f startX:%f startY:%f endX:%f endY:%f support:%d ", (*it).A,(*it).B,(*it).C,(*it).startX,(*it).startY,(*it).endX,(*it).endY,(*it).edgeCount);
	}

	return 0;
}
