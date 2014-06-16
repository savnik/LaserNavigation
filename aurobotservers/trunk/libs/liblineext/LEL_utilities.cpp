/*
 * LEL_utilities.cpp
 *
 *  Created on: 05/08/2010
 *      Author: eba
 */

#include "LEL_utilities.h"

void arrangeGLines(void *edgesarr, list<LEL_GroupedLine *> &GLPL, int minLen, CvScalar color, bool keepImage) {

	IplImage edgesStub, *edges = (IplImage *) edgesarr;

	edges = cvGetImage(edges,&edgesStub);

	LEL_GroupedLine curLine;
	list<LEL_GroupedLine *>::iterator it=GLPL.begin();
	CvPoint Cb,Ce;

	if(!keepImage) memset(edges->imageData,0,edges->imageSize);

	for(; it!=GLPL.end(); it++) {
		if((*it)->edgeCount<minLen) continue;
		curLine=*(*it);
		Cb=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.lb), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.lb));
		Ce=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.le), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.le));

		cvLine(edges,Cb,Ce,color,1);

	}
}
