/*
 * LEL_structure.cpp
 *
 *  Created on: 15/12/2009
 *      Author: eba
 */

//#define OPENCV2
#ifdef OPENCV2
#include <opencv2/core/core_c.h>
#else
#include <opencv/cxcore.h>
#endif
#include "LEL_hough.h"

int _main() {
return 0;
}

ImageInfo::ImageInfo(int width, int height, float focalDist) {

	this->width = width;
	this->height = height;
	this->focalDist = focalDist;
	centerX = (float)(width-1)/2;
	centerY = (float)(height-1)/2;

}


void freeDistributedEdges(LEL_DistributedEdges *DE) {
	int i;
	int regionCount=DE->vertReg*DE->horReg;
	for (i=0; i<regionCount;i++) {
		cvFree((void **)&(DE->regions[i].edges));
	}
	cvFree((void **)&(DE->regions));
}

void cloneDistributedEdges(LEL_DistributedEdges *DEin,LEL_DistributedEdges *DEout) {
	int i;
	int regionCount=DEin->vertReg*DEin->horReg;
	*DEout=*DEin;
	DEout->regions=(LEL_ImRegion *) cvAlloc(regionCount*sizeof(LEL_ImRegion));
	for (i=0; i<regionCount;i++) {
		DEout->regions[i]=DEin->regions[i];
		DEout->regions[i].edges=(LEL_IndexedPoint *)cvAlloc(DEout->regions[i].edgeCount*sizeof(LEL_IndexedPoint));
		memcpy(DEout->regions[i].edges,DEin->regions[i].edges,DEout->regions[i].edgeCount*sizeof(LEL_IndexedPoint));
	}
}

void freeDistributedLines(DistributedLines *DL) {
	for(int i=0;i<DL->rows*DL->columns;i++)
		if(DL->lineCounts[i]) cvFree((void **)((LEL_LineSegment **)(DL->lines)+i));
	cvFree((void **)&(DL->lines));
	cvFree((void **)&(DL->lineCounts));

}

LEL_FloatingPoint floatingPoint(float x, float y) {

	LEL_FloatingPoint result;
	result.x = x;
	result.y = y;
	return result;
}
