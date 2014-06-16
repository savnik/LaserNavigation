#ifndef LEL_UTILITIES_H_
#define LEL_UTILITIES_H_

#include <iostream>
#include <list>
#include "LEL_commons.h"
#include "LEL_quadtree.h"
#include "LEL_hough.h"

using namespace std;

/*void arrangeEdges(IplImage* edges, DistributedEdges* DistEdges) {

	CV_FUNCNAME( "cvArrange" );

	__BEGIN__;

	if(edges->depth!=IPL_DEPTH_8U || edges->nChannels!=1)
		CV_ERROR(CV_StsUnsupportedFormat, "");

	int i,j,k,beginIndex;
	int widthStep = edges->widthStep;
	int regSize=DistEdges->regionSize;
	int regStep = edges->widthStep*regSize;
	char * data = edges->imageData;

	memset(data,0,edges->imageSize);
	
	ImRegion *regions=DistEdges->regions;
	ImRegion *thisRegion;

	for(i=0;i<DistEdges->vertReg;i++)
	{
		for(j=0;j<DistEdges->horReg; j++) {
			beginIndex = i*regStep+j*regSize;
			thisRegion = &regions[DistEdges->horReg*i+j];
			for(k=0;k<thisRegion->edgeCount;k++) {
				IndexedPoint point = thisRegion->edges[k];
				data[beginIndex+point.x+point.y*widthStep]=255;
			}
		}
	}	

	__END__;

}

void arrangeLines(IplImage *edges, Line ** lines, int *lineCounts,int rows, int columns, int regSize) {
	
    CvMat* canvas = cvCreateMat( regSize, regSize, CV_8U );
	Line curLine;
	CvPoint Cb,Ce;
	
	memset(edges->imageData,0,edges->imageSize);
	
	for(int i=0; i<rows; i++) {
		for(int j=0; j<columns; j++) {
			for(int k=0; k<lineCounts[i*columns+j]; k++) {
				curLine=lines[i*columns+j][k];
				Cb=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.lb), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.lb));
				Ce=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.le), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.le));
				linePath((char *) canvas->data.ptr,Cb,Ce,regSize);

				for(int l=0; l<regSize; l++) {
					for(int m=0; m<regSize; m++) {
						if(i*regSize+l<edges->height && j*regSize+m<edges->width)
							edges->imageData[(i*regSize+l)*edges->widthStep+j*regSize+m] |= ((char*)(canvas->data.ptr))[l*regSize+m];
					}
				}
			}
		}
	}
	
	cvFree((void **)&canvas);
}*/

void arrangeGLines(void *edgesarr, list<LEL_GroupedLine *> &GLPL, int minLen, CvScalar color, bool keepImage=0, int thickness=1) {
	
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
		
		cvLine(edges,Cb,Ce,color,thickness);

	}
}

void drawLine(void *edgesarr, LEL_GroupedLine * GL, CvScalar color=CV_RGB(0,0,255)) {

	IplImage edgesStub, *edges = (IplImage *) edgesarr;

	edges = cvGetImage(edges,&edgesStub);

	LEL_GroupedLine curLine = *GL;
	CvPoint Cb=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.lb), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.lb));
	CvPoint Ce=cvPoint((int)roundf(curLine.R*curLine.C+curLine.S*curLine.le), (int)roundf(curLine.R*curLine.S-curLine.C*curLine.le));
	
	cvLine(edges,Cb,Ce,color,1);
	
}

/*void tabDepth(int d) {
	while(d--)
		cout<<"\t";
}

void exploreQuadTree(QuadTreeNode * QTN, int d) {
	
	tabDepth(d);
	cout<<"dl:";
	if(QTN->dl==NULL) cout<<"empty\n";
	else if(QTN->dl->isNode) {
		cout<<"There is a node, entering \n";
		exploreQuadTree(QTN->dl->pointers.node,d+1);
		tabDepth(d);
		cout<<"Leaving node \n";
		}
	else cout<<"There is a point:"<<QTN->dl->pointers.point->x<<","<<QTN->dl->pointers.point->y<<"\n";

	tabDepth(d);
	cout<<"dr:";
	if(QTN->dr==NULL) cout<<"empty\n";
	else if(QTN->dr->isNode) {
		cout<<"There is a node, entering \n";
		exploreQuadTree(QTN->dr->pointers.node,d+1);
		tabDepth(d);
		cout<<"Leaving node \n";
		}
	else cout<<"There is a point:"<<QTN->dr->pointers.point->x<<","<<QTN->dr->pointers.point->y<<"\n";

	tabDepth(d);
	cout<<"ul:";
	if(QTN->ul==NULL) cout<<"empty\n";
	else if(QTN->ul->isNode) {
		cout<<"There is a node, entering \n";
		exploreQuadTree(QTN->ul->pointers.node,d+1);
		tabDepth(d);
		cout<<"Leaving node \n";
		}
	else cout<<"There is a point:"<<QTN->ul->pointers.point->x<<","<<QTN->ul->pointers.point->y<<"\n";
	
	tabDepth(d);
	cout<<"ur:";
	if(QTN->ur==NULL) cout<<"empty\n";
	else if(QTN->ur->isNode) {
		cout<<"There is a node, entering \n";
		exploreQuadTree(QTN->ur->pointers.node,d+1);
		tabDepth(d);
		cout<<"Leaving node \n";
		}
	else cout<<"There is a point:"<<QTN->ur->pointers.point->x<<","<<QTN->ur->pointers.point->y<<"\n";
}*/

#endif /*LEL_UTILITIES_H_*/
