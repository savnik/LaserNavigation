/*
 * hough.cpp
 *
 *  Created on: 16/12/2009
 *      Author: eba
 */
#include <iostream>
using namespace std;

#include "LEL_hough.h"
#include "LEL_houghInt.h"


inline void calculateLine(LEL_IndexedPoint *edges, int edgeCount, LEL_LineSegment &line);
inline void combineLine(LEL_GroupedLine &GL, LEL_LineSegment &L, CvPoint curRegion);
inline bool evaluateLine(LEL_GroupedLine &GL, LEL_LineSegment &L, CvPoint curRegion);

const float MAXMSE1 = 3;	//The maximum MSE value allowed in the first pass of line evaluation
const float MAXMSE2 = 0.4;	//The maximum MSE value allowed in the second pass of line evaluation

void hough(const void* edgeImage, std::list<LEL_GroupedLine> &GLL) {

	LEL_DistributedEdges DE;

	distribute(edgeImage,&DE,REGIONSIZE);

	int i,j,k,l,pointIndex, rowIndex, curIndex;
	int b1,b2;
	int binCount;
	int maxBinCouple,maxBinElem, maxBinAngle=-1, maxBinRadius=-1;
	int regColumns=DE.horReg;
	int regRows=DE.vertReg;
	int regSize= DE.regionSize;
	LEL_ImRegion* regions = DE.regions;
	LEL_ImRegion* thisRegion;
	signed char ** bins2Vote;
	signed char *curBin;
	LEL_IndexedPoint * lineEdges, * thisEdges, tmpPoint;
	int thisEdgeCount;
	char *bin1toPixels, * bin2toPixels;
	DistributedLines DL;

	LEL_LineSegment curLine;
	LEL_LineSegment ** lines = (LEL_LineSegment **) cvAlloc(regRows*regColumns*sizeof(LEL_LineSegment *));
	int * lineCounts = (int *) cvAlloc(regRows*regColumns*sizeof(int));
	memset(lineCounts,0,regRows*regColumns*sizeof(int));

	LEL_LineSegment *buffer = (LEL_LineSegment *) cvAlloc(REGIONSIZE*sizeof(LEL_LineSegment));
	int bufferCount=0;

	for(i=0; i<regRows;i++) {
		rowIndex = i*regColumns;
		for(j=0; j<regColumns; j++){
			if (!((thisRegion=regions+(curIndex=rowIndex+j))->edgeCount))
				continue;
			while(thisRegion->edgeCount>2) {
				memcpy(bins,templateBins,NUMOFBINS);

				thisEdges=thisRegion->edges;

				for(thisEdgeCount=k=thisRegion->edgeCount;k;) {
					bins2Vote = pixels2Bins[pointIndex=(thisEdges[--k].index)];
					for (l=pixels2BinCount[pointIndex];l;) { // l = bincount for this point
						*bins2Vote[--l]+=1;
					}
				}

				for(k=NUMOFANGLES,maxBinCouple=0,maxBinElem=0; k;){
					curBin=angles[--k].rowBegin;
					b1=*curBin++;
					for(binCount=l=angles[k].binCount;--l;){ // The condition is so since we need count -1 iterations
						b2=*curBin++;
						if(b1+b2>maxBinCouple || (b1+b2==maxBinCouple && (b2>maxBinElem || b1>maxBinElem))){
							maxBinCouple = b1+b2;
							maxBinElem = b1>b2?b1:b2;
							maxBinAngle = k;
							maxBinRadius = binCount-l;
						}
						if(!(--l))
							break;
						b1=*curBin++;
						if(b1+b2>maxBinCouple || (b1+b2==maxBinCouple && (b1>maxBinElem || b2>maxBinElem))){
							maxBinCouple = b1+b2;
							maxBinElem = b1>b2?b1:b2;
							maxBinAngle = k;
							maxBinRadius = binCount-l;
						}
					}
				}

				if(maxBinCouple<3) break;

				bin1toPixels = angles[maxBinAngle].bins2Pixels[maxBinRadius];
				bin2toPixels = angles[maxBinAngle].bins2Pixels[maxBinRadius-1];

				if((thisRegion->edgeCount=thisEdgeCount-maxBinCouple)) {
					lineEdges=thisEdges+thisEdgeCount-1;
					while(1) {
						while(1) {
							if(bin1toPixels[pointIndex=thisEdges->index]||bin2toPixels[pointIndex]) {
								break;
							}
							thisEdges++;
						}
						while(1) {
							if(bin1toPixels[pointIndex=lineEdges->index]||bin2toPixels[pointIndex]) {
								if(thisEdges == lineEdges) goto outOfLoop;
								lineEdges--;
							}
							break;
						}
						tmpPoint = *lineEdges;
						*lineEdges = *thisEdges;
						*thisEdges = tmpPoint;
					}
					outOfLoop:;

				} else lineEdges=thisEdges;
				calculateLine(lineEdges,maxBinCouple,curLine);
				buffer[bufferCount++]=curLine;
			}
			if(bufferCount) {
				lines[curIndex]=(LEL_LineSegment *) cvAlloc(bufferCount*sizeof(LEL_LineSegment));
				memcpy(lines[curIndex],buffer,bufferCount*sizeof(LEL_LineSegment));
				lineCounts[curIndex]=bufferCount;
				bufferCount=0;
			}
		}
	}

	int searchDir; // In order to keep a consistent search, 2= ur, 4=dr

	LEL_GroupedLine curGrLine;

	CvPoint * regionStack = (CvPoint *) cvAlloc((regRows+regColumns)*sizeof(CvPoint));
	int stackCount = 0;
	CvPoint curRegion;

#define LEL_HOUGH_PUSH(xp,yp) regionStack[stackCount++]=cvPoint(xp,yp)
#define LEL_HOUGH_POP(cvp) cvp = regionStack[--stackCount]
#define LEL_HOUGH_ISEMPTY (stackCount==0)

	for(i=0; i<regRows;i++) {
		rowIndex = i*regColumns;
		for(j=0; j<regColumns; j++) {
			k=lineCounts[curIndex=rowIndex+j];
			while(k--) {
				LEL_LineSegment *curLinePtr;
				if (!((curLinePtr=lines[curIndex]+k)->connected)) {
					curLinePtr->connected=1;
					curLine=*curLinePtr;
					curGrLine.initialized=0;
					combineLine(curGrLine,curLine,cvPoint(j,i));
					float Th=curGrLine.Th>0?curGrLine.Th:curGrLine.Th+M_PI;
					if(Th>M_PI_2) searchDir=4;
					else searchDir =2;

					// Startsearch in searchDir
					/*if(searchDir==1 && curLine.ul) {
						if(j-1>=0 && i-1>=0)
							HOUGH_PUSH(j-1,i-1);
					}*/
					if((searchDir==2) && curLine.u) {
						if(i-1>=0)
							LEL_HOUGH_PUSH(j,i-1);
					}
					if((searchDir==2) && curLine.ur) {
						if(j+1<regColumns && i-1>=0)
							LEL_HOUGH_PUSH(j+1,i-1);
					}
					if((searchDir==2||searchDir==4) && curLine.r) {
						if(j+1<regColumns)
							LEL_HOUGH_PUSH(j+1,i);
					}
					if((searchDir==4) && curLine.dr) {
						if(j+1<regColumns && i+1<regRows)
							LEL_HOUGH_PUSH(j+1,i+1);
					}
					if(searchDir==4 && curLine.d) {
						if(i+1<regRows)
							LEL_HOUGH_PUSH(j,i+1);
					}

					while(!LEL_HOUGH_ISEMPTY) {
						LEL_HOUGH_POP(curRegion);
						int workingIndex=curRegion.x+curRegion.y*regColumns;
						l=lineCounts[workingIndex];
						while(l--) {
							if(!((curLinePtr=lines[workingIndex]+l)->connected)
									&& evaluateLine(curGrLine,*curLinePtr,curRegion)) {
								curLinePtr->connected=1;
								float Th=curGrLine.Th>0?curGrLine.Th:curGrLine.Th+M_PI;
								if(Th>M_PI_2) searchDir=4;
								else searchDir =2;
								/*if(searchDir==1 && curLinePtr->ul) {
									if(curRegion.x-1>=0 && curRegion.y-1>=0)
										HOUGH_PUSH(curRegion.x-1,curRegion.y-1);
								}*/
								if((searchDir==2) && curLinePtr->u) {
									if(curRegion.y-1>=0)
										LEL_HOUGH_PUSH(curRegion.x,curRegion.y-1);
								}
								if((searchDir==2) && curLinePtr->ur) {
									if(curRegion.x+1<regColumns && curRegion.y-1>=0)
										LEL_HOUGH_PUSH(curRegion.x+1,curRegion.y-1);
								}
								if((searchDir==2||searchDir==4) && curLinePtr->r) {
									if(curRegion.x+1<regColumns)
										LEL_HOUGH_PUSH(curRegion.x+1,curRegion.y);
								}
								if((searchDir==4) && curLinePtr->dr) {
									if(curRegion.x+1<regColumns && curRegion.y+1<regRows)
										LEL_HOUGH_PUSH(curRegion.x+1,curRegion.y+1);
								}
								if(searchDir==4 && curLinePtr->d) {
									if(curRegion.y+1<regRows)
										LEL_HOUGH_PUSH(curRegion.x,curRegion.y+1);
								}
							}
						}
					}

					// Startsearch in reverse searchDir
					/*if(searchDir==1 && curLine.dr) {
						if(j+1<regColumns && i+1<regRows)
							HOUGH_PUSH(j+1,i+1);
					}*/
					if((searchDir==2) && curLine.d) {
						if(i+1<regRows)
							LEL_HOUGH_PUSH(j,i+1);
					}
					if((searchDir==2) && curLine.dl) {
						if(j-1>=0 && i+1<regRows)
							LEL_HOUGH_PUSH(j-1,i+1);
					}
					if((searchDir==2||searchDir==4) && curLine.l) {
						if(j-1>=0)
							LEL_HOUGH_PUSH(j-1,i);
					}
					if((searchDir==4) && curLine.ul) {
						if(j-1>=0 && i-1>=0)
							LEL_HOUGH_PUSH(j-1,i-1);
					}
					if(searchDir==4 && curLine.u) {
						if(i-1>=0)
							LEL_HOUGH_PUSH(j,i-1);
					}

					while(!LEL_HOUGH_ISEMPTY) {
						LEL_HOUGH_POP(curRegion);
						int workingIndex=curRegion.x+curRegion.y*regColumns;
						l=lineCounts[workingIndex];
						while(l--) {
							if(!((curLinePtr=lines[workingIndex]+l)->connected)
									&& evaluateLine(curGrLine,*curLinePtr,curRegion)) {
								curLinePtr->connected=1;
								float Th=curGrLine.Th>0?curGrLine.Th:curGrLine.Th+M_PI;
								if(Th>M_PI_2) searchDir=4;
								else searchDir =2;
								/*if(searchDir==1 && curLinePtr->dr) {
									if(curRegion.x+1<regColumns && curRegion.y+1<regRows)
										HOUGH_PUSH(curRegion.x+1,curRegion.y+1);
								}*/
								if((searchDir==2) && curLinePtr->d) {
									if(curRegion.y+1<regRows)
										LEL_HOUGH_PUSH(curRegion.x,curRegion.y+1);
								}
								if(searchDir==2 && curLinePtr->dl) {
									if(curRegion.x-1>=0 && curRegion.y+1<regRows)
										LEL_HOUGH_PUSH(curRegion.x-1,curRegion.y+1);
								}
								if((searchDir==2||searchDir==4) && curLinePtr->l) {
									if(curRegion.x-1>=0)
										LEL_HOUGH_PUSH(curRegion.x-1,curRegion.y);
								}
								if((searchDir==4) && curLinePtr->ul) {
									if(curRegion.x-1>=0 && curRegion.y-1>=0)
										LEL_HOUGH_PUSH(curRegion.x-1,curRegion.y-1);
								}
								if(searchDir==4 && curLinePtr->u) {
									if(curRegion.y-1>=0)
										LEL_HOUGH_PUSH(curRegion.x,curRegion.y-1);
								}
							}
						}
					}

					GLL.push_back(curGrLine);
					curGrLine.initialized=0;
				}
			}
		}
	}

	DL.lines=lines;
	DL.lineCounts=lineCounts;
	DL.rows=regRows;
	DL.columns=regColumns;
	DL.regSize=regSize;

	cvFree((void **)&buffer);
	cvFree((void **)&regionStack);
	freeDistributedLines(&DL);
	freeDistributedEdges(&DE);

}

inline void calculateLine(LEL_IndexedPoint *edges, int edgeCount, LEL_LineSegment &line) {
	float ex=0, ey=0, ex2=0, ey2=0, exy=0, xc, yc, lb, le, l, S, C;
	int n=edgeCount;

	line.edges=edges;
	line.edgeCount=edgeCount;

	while(edgeCount--) {
		xc=edges->x;
		yc=(edges++)->y;
		ex+=xc;
		ex2+=xc*xc;
		ey+=yc;
		exy+=xc*yc;
		ey2+=yc*yc;
	}
	line.Th=atan2(2*ex*ey-2*n*exy,ex*ex-ey*ey-n*(ex2-ey2))/2;
	line.R=(ex/(float)n)*cos(line.Th)+(ey/(float)n)*sin(line.Th);
	if(line.R<0) {
		line.R=-line.R;
		if(line.Th<=0) line.Th=line.Th+M_PI;
		else line.Th=line.Th-M_PI;
	}

	line.mux=ex/(float)n;
	line.muy=ey/(float)n;
	line.mxx=ex2/(float)n;
	line.mxy=exy/(float)n;
	line.myy=ey2/(float)n;

	line.C=C=cos(line.Th);
	line.S=S=sin(line.Th);

	edges--;
	lb=le= (edges)->x*S-(edges)->y*C;

	while(--n) {
		edges--;
		l=(edges)->x*S-(edges)->y*C;
		if(l>le) le=l;
		else if(l<lb) lb=l;
	}
	line.lb=lb;
	line.le=le;

	float CC = C*C; //cos2(Th);
	float SC = S*C; //cos(Th)sin(Th);
	float SS = S*S; //sin2(Th);
	float CR = line.R*C;
	float SR = line.R*S;

	if(fabsf(CC*CENTERX+SC*UPY-CR)<REGIONSIZE_2_1 && fabsf(SC*CENTERX+SS*UPY-SR)<REGIONSIZE_2_1) line.u=1; else line.u=0;
	if(fabsf(CC*RIGHTX+SC*UPY-CR)<REGIONSIZE_2_1 && fabsf(SC*RIGHTX+SS*UPY-SR)<REGIONSIZE_2_1) line.ur=1; else line.ur=0;
	if(fabsf(CC*RIGHTX+SC*CENTERY-CR)<REGIONSIZE_2_1 && fabsf(SC*RIGHTX+SS*CENTERY-SR)<REGIONSIZE_2_1) line.r=1; else line.r=0;
	if(fabsf(CC*RIGHTX+SC*DOWNY-CR)<REGIONSIZE_2_1 && fabsf(SC*RIGHTX+SS*DOWNY-SR)<REGIONSIZE_2_1) line.dr=1; else line.dr=0;
	if(fabsf(CC*CENTERX+SC*DOWNY-CR)<REGIONSIZE_2_1 && fabsf(SC*CENTERX+SS*DOWNY-SR)<REGIONSIZE_2_1) line.d=1; else line.d=0;
	if(fabsf(CC*LEFTX+SC*DOWNY-CR)<REGIONSIZE_2_1 && fabsf(SC*LEFTX+SS*DOWNY-SR)<REGIONSIZE_2_1) line.dl=1; else line.dl=0;
	if(fabsf(CC*LEFTX+SC*CENTERY-CR)<REGIONSIZE_2_1 && fabsf(SC*LEFTX+SS*CENTERY-SR)<REGIONSIZE_2_1) line.l=1; else line.l=0;
	if(fabsf(CC*LEFTX+SC*UPY-CR)<REGIONSIZE_2_1 && fabsf(SC*LEFTX+SS*UPY-SR)<REGIONSIZE_2_1) line.ul=1; else line.ul=0;

	line.connected=0;

}

inline void combineLine(LEL_GroupedLine &GL, LEL_LineSegment &L, CvPoint curRegion) {

	int Ox=curRegion.x*REGIONSIZE;
	int Oy=curRegion.y*REGIONSIZE;

	if(!GL.initialized) {
		GL.R=L.R+Ox*L.C+Oy*L.S;
		bool reversed = 0; // Used later, 1 if angle is reversed
		if(GL.R<0) {
			GL.R=-GL.R;
			reversed=1;
			if(L.Th<0) GL.Th = L.Th+M_PI;
			else GL.Th=L.Th-M_PI;
		} else GL.Th=L.Th;
		GL.mux=L.mux+Ox;
		GL.muy=L.muy+Oy;
		GL.mxx=L.mxx+2*Ox*L.mux+Ox*Ox;
		GL.mxy=L.mxy+Oy*L.mux+Ox*L.muy+Ox*Oy;
		GL.myy=L.myy+2*Oy*L.muy+Oy*Oy;
		GL.C=reversed?-L.C:L.C;
		GL.S=reversed?-L.S:L.S;
		GL.edgeCount=L.edgeCount;
		float Rdiff = Ox*GL.S-Oy*GL.C;
		if(reversed) {
			GL.lb=Rdiff-L.le;
			GL.le=Rdiff-L.lb;
		} else {
			GL.lb=Rdiff+L.lb;
			GL.le=Rdiff+L.le;
		}
		GL.initialized=1;
	} else {
		float Rold = GL.R;
		float Sold= GL.S;
		float Cold= GL.C;

		int totalEdges=GL.edgeCount+L.edgeCount;
		float wGL=GL.edgeCount/(float)totalEdges;
		float wL= L.edgeCount/(float)totalEdges;
		GL.mux=(L.mux+Ox)*wL+GL.mux*wGL;
		GL.muy=(L.muy+Oy)*wL+GL.muy*wGL;
		GL.mxx=(L.mxx+2*Ox*L.mux+Ox*Ox)*wL+GL.mxx*wGL;
		GL.mxy=(L.mxy+Ox*L.muy+Oy*L.mux+Ox*Oy)*wL+GL.mxy*wGL;
		GL.myy=(L.myy+2*Oy*L.muy+Oy*Oy)*wL+GL.myy*wGL;
		GL.edgeCount=totalEdges;

		GL.Th=atan2(2*GL.mux*GL.muy-2*GL.mxy,GL.mux*GL.mux-GL.muy*GL.muy-GL.mxx+GL.myy)/2;
		GL.R=GL.mux*cos(GL.Th)+GL.muy*sin(GL.Th);
		if(GL.R<0) {
			GL.R=-GL.R;
			if(GL.Th<=0) GL.Th=GL.Th+M_PI;
			else GL.Th=GL.Th-M_PI;
		}

		GL.C=cos(GL.Th);
		GL.S=sin(GL.Th);

		float lbnew1,lbnew2, lenew1,lenew2;
		lbnew1 = GL.S*(Rold*Cold+GL.lb*Sold)-GL.C*(Rold*Sold-GL.lb*Cold);
		lbnew2 = GL.S*(Ox+L.R*L.C+L.lb*L.S)-GL.C*(Oy+L.R*L.S-L.lb*L.C);
		lenew1 = GL.S*(Rold*Cold+GL.le*Sold)-GL.C*(Rold*Sold-GL.le*Cold);
		lenew2 = GL.S*(Ox+L.R*L.C+L.le*L.S)-GL.C*(Oy+L.R*L.S-L.le*L.C);

		GL.lb = MIN(MIN(lbnew1,lbnew2),MIN(lenew1,lenew2));
		GL.le = MAX(MAX(lenew1,lenew2),MAX(lbnew1,lbnew2));

	}
}

inline bool evaluateLine(LEL_GroupedLine &GL, LEL_LineSegment &L, CvPoint curRegion) {
	int Ox = curRegion.x*REGIONSIZE;
	int Oy = curRegion.y*REGIONSIZE;
	float a = GL.C*Ox+GL.S*Oy-GL.R;
	float MSE= GL.C*GL.C*L.mxx+GL.S*GL.S*L.myy+a*a+2*GL.S*GL.C*L.mxy+2*a*(GL.C*L.mux+GL.S*L.muy); //Mean square error

	if(MSE>MAXMSE1) return 0;

	LEL_GroupedLine tmp=GL;
	combineLine(tmp,L,curRegion);
	a = tmp.C*Ox+tmp.S*Oy-tmp.R;
	MSE= tmp.C*tmp.C*L.mxx+tmp.S*tmp.S*L.myy+a*a+2*tmp.S*tmp.C*L.mxy+2*a*(tmp.C*L.mux+tmp.S*L.muy);
	if(MSE>MAXMSE2) return 0;
	GL=tmp;
	return 1;
}

void distribute(const void* edgesarr, LEL_DistributedEdges* DistEdges, int regSize) {

	IplImage edgesStub, *edges = (IplImage *) edgesarr;

	edges = cvGetImage(edges,&edgesStub);

	if(edges->depth!=IPL_DEPTH_8U || edges->nChannels!=1)
		cout<<"\n\nYou're about to get a segmentation fault :P!(This message is originating from LEL_hough.cpp - distribute)\n\n";

	int i,j,k,l,beginIndex,rowIndex;
	int widthStep = edges->widthStep;
	int regStep = edges->widthStep*regSize;
	char * data = edges->imageData;
	LEL_IndexedPoint * buffer = (LEL_IndexedPoint *)cvAlloc(regSize*regSize*sizeof(LEL_IndexedPoint));//new IndexedPoint[regSize*regSize];
	LEL_IndexedPoint * points;
	int buffin;


	DistEdges->Size = cvSize(edges->width,edges->height);
	DistEdges->regionSize=regSize;
	DistEdges->vertReg = (int) ceil(edges->height/(double)regSize);
	DistEdges->horReg = (int) ceil(edges->width/(double)regSize);
	DistEdges->regions = (LEL_ImRegion *) cvAlloc(DistEdges->vertReg*DistEdges->horReg*sizeof(LEL_ImRegion));//new ImRegion[DistEdges->vertReg*DistEdges->horReg];
	LEL_ImRegion *regions=DistEdges->regions;
	LEL_ImRegion *thisRegion;


	for(i=0;i<DistEdges->vertReg;i++)
	{
		int remainingRows = edges->height-i*regSize;
		for(j=0;j<DistEdges->horReg; j++) {
			int remainingColumns = edges->width-j*regSize;
			beginIndex = i*regStep+j*regSize;
			buffin=0;
			for(k=0;k<regSize && k<remainingRows;k++) {
				rowIndex = beginIndex+k*widthStep;
				for(l=0;l<regSize && l<remainingColumns;l++) {
					if (data[rowIndex+l]){
						buffer[buffin].x=l;
						buffer[buffin].y=k;
						buffer[buffin].index=k*regSize+l;
						buffin++;
					}
				}
			}
			thisRegion = &regions[DistEdges->horReg*i+j];
			thisRegion->edgeCount=buffin;
			if(!buffin) {
				thisRegion->edges = 0;
				continue;
			}
			points = (LEL_IndexedPoint *) cvAlloc(buffin*sizeof(LEL_IndexedPoint));//new IndexedPoint[buffin];
			thisRegion->edges = points;
			while(buffin) {
				buffin--;
				points[buffin]=buffer[buffin];
			}
		}
	}

	cvFree((void **)&buffer);

}

void hough(const void* edgeImage, std::list<LEL_GFLine> &GFLL) {

	std::list<LEL_GroupedLine> GLL;

	hough(edgeImage, GLL);

	for(std::list<LEL_GroupedLine>::iterator GLIt = GLL.begin();GLIt != GLL.end(); GLIt++){
		LEL_GFLine newLine;
		// The conversion from cos(Th)*x + sin(Th)*x = R to Ax + By + C = 0 is
		// performed such that A is always positive and sqrt(A^2+B^2) = 1
		if((*GLIt).C>0) {
		newLine.A = (*GLIt).C;
		newLine.B = (*GLIt).S;
		newLine.C = -(*GLIt).R;
		} else {
			newLine.A = -(*GLIt).C;
			newLine.B = -(*GLIt).S;
			newLine.C = (*GLIt).R;
		}

		newLine.edgeCount = (*GLIt).edgeCount;

		GFLL.push_back(newLine);
	}


}
