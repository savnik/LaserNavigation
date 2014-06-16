#ifndef LEL_HOUGH_H_
#define LEL_HOUGH_H_

#include <list>
//#define OPENCV2
#ifdef OPENCV2
#include <core/core_c.h>
#else
#include <opencv/cxcore.h>
#endif
#include "LEL_commons.h"

struct LEL_IndexedPoint {
	int index;
	int x;
	int y;
};

struct LEL_FloatingPoint{
	float x;
	float y;
	inline LEL_FloatingPoint(){};
	inline LEL_FloatingPoint(float x, float y) {this->x=x; this->y=y;}
};

struct LEL_ImRegion {
	int edgeCount; // The number of edges in this region.
	LEL_IndexedPoint *edges; // The pointer to the array holding the edges in this region

};

struct LEL_DistributedEdges {
	CvSize Size; // The overall image size
	int regionSize; // The size of each image sub-region
	LEL_ImRegion* regions; // Pointer to the regions
	int vertReg; // The number of regions in the vertical direction
	int horReg; // The number of regions in the horizontal direction

};

struct LEL_Angle {
	signed char * rowBegin; // The pointer to the beginning of the bins with this angle
	char ** bins2Pixels; // Used while generating the tables
	int binCount; // The number of bins that have this angle
	bool direction; // The direction of this angle (vert/hor)
	int pixelDiff; // The difference between the starting and ending pixels in hor/vert dir.
	float radialBegin; // The radial distance of the line represented by the starting bin
	float radialIncrement; // The radial increment at each bin for this angle
};

struct LEL_LineSegment {
	LEL_IndexedPoint * edges; // The pointer to the array of edges supporting this line
	int edgeCount; // The number of edges supporting this line
	float mux; // The x component of the mean of the distribution of this line
	float muy; // The y component of the mean of the distribution of this line
	float mxx; // The x2 moment of the points
	float mxy; // The xy moment of the points
	float myy; // The y2 moment of the points
	float R; // The distance of this line to the origin (of some coordinate system)
	float Th; // The angle of the direction of this line
	float lb; // The beginning point of this line, given along the line
	float le; // The end point of this line, given along the line

	// These are calculated values of sin and cos of Th, to increase speed
	float C; // The cosine of Th
	float S; // The sine of Th

	// The connectivity values
	bool u:1, ur:1, r:1, dr:1, d:1, dl:1, l:1, ul:1; // up, up-right, right, down-right,...

	bool connected:1; // A flag, that indicates if this line is connected to a line group
};

struct LEL_GroupedLine { // The line representation is cos(Th)*x + sin(Th)*x = R

	int edgeCount; // The number of edges supporting this line
	float mux; // The x component of the mean of the distribution of this line
	float muy; // The y component of the mean of the distribution of this line
	float mxx; // The x2 moment of the points
	float mxy; // The xy moment of the points
	float myy; // The y2 moment of the points
	float R; // The distance of this line to the origin (of some coordinate system)
	float Th; // The angle of the direction of this line
	float lb; // The beginning point of this line, given along the line
	float le; // The end point of this line, given along the line

	// These are calculated values of sin and cos of Th, to increase speed
	float C; // The cosine of Th
	float S; // The sine of Th

	bool initialized; // Indicates if the line is empty or not

};

struct DistributedLines {
	LEL_LineSegment ** lines;
	int * lineCounts;
	int rows;
	int columns;
	int regSize;
};

struct ImageInfo {

	float centerX;
	float centerY;
	float focalDist;
	int width;
	int height;

	ImageInfo(int width, int height, float focalDist);
	inline ImageInfo(){centerX=0; centerY=0; focalDist=0; width=0; height=0;}
};

void freeDistributedEdges(LEL_DistributedEdges *DE);

void cloneDistributedEdges(LEL_DistributedEdges *DEin,LEL_DistributedEdges *DEout);

void freeDistributedLines(DistributedLines *DL);

LEL_FloatingPoint floatingPoint(float x, float y);

void prepareTables();

void freeTables();

void hough(const void* edgeImage, std::list<LEL_GroupedLine> &GLL);

void hough(const void* edgeImage, std::list<LEL_GFLine> &GLL);

void cvModCanny( IplImage* img, IplImage* dst, double low_thresh, double high_thresh, int aperture_size=3);

void cvModCanny( CvMat* dx, CvMat* dy, IplImage* dst, double low_thresh, double high_thresh);

#endif /*LEL_HOUGH_H_*/
