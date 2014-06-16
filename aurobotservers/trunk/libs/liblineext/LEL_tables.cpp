#include "LEL_hough.h"
#include "LEL_houghInt.h"

int numOfBins;
signed char * bins; // The bins to vote on each time, defined here with the hope that it will stay in the cache
signed char * templateBins; // The template for the memcpy to copy on bins
signed char *** pixels2Bins;
char * pixels2BinCount;
LEL_Angle * angles;
char * LEL_memory;
signed char **pointers;


void linePath( char* pixels, CvPoint Cb, CvPoint Ce, int regSize );

void prepareTables() {

	angles = (LEL_Angle *) cvAlloc(NUMOFANGLES*sizeof(LEL_Angle));//new Angle[NUMOFANGLES];
	CvPoint Cb, Ce;
	int i,j,k=0,l,b=0;

	bins = (signed char *) cvAlloc(NUMOFBINS*sizeof(signed char));//new signed char[NUMOFBINS];
	templateBins = (signed char *) cvAlloc(NUMOFBINS*sizeof(signed char));//new signed char[NUMOFBINS];
	memset(templateBins,0,NUMOFBINS);

	LEL_memory = (char *) cvAlloc(NUMOFBINS*REGIONSIZE*REGIONSIZE*sizeof(char));//new char[NUMOFBINS*REGIONSIZE*REGIONSIZE];
	for(i=0; i<NUMOFANGLES; i++) {
		if (i<REGIONSIZE) {
			angles[i].binCount=2*REGIONSIZE-1-i;
			angles[i].rowBegin=&bins[b];
			b+=angles[i].binCount;
			angles[i].bins2Pixels = (char **) cvAlloc(angles[i].binCount*sizeof(char *));//new (char *[angles[i].binCount]);
			for (j=0; j<angles[i].binCount; j++) {
				angles[i].bins2Pixels[j] = LEL_memory+k++*REGIONSIZE*REGIONSIZE;
				Cb = cvPoint(j,REGIONSIZE-1);
				Ce = cvPoint(j-REGIONSIZE+1+i,0);
				linePath(angles[i].bins2Pixels[j],Cb,Ce,REGIONSIZE);
			}
		}else if(i<2*REGIONSIZE-1){
			angles[i].binCount=i+1;
			angles[i].rowBegin=&bins[b];
			b+=angles[i].binCount;
			angles[i].bins2Pixels = (char **) cvAlloc(angles[i].binCount*sizeof(char *));//new (char *[angles[i].binCount]);
			for (j=0; j<angles[i].binCount; j++) {
				angles[i].bins2Pixels[j] = LEL_memory+k++*REGIONSIZE*REGIONSIZE;
				Cb = cvPoint(j,0);
				Ce = cvPoint(j+REGIONSIZE-i-1,REGIONSIZE-1);
				linePath(angles[i].bins2Pixels[j],Cb,Ce,REGIONSIZE);
			}			
		}else if(i<3*REGIONSIZE-2){ 
			angles[i].binCount=4*REGIONSIZE-3-i;
			angles[i].rowBegin=&bins[b];
			b+=angles[i].binCount;
			angles[i].bins2Pixels = (char **) cvAlloc(angles[i].binCount*sizeof(char *));//new (char *[angles[i].binCount]);
			for (j=0; j<angles[i].binCount; j++) {
				angles[i].bins2Pixels[j] = LEL_memory+k++*REGIONSIZE*REGIONSIZE;
				Cb = cvPoint(0,j);
				Ce = cvPoint(REGIONSIZE-1,j+i+3-3*REGIONSIZE);
				linePath(angles[i].bins2Pixels[j],Cb,Ce,REGIONSIZE);
			}						
		}else {
			angles[i].binCount=i+3-2*REGIONSIZE;
			angles[i].rowBegin=&bins[b];
			b+=angles[i].binCount;
			angles[i].bins2Pixels = (char **) cvAlloc(angles[i].binCount*sizeof(char *));//new (char *[angles[i].binCount]);
			for (j=0; j<angles[i].binCount; j++) {
				angles[i].bins2Pixels[j] = LEL_memory+k++*REGIONSIZE*REGIONSIZE;
				Cb = cvPoint(REGIONSIZE-1,j);
				Ce = cvPoint(0,j-i-3+3*REGIONSIZE);
				linePath(angles[i].bins2Pixels[j],Cb,Ce,REGIONSIZE);
			}						
		}
	}

	pixels2BinCount = (char *) cvAlloc(REGIONSIZE*REGIONSIZE*sizeof(char));//new char[REGIONSIZE*REGIONSIZE];
	memset(pixels2BinCount,NUMOFANGLES,REGIONSIZE*REGIONSIZE);
	pixels2Bins = (signed char ***) cvAlloc(REGIONSIZE*REGIONSIZE*sizeof(signed char **));//new (signed char **[REGIONSIZE*REGIONSIZE]);
	pointers = (signed char **) cvAlloc(NUMOFANGLES*REGIONSIZE*REGIONSIZE*sizeof(signed char *));//new signed char *[NUMOFANGLES*REGIONSIZE*REGIONSIZE];
	for(i=0,k=0; i<REGIONSIZE*REGIONSIZE; i++) {
		pixels2Bins[i]=pointers+k++*NUMOFANGLES;
		for (j=0; j<NUMOFANGLES; j++) {
			for (l=0; l<angles[j].binCount;l++) {
				if(angles[j].bins2Pixels[l][i]) {
					pixels2Bins[i][j]=angles[j].rowBegin+l;
				}
			}
		}
	}
}

void linePath( char* pixels, CvPoint Cb, CvPoint Ce, int regSize ) {
	int x0, y0, x1, y1, xt,yt, deltax, deltay,x,y,ystep;
	double error, deltaerr;
	bool steep;
	
	for(int i=0; i<regSize*regSize;i++) {
		pixels[i]=0;
	}
	
	x0=Cb.x;
	y0=Cb.y;
	x1=Ce.x;
	y1=Ce.y;
	steep = abs(y1 - y0) > abs(x1 - x0);
	if(steep) {
		xt=x0;
		x0=y0;
		y0=xt;
		xt=x1;
		x1=y1;
		y1=xt;
	}
	if(x0 > x1) {
    	xt=x0;
    	x0=x1;
    	x1=xt;
    	yt=y0;
    	y0=y1;
    	y1=yt;        
	}
	deltax= x1 - x0;
	deltay= abs(y1 - y0);
	error = 0;
	deltaerr = ((double)deltay) / deltax;
	y = y0;
	if (y0 < y1) 
		ystep = 1;
	else 
		ystep = -1;

	for (x = x0; x<=x1; x++) {
		if(x<regSize && x>=0 && y<regSize && y>=0)
			if(steep) 
				pixels[x*regSize+y]='\xFF';
			else
				pixels[y*regSize+x]='\xFF';
		error += deltaerr;
		if (error >= 0.5) {
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

//void drawLine( IplImage* image, CvPoint Cb, CvPoint Ce) {
//	int x0, y0, x1, y1, xt,yt, deltax, deltay,x,y,ystep;
//	double error, deltaerr;
//	bool steep;
//
//	x0=Cb.x;
//	y0=Cb.y;
//	x1=Ce.x;
//	y1=Ce.y;
//	steep = abs(y1 - y0) > abs(x1 - x0);
//	if(steep) {
//		xt=x0;
//		x0=y0;
//		y0=xt;
//		xt=x1;
//		x1=y1;
//		y1=xt;
//	}
//	if(x0 > x1) {
//    	xt=x0;
//    	x0=x1;
//    	x1=xt;
//    	yt=y0;
//    	y0=y1;
//    	y1=yt;
//	}
//	deltax= x1 - x0;
//	deltay= abs(y1 - y0);
//	error = 0;
//	deltaerr = ((double)deltay) / deltax;
//	y = y0;
//	if (y0 < y1)
//		ystep = 1;
//	else
//		ystep = -1;
//
//	for (x = x0; x<=x1; x++) {
//		if(x>=0 && y>=0)
//			if(steep) {
//				if(x<image->height && y<image->width)
//					image->imageData[x*image->widthStep+y]=255;
//			}
//			else
//				if(y<image->height && x<image->width)
//					image->imageData[y*image->widthStep+x]=255;
//		error += deltaerr;
//		if (error >= 0.5) {
//			y = y + ystep;
//			error = error - 1.0;
//		}
//	}
//}

void freeTables() {
	
	int i;
	
	for(i=0; i<NUMOFANGLES; i++) {
		cvFree((void **)&(angles[i].bins2Pixels));
	}
	//cvFree(&angles);
	cvFree((void **)&LEL_memory);
	cvFree((void **)&bins);
	cvFree((void **)&templateBins);
	cvFree((void **)&pointers);
	cvFree((void **)&pixels2Bins);
}

