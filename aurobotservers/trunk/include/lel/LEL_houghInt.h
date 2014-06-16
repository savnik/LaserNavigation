/*
 * LEL_houghInt.h
 *
 *  Created on: 16/12/2009
 *      Author: eba
 */

#ifndef LEL_HOUGHINT_H_
#define LEL_HOUGHINT_H_

const int REGIONSIZE=10; // The size of each sub-region of the image, IMPORTANT!<=16

const int NUMOFANGLES=4*REGIONSIZE-4;
const int NUMOFBINS =6*REGIONSIZE*REGIONSIZE-8*REGIONSIZE+2;

// Useful constants to speed up
const float LEFTX = -((float)REGIONSIZE)/2-0.5; // The x coordinate of the center of the left regions
const float CENTERX = ((float)REGIONSIZE)/2-0.5; // The x coordinate of the center of the center regions
const float RIGHTX = 3*((float)REGIONSIZE)/2-0.5; // x of rigth
const float UPY = -((float)REGIONSIZE)/2-0.5; // The y coordinate of the center of the up regions
const float CENTERY = ((float)REGIONSIZE)/2-0.5; // The y coordinate of the center of the center regions
const float DOWNY = 3*((float)REGIONSIZE)/2-0.5; // y of down

const float REGIONSIZE_2_1=((float)REGIONSIZE)/2+1; // As it appears

void distribute(const void* edgesarr, LEL_DistributedEdges* DistEdges, int regSize);

extern int numOfBins;
extern signed char * bins; // The bins to vote on each time, defined here with the hope that it will stay in the cache
extern signed char * templateBins; // The template for the memcpy to copy on bins
extern signed char *** pixels2Bins;
extern char * pixels2BinCount;
extern LEL_Angle * angles;
extern char * LEL_memory;
extern signed char **pointers;

#endif /* LEL_HOUGHINT_H_ */
