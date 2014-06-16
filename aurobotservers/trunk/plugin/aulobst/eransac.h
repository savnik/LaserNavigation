/*
 * ransac.h
 *
 *  Created on: 14/12/2009
 *      Author: eba
 */

#ifndef E_RANSAC_H_
#define E_RANSAC_H_

#include <list>
#include "ecommons.h"
using namespace std;

/**
Calculate a list of lines from this array of X,Y points.
\param par is a struct with parameters for the ransac operation
\param X Array of at least pointCount values
\param Y Array of at least pointCount values
\param admit Array of at least pointCount values, on return the entries with value 0 is unused, other values N are used to construct line N-1.
\param pntCnt is the number of points in the X, Y and admit arrays.
\param lineList is an array (of at least 50) line definitions.
\returns the number of lines found. */
int ransac(LEL_GFParams * par, double* X, double* Y, int * admit, int pntCnt,
           GFLine * lineList);

#endif /* RANSAC_H_ */
