/*
 * ransac.h
 *
 *  Created on: 14/12/2009
 *      Author: eba
 */

#ifndef RANSAC_H_
#define RANSAC_H_

#include <list>
#include "LEL_commons.h"
using namespace std;

void ransac(double* X, double* Y, int pointCount, list<LEL_GFLine> &GFLL );

#endif /* RANSAC_H_ */
