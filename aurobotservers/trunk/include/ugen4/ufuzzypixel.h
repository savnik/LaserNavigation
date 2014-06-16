/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUZZYPIXEL_H
#define UFUZZYPIXEL_H

#include "ucommon.h"
#include "ufuzzyelement.h"

/**
Fuzzy classification of pixel colour

@author Christian Andersen
*/
class UFuzzyPixel : public UFuzzyElement
{
public:
  /**
  Constructor */
  UFuzzyPixel();
  /**
  Destructor */
  ~UFuzzyPixel();
  /**
  Get membership value of cluster 'cluster' */
  virtual double getMembValue(int clust)
  { return mu1[clust]; };
  /**
  Add Position vector to this vector, veightet with the
  membership value to power m.
  Add the membership value (to power m) to membSum.
  Returns false if an error is detected.
  If 'clust' < 0, then membershib is set to 1.0. */
  virtual bool addPosition(int clust, double m, UMatrix * mAvg, double * membSum);
  /**
  Add the squared distance from this cluster center
  weighted with the membership value, and add membership value to 'membSum'.
  weight and membership value must be raised to power m.
  If 'clust' < 0, then membershib is set to 1.0. */
  virtual bool addCovariance(int clust, double m, UMatrix * mF,
                     UMatrix * mV,
                     double * membSum);
  /**
  Set distance to cluster 'cluster'.
  Returns false if parameter errors were found. */
  virtual bool setDistance(int clust, UMatrix * mAi, UMatrix * mV);
  /**
  Update participation value based on
  distance to cluset and this weight exponent value (e.g. 1.3),
  must be > 1.0.
  Returns false if there is calculation errors. */
  virtual void updateMembValue(double m, int clustCnt);
  /**
  Get vector size of element position */
  virtual int getVectorSize();
  /**
  Get membership change value from last iteration */
  virtual double getMembValueChange(int clustCnt);
  /**
  Set membership value for a cluster */
  virtual void setMembValue(double value, int clust);
  /**
  Seet the element with random membership values */
  virtual void seedRandom(int clustCnt);
  /**
  Get class for this element */
  virtual int getClass(int clustCnt);
  /**
  Get maximum number of clusters this element type
  can be split into */
  virtual int getMaxClusterCnt();
  /**
  Set pixel value */
  inline void setPixel(UPixel * pixRef)
  { pix = pixRef; };
  /**
  Print values */
  virtual void print(const char * prestring, int clustCnt);

protected:
  /**
  Vector size */
  static const int MAX_CLUSTER_CNT = 4;
  /**
  Membership value for each cluster */
  double mu1[MAX_CLUSTER_CNT];
  /**
  Membership value for each cluster
  in last iteration */
  double mu2[MAX_CLUSTER_CNT];
  /**
  Mahalanobis distance to cluster center */
  double d[MAX_CLUSTER_CNT];
  /**
  Pixel value */
  UPixel * pix;
};

#endif