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
#ifndef UFUZZYSPLIT_H
#define UFUZZYSPLIT_H

#include "ufuzzyelement.h"

/**
Split fuzzy elements into n classes

@author Christian Andersen
*/
class UFuzzySplit
{
public:
  /**
  Constructor */
  UFuzzySplit();
  /**
  Destructor */
  ~UFuzzySplit();
  /**
  Clear all elements away */
  void clear();
  /**
  Add element.
  Returns false if no more space. */
  bool addElement(UFuzzyElement * elem);
  /**
  Get pointer to an element */
  UFuzzyElement * getElement(int index);
  /**
  Split into x classes */
  bool classify(int clusters,
                double stopVal,
                int maxLoopCnt,
                bool initialize);
  /**
  Old split function   */
  bool classifyOld(int clusters,
                   double stopVal,
                   int maxLoopCnt);
  /**
  Coumt members */
  void countMembers();
  /**
  Get number of members in a group.
  NB! countMembers must be called first. */
  int getMembCount(int clust);
  /**
  Set class center, covariance matrix and
  from these elements
  with a common membership value (=0.5) */
  bool initFromValues(int clust, int inx, int idxCnt);
  /**
  Update this element with distance and membershop value.
  Returns number of best cluster */
  int updateElement(UFuzzyElement * elem,
                    int clustCnt);
  /**
  Get cluster center mvector */
  inline UMatrix * getV(int clust)
  { return &mV[clust]; };
  /**
  Get cluster center mvector */
  inline UMatrix * getF(int clust)
  { return &mF[clust]; };

protected:
  /**
  Calculate center of cluster
  if 'clust' = -1, then membership is set to 1.0. */
  bool getCenter(int clust,
                 double m,
                 UFuzzyElement **e,
                 int eCnt,
                 UMatrix * pmV);
  /**
  Calculate new value for covariance matrices.
  Requires that center is calculated (in pmV)
  Result is set in pmF and pmAi.
  Returns true */
  bool getCovariance(int clust,
                  double m,
                  UFuzzyElement **e,
                  int eCnt,
                  UMatrix * pmF,
                  UMatrix * pmV,
                  UMatrix * pmAi);

protected:
  /**
  Maximum number of elements to classify */
  static const int MAX_ELEMENTS = 100000;
  /**
  Maximum number of clusters usable */
  static const int MAX_CLUSTER_CNT = 20;
  /**
  Array of pointers to elements to classify */
  UFuzzyElement * elems[MAX_ELEMENTS];
  /**
  Number of elements in class */
  int elemsCnt;
  /**
  Cluster center */
  UMatrix4 mV[MAX_CLUSTER_CNT];
  /**
  Covariance matrix values for classes */
  UMatrix4 mF[MAX_CLUSTER_CNT];
  /**
  Inverse Mahalanobi matrix values for classes */
  UMatrix4 mAi[MAX_CLUSTER_CNT];
  /**
  Number of requested classes */
  int clustCnt;
  /**
  Number of elements in each cluster */
  int elemClust[MAX_CLUSTER_CNT];
};

#endif
