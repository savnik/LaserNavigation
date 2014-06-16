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
#include "ufuzzysplit.h"

UFuzzySplit::UFuzzySplit()
{
  clear();
}


UFuzzySplit::~UFuzzySplit()
{}


void UFuzzySplit::clear()
{
  elemsCnt = 0;
}

///////////////////////////////////////////////////

bool UFuzzySplit::addElement(UFuzzyElement * elem)
{
  bool result = elemsCnt < MAX_ELEMENTS;
  if (elem == NULL)
    printf("UFuzzySplit::addElement: error, adding NULL element\n");
  else if (result)
    elems[elemsCnt++] = elem;
  else
    printf("UFuzzySplit::addElement: no more space, already %d elements\n", MAX_ELEMENTS);    
  return result;
}

///////////////////////////////////////////////////

UFuzzyElement * UFuzzySplit::getElement(int index)
{
  UFuzzyElement * result = NULL;
  if (index < elemsCnt)
    result = elems[index];
  return result;
}

///////////////////////////////////////////////////

int UFuzzySplit::updateElement(UFuzzyElement * elem,
                                           int clustCnt)
{
  int i;
  int result;
  //
  for (i = 0; i < clustCnt; i++)
    elem->setDistance(i, &mAi[i], &mV[i]);
  elem->updateMembValue(1.3, clustCnt);
  result = elem->getClass(clustCnt);
  return result;
}


///////////////////////////////////////////////////

bool UFuzzySplit::classify(int clusters,
                           double stopVal,
                           int maxLoopCnt,
                           bool initialize)
{
  int i, c;
  int loop = 0;
  bool result = false;
  double epsi;
  UFuzzyElement ** ppe;
  UFuzzyElement * pe;
  UMatrix4 * pmV;
  UMatrix4 * pmF;
  UMatrix4 * pmAi;
  int vectSize;
  const double m = 1.2;
  // start with calculate V and F
  bool calcVF = false;
//   const int MSL = 250;
//   char s[MSL];
  //
  result = elemsCnt >= clusters;
  if (result)
  { 
    ppe = elems;
    pe = *ppe;
    if (pe->getMaxClusterCnt() >= clusters)
      clustCnt = clusters;
    else
    {
      clustCnt = pe->getMaxClusterCnt();
      printf("Number of clusters reduced from %d to %d --- "
          " no more space!\n", clusters, clustCnt);
    }
    vectSize = pe->getVectorSize();
    //
    if (initialize)
    { // initialize with random seed
      // seeed with random membership
      for (i = 0; i < elemsCnt; i++)
        (*ppe++)->seedRandom(clustCnt);
      //
      for (c = 0; c < clustCnt; c++)
      { // initialize matrices
        mV[c].setSize(vectSize, 1);
        mF[c].setSize(vectSize, vectSize);
        mAi[c].setSize(vectSize, vectSize);
      }
      calcVF = true;
    }
//     else if (false)
//     { // debug
//       mV[0].print("V0");
//       mF[0].print("mF1");
//       mV[1].print("V1");
//       mF[1].print("mF1");
//       mV[2].print("V2");
//       mF[2].print("mF1");
//       // debug end
//     }
    while (loop++ < maxLoopCnt)
    { // calculate cluster center
      // step 1
      if (calcVF)
      { // this step is needed after initialize only
      // debug
//         if (false)
//         {
//           countMembers();
//           for (i = 0; i < clustCnt; i++)
//           {
//             printf("Members of cluster %d is %d\n",
//                   i, getMembCount(i));
//             snprintf(s, MSL, "Cl %d V", i);
//             mV[i].print(s);
//             snprintf(s, MSL, "Cl %d F", i);
//             mF[i].print(s);
//             snprintf(s, MSL, "Cl %d Ai", i);
//             mAi[i].print(s);
//           }
//           e = elems;
//           for (i = 0; i < elemsCnt; i++)
//           {
//             (*e)->print("pix", clustCnt);
//             e++;
//           }
//         }
        // debug end
        pmV = mV;
        for (c = 0; c < clustCnt; c++)
        {
          ppe = elems;
          getCenter(c, m, ppe, elemsCnt, pmV);
          pmV++;
        }
        // step 2
        // calculate covariance matrices F
        pmF = mF;
        pmV = mV;
        pmAi = mAi;
        for (c = 0; c < clustCnt; c++)
        {
          ppe = elems;
          getCovariance(c, m, ppe, elemsCnt, pmF, pmV, pmAi);
          pmF++;
          pmV++;
          pmAi++;
        }
      }
      else
        calcVF = true;
      // step 3
      // calculate mahalanobis distance to center
      pmF = mF;
      pmV = mV;
      pmAi = mAi;
      for (c = 0; c < clustCnt; c++)
      { // distances for cluster c
        ppe = elems;
        for (i = 0; i < elemsCnt; i++)
        {
          (*ppe)->setDistance(c, pmAi, pmV);
          ppe++;
        }
        pmAi++;
        pmF++;
        pmV++;
      }
      // debug
/*      e = elems;
      for (i = 0; i < elemsCnt; i++)
      {
        (*e)->print("pix", clustCnt);
        e++;
      }*/
      // debug end
      // step 4
      // update membership
      ppe = elems;
      for (i = 0; i < elemsCnt; i++)
        (*ppe++)->updateMembValue(m, clustCnt);
      // step 5
      // finished?
      epsi = 0.0;
      ppe = elems;
      for (i = 0; i < elemsCnt; i++)
        epsi += (*ppe++)->getMembValueChange(clustCnt);
      epsi /= double(elemsCnt);
      // debug
//       if (false)
//       {
//         countMembers();
//         for (i = 0; i < clustCnt; i++)
//           printf("Cluster %d has %d members\n", i, getMembCount(i));
//         printf("Loop %d epsi=%f\n", loop, epsi);
//       }
      // debug end
      if (epsi < stopVal)
        break;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UFuzzySplit::getCenter(int clust,
               double m,
               UFuzzyElement **e,
               int eCnt,
               UMatrix * pmV)
{
  int i;
  double summu;
  //
  pmV->clear();
  summu = 0.0;
  for (i = 0; i < eCnt; i++)
  {
    (*e++)->addPosition(clust, m, pmV, &summu);
    if ((e - elems) > elemsCnt)
      printf("Index error %d > %d\n", (int)(e - elems), elemsCnt);
  }
  pmV->mult(1.0/summu);
  return true;
}

///////////////////////////////////////////////////

bool UFuzzySplit::getCovariance(int clust,
                            double m,
                            UFuzzyElement **e,
                            int eCnt,
                            UMatrix * pmF,
                            UMatrix * pmV,
                            UMatrix * pmAi)
{
  int i, r, c;
  double summu;
  double det;
  int err, loops;
  UMatrix4 * pmF4 = (UMatrix4*)pmF;
  UMatrix4 * pmAi4 = (UMatrix4*)pmAi;
  //
  pmF4->clear();
  summu = 0.0;
  for (i = 0; i < eCnt; i++)
    (*e++)->addCovariance(clust, m, pmF4, pmV, &summu);
  pmF4->mult(1.0/summu);
  // generate inverse matrix for mahalanobis distance
  det = pmF4->det(&err);
  loops = 0;
  while (det < 1e-20)
  { // if so, then add a bit,until determinant is positive
    printf("Cluster %d has negative determinant for F (%g)\n", clust, det);
    pmF4->add(1e-5);
    det = pmF4->det(&err);
    loops++;
    if (loops > 3)
    { // reduce to diagonal
      for (r=0; r < (int)pmF4->rows(); r++)
        for (c=0; c < (int)pmF4->cols(); c++)
          if (r != c)
            pmF4->setRC(r, c, 0.0);
      loops = 0;
    }
    if (loops > 5)
    {
      printf("Bad cluster statistics!!!\n");
      break;
    }
  }
  if (err < 0)
  {
      printf("Cluster %d Error in determining determinant for F\n", clust);
  }
  if (det > 1e-20)
  { // determinant is OK
    det = pow(det, 1.0/3.0);
    *pmAi4 = pmF4->inversed();
    pmAi4->mult(det);
  }
  return true;
}

///////////////////////////////////////////////////

void UFuzzySplit::countMembers()
{
  UFuzzyElement ** e = elems;
  int i;
  int c;
  //
  for (i = 0; i < clustCnt; i++)
    elemClust[i] = 0;
  for (i = 0; i < elemsCnt; i++)
  {
    c = (*e++)->getClass(clustCnt);
    elemClust[c]++;
  }
}

///////////////////////////////////////////////////

int UFuzzySplit::getMembCount(int clust)
{
  return elemClust[clust];
}

////////////////////////////////////////////////

bool UFuzzySplit::initFromValues(int clust, int inx, int idxCnt)
{
  UMatrix4 * pmV = &mV[clust];
  UMatrix4 * pmF = &mF[clust];
  UMatrix4 * pmAi = &mAi[clust];
  UFuzzyElement ** e;
  int n, i;
  bool result = false;
//  double mCnt = 0.0;
  // initialize matrices
  if (idxCnt > 0)
  {
    e = &elems[inx];
    n = (*e)->getVectorSize();
    pmV->setSize(n, 1);
    pmF->setSize(n, n);
    pmAi->setSize(n, n);
    // range test
    if (inx + idxCnt > elemsCnt)
      printf("UFuzzySplit::initFromValues - Index error %d+%d > %d\n",
          inx, idxCnt, elemsCnt);
    // get center value
    getCenter(-1, 1.0, e, idxCnt, pmV);
    // get covariance value
    e = &elems[inx];
    getCovariance(-1, 1.0, e, idxCnt, pmF, pmV, pmAi);
    // set (false) membership to 0.5
    e = &elems[inx];
    for (i = 0; i < idxCnt; i++)
      (*e++)->setMembValue(0.5, clust);
    result = true;
  }
  return result;
}

//////////////////////////////////////////////////

bool UFuzzySplit::classifyOld(int clusters, double stopVal, int maxLoopCnt)
{
  int i, c;
  int loop = 0;
  bool result = false;
  double epsi;
  UFuzzyElement ** e;
  UMatrix4 * pmV;
  UMatrix4 * pmF;
  UMatrix4 * pmAi;
  int vectSize;
  const double m = 1.2;
  double summu;
  int err;
  double det;
  UMatrix4 mF2;
  const int MSL = 250;
  char s[MSL];
  //
  result = elemsCnt >= clusters;
  if (result)
  {
    e = elems;
    //
    if ((*e)->getMaxClusterCnt() > clusters)
      clustCnt = clusters;
    else
    {
      clustCnt = (*e)->getMaxClusterCnt();
      printf("Number of clusters reduced from %d to %d --- "
          " no more space!\n", clusters, clustCnt);
    }
    vectSize = (*e)->getVectorSize();
    // seeed with random membership
    for (i = 0; i < elemsCnt; i++)
      (*e++)->seedRandom(clustCnt);
    //
    for (c = 0; c < clustCnt; c++)
    { // initialize matrices
      mV[c].setSize(vectSize, 1);
      mF[c].setSize(vectSize, vectSize);
      mAi[c].setSize(vectSize, vectSize);
    }
    while (loop++ < maxLoopCnt)
    { // calculate cluster center
      // debug
/*      mV[0].print("V0");
      mV[1].print("V1");
      mV[2].print("V2");*/
      // debug
      countMembers();
      for (i = 0; i < clustCnt; i++)
      {
        printf("Members of cluster %d is %d\n",
               i, getMembCount(i));
        snprintf(s, MSL, "Cl %d V", i);
        mV[i].print(s);
        snprintf(s, MSL, "Cl %d F", i);
        mF[i].print(s);
        snprintf(s, MSL, "Cl %d Ai", i);
        mAi[i].print(s);
      }
      e = elems;
      for (i = 0; i < elemsCnt; i++)
      {
        (*e)->print("px", clustCnt);
        e++;
      }
        // debug end
      // debug end
      // step 1
      pmV = mV;
      for (c = 0; c < clustCnt; c++)
      {
        pmV->clear();
        summu = 0.0;
        e = elems;
        for (i = 0; i < elemsCnt; i++)
          (*e++)->addPosition(c, m, pmV, &summu);
        pmV->mult(1.0/summu);
        pmV++;
      }
      // step 2
      // calculate covariance matrices F
      pmF = mF;
      pmV = mV;
      for (c = 0; c < clustCnt; c++)
      {
        pmF->clear();
        summu = 0.0;
        e = elems;
        for (i = 0; i < elemsCnt; i++)
          (*e++)->addCovariance(c, m, pmF, pmV, &summu);
        pmF->mult(1.0/summu);
        pmF++;
        pmV++;
      }
      // step 3
      // calculate mahalanobis distance to center
      pmF = mF;
      pmV = mV;
      pmAi = mAi;
      for (c = 0; c < clustCnt; c++)
      { // calculate A matrix
        det = pmF->det(&err);
        if ((det < 1e-20) or (err != 0))
        { // if so, then use old matrices
          printf("Cluster %d has negative determinant for F (%g)\n", c, det);
          if (err != 0)
            printf("Cluster %d Error in determining determinant for F\n", c);
        }
        else
        { // determinant is OK
          det = pow(det, 1.0/3.0);
          *pmAi = pmF->inversed();
          pmAi->mult(det);
        }
        // distances for cluster c
        e = elems;
        for (i = 0; i < elemsCnt; i++)
        {
          (*e)->setDistance(c, pmAi, pmV);
          e++;
        }
        pmAi++;
        pmF++;
        pmV++;
      }
      // debug
/*      e = elems;
      for (i = 0; i < elemsCnt; i++)
      {
        (*e)->print("pix", clustCnt);
        e++;
      }*/
      // debug end
      // step 4
      // update membership
      e = elems;
      for (i = 0; i < elemsCnt; i++)
        (*e++)->updateMembValue(m, clustCnt);
      // step 5
      // finished?
      epsi = 0.0;
      e = elems;
      for (i = 0; i < elemsCnt; i++)
        epsi += (*e++)->getMembValueChange(clustCnt);
      // debug
      countMembers();
      for (i = 0; i < clustCnt; i++)
        printf("Cluster %d has %d members\n", i, getMembCount(i));
      printf("Loop %d epsi=%f\n", loop, epsi);
      // debug end
      if (epsi < stopVal)
        break;
    }
  }
  //
  return result;
}

