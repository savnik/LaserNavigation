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
#include "ufuzzypixel.h"

UFuzzyPixel::UFuzzyPixel()
        : UFuzzyElement()
{
  int i;
  for (i = 0; i < MAX_CLUSTER_CNT; i++)
  {
    mu1[i] = 0.0;
    mu2[i] = 0.0;
  }
}

///////////////////////////////////////////////

UFuzzyPixel::~UFuzzyPixel()
{}

///////////////////////////////////////////////

bool UFuzzyPixel::addPosition(int clust, double m, UMatrix * mAvg, double * membSum)
{
  double mum;
  double * v = mAvg->getData();
  bool result = true;
  //
  if (clust >= 0)
    // normal member value
    mum = pow(mu1[clust], m);
  else
    // full membership
    mum = 1.0;
  if (pix != NULL)
  { // add distance
    *v++ += mum * double(pix->p1);
    *v++ += mum * double(pix->p2);
    *v   += mum * double(pix->p3);
    *membSum += mum;
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////

bool UFuzzyPixel::addCovariance(int clust,
                                double m,
                                UMatrix * mF,
                                UMatrix * mV,
                                double * membSum)
{
  double mum;
  UMatrix4 mX(3, 1);
  UMatrix4 mXt;
  UMatrix4 mSq;
  bool result = true;
  double * vx = mX.getData();
  double * vv = mV->getData();
  //
  if (clust >= 0)
    mum = pow(mu1[clust], m);
  else
    mum = 1.0;
  if (pix != NULL)
  { // set matrix values as distance to center
    *vx++ = double(pix->p1) - *vv++;
    *vx++ = double(pix->p2) - *vv++;
    *vx   = double(pix->p3) - *vv;
    // make a transposed vector
    mXt = mX;
    mXt.setSize(1, 3);
    // Make outher product
    mSq.mult(&mX, &mXt);
    // scanle with membership
    if (clust >= 0)
      mSq.mult(mum);
    // add to covatiance matrix
    mF->add(&mSq);
    // add sum of weight too
    *membSum += mum;
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////

bool UFuzzyPixel::setDistance(int clust, UMatrix * mAi, UMatrix * mV)
{
  UMatrix4 mX(3,1);
  UMatrix4 mXt;
  UMatrix4 mD;
  UMatrix4 * pmAi = (UMatrix4 *)mAi;
  bool result = true;
  double * vx = mX.getData();
  double * vv = mV->getData();
  if (pix != NULL)
  { // set matrix values as distance to center
    *vx++ = double(pix->p1) - *vv++;
    *vx++ = double(pix->p2) - *vv++;
    *vx   = double(pix->p3) - *vv;
    // make a transposed vector
    mXt = mX;
    mXt.setSize(1,3);
    // get distance as mahalanobi distance
    mD = mXt * (*pmAi) * mX;
    // get only value
    d[clust] = mD.get(0, 0);
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////

void UFuzzyPixel::updateMembValue(double m, int clustCnt)
{
  int i, c;
  double mx = 1.0 / (m - 1.0);
  double sd;
  double * pdc;
  double * pd;
  int negD = 0;
  //
  for (c = 0; c < clustCnt; c++)
    if (d[c] <= 0.0)
      negD++;
  if (negD == 0)
  { // no negative
    for (c = 0; c < clustCnt; c++)
    { // update cluster c
      pdc = &d[c];
      pd = d;
      sd = 0.0;
      for (i = 0; i < clustCnt; i++)
        sd += pow(*pdc / *pd++, mx);
      // save old value
      mu2[c] = mu1[c];
      // get new value
      mu1[c] = 1.0 / sd;
    }
  }
  else
  { // one or more negative distances
    for (c = 0; c < clustCnt; c++)
    { // save old value
      mu2[c] = mu1[c];
      // set value
      if (d[c] <= 0.0)
        mu1[c] = 1.0/double(negD);
      else
        mu1[c] = 0.0;
    }
  }
}

///////////////////////////////////////////////

int UFuzzyPixel::getVectorSize()
{ // three values in a colour pixel
  return 3;
}

///////////////////////////////////////////////

double UFuzzyPixel::getMembValueChange(int clustCnt)
{
  int i;
  double result = 0;
  int n = getVectorSize();
  //
  for (i = 0; i < n; i++)
    result += sqr(mu1[i] - mu2[i]);
  result /= double(n);
  return result;
}

///////////////////////////////////////////////

void UFuzzyPixel::setMembValue(double value, int clust)
{
  mu1[clust] = value;
}

///////////////////////////////////////////////

void UFuzzyPixel::seedRandom(int clustCnt)
{
  int i;
  double * m = mu1;
  double p = 1.0;
  //
  for (i = 0; i < clustCnt - 1; i++)
  { // the first clCnt clusters are given a random
        // share p = ([0..1[)/(clCnt-1), ...
    *m = double(rand() / ((double(RAND_MAX) + 1.0) * (clustCnt - 1)));
    p -= *m++;
  }
  // the last one gets the rest;
  *m = p;
}

///////////////////////////////////////////////

int UFuzzyPixel::getClass(int clustCnt)
{
  int i;
  double max = mu1[0];
  double * v = mu1;
  int result = 0;
  //
  for (i = 1; i < clustCnt; i++)
  {
    if (*++v > max)
    {
      max = *v;
      result = i;
    }
  }
  return result;
}

/////////////////////////////////////////////

int UFuzzyPixel::getMaxClusterCnt()
{
  return MAX_CLUSTER_CNT;
}

/////////////////////////////////////////////


void UFuzzyPixel::print(const char * prestring, int clustCnt)
{
  int i;
  printf("%s %4d%4d%4d u: %5.3f",
         prestring, pix->p1, pix->p2, pix->p3, sqrt(mu1[0]));
  for (i = 1; i < clustCnt; i++)
    printf(",%5.3f",  sqrt(mu1[i]));
  printf(" (%d) d: %8g", getClass(clustCnt), d[0]);
  for (i = 1; i < clustCnt; i++)
    printf(",%8g",  sqrt(d[i]));
  printf("\n");
}

