//============================================================================
// Name        : Ransac.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include "eransac.h"
using namespace std;

inline void calculateLine(double x1, double y1, double x2, double y2,
                          GFLine * line);
inline void calculateLine(double *X, double *Y, int * admit, int lineNumber,
                          int pointCount, GFLine * line);
inline int randInRange(int l, int h);

int splitLine(double * X, double * Y, int * admit,
              int lineNumber, int pointCount, GFLine * line,
              int enableSplit, double splitDist, int minSplitCnt);
              
/**
  Extracts lines from a point cloud using ransac.
  \param Xvalues array if x-vales
  \param Yvalues array of y-values
  \param admit array if integer - will on exit hold line index numbers - first line is 1, no line is 0
  \param valCnt is number of measurements points in X, Y and admit arrays.
  \param lineList are the list of lines on return.
  \returns the number of lines found. */
int ransac(LEL_GFParams * par, double* Xvalues, double* Yvalues, int * admit, int valCnt,
           GFLine * lineList )
{

//  bool admit[pointCount];
  double* X = Xvalues;
  double* Y = Yvalues;
  int pointsLeft = valCnt;
  int lineNumber = 0;
  GFLine * newLine;

  for(int lIn = 0; lIn < par->lineIter; lIn++)
  {
    if (pointsLeft < par->minNoOfPoints)
    {
      break;
    }
    int maxSupport = 0;
    GFLine bestLine(0.0, 0.0, 0.0, 0);
    for(int sIn = 0; sIn < par->sampleIter; sIn++)
    { // get 2 valid points
      int in1 = randInRange( 0, valCnt);
      while (admit[in1] >= 0)
        in1 = randInRange(0, valCnt);
      int in2 = randInRange(0, valCnt);
      while (in1 == in2 or admit[in2] >= 0)
        in2 = randInRange(0, valCnt);
      // get line from points
      GFLine cl;
      calculateLine(X[in1],Y[in1],X[in2],Y[in2], &cl);
      int support = 0;
      for(int pIn = 0; pIn < valCnt; pIn++)
      { // count support for this line
        if (admit[pIn] == -1)
        { // get distance to line
          double dist = cl.A * X[pIn] + cl.B * Y[pIn] + cl.C;
          if(fabs(dist) < par->distThreshold)
            support++;
        }
      }
      if(support > maxSupport)
      { // get best line
        maxSupport = support;
        bestLine = cl;
      }
    }
    //
    if(maxSupport > par->minLineSupport)
    { // line has OK support, so continue
      int tmpSupport = 0;
      newLine = &lineList[lineNumber];
      for(int pIn = 0; pIn < valCnt; pIn++)
      { // find and mark supporters for best line
        if (admit[pIn] == -1)
        { // mark (current) line supportes to admit list
          double dist = bestLine.A * X[pIn] + bestLine.B * Y[pIn] + bestLine.C;
          if (fabs(dist) < par->distThreshold)
          {
            admit[pIn]=lineNumber;
            tmpSupport++;
          }
        }
      }
      // create line structure
      int newLineSupport;
      while(1)
      {
        calculateLine(X, Y, admit, lineNumber, valCnt, newLine);
        bool changeFlag = false;
        newLineSupport = 0;
        for(int pIn = 0; pIn < valCnt; pIn++)
        { // get support for new line
          if (admit[pIn] == -1 or admit[pIn] == lineNumber)
          {
            double dist = newLine->A * X[pIn] + newLine->B * Y[pIn] + newLine->C;
            bool newAdmit = fabs(dist) < par->distThreshold;
            if(newAdmit)
            {
              newLineSupport++;
              if (admit[pIn] == -1)
                changeFlag=true;
              admit[pIn] = lineNumber;
            }
            else if (admit[pIn] == lineNumber)
            { // lost support from this point
              changeFlag = true;
              // release point
              admit[pIn] = -1;
            }
          }
        }
        if(!changeFlag)
          // no change, so stop
          break;
      }
      // test for breaks
      if (par->enableSplit || par->minSplitCnt > 0)
        newLineSupport = splitLine(X, Y, admit, lineNumber, valCnt, newLine,
                                 par->enableSplit, par->splitDist, par->minSplitCnt);
      // save result
      if (newLineSupport > par->minLineSupport)
      {
        newLine->edgeCount = newLineSupport;
        lineNumber++;
      }
      else
      { // line too thin - remove support points
        for(int pIn = 0; pIn < valCnt; pIn++)
          if (admit[pIn] == lineNumber)
            admit[pIn] = -1;
      }
      // count remaining points
      pointsLeft -= newLineSupport;
    }
  }
  return lineNumber;
}

///////////////////////////////

inline int randInRange(int l, int h)
{
  int r = h-l;
  return l+int(double(r)*rand()/(RAND_MAX + 1.0));
}

///////////////////////////////

inline void calculateLine(double x1, double y1, double x2, double y2,
                          GFLine * line)
{
  double ex=x1+x2;
  double ey=y1+y2;
  double ex2=x1*x1+x2*x2;
  double ey2=y1*y1+y2*y2;
  double exy=x1*y1+x2*y2;

  double th=atan2(2.0*ex*ey-4.0*exy, ex*ex-ey*ey-2.0*(ex2-ey2))/2.0;
  double R=(ex/2.0)*cos(th)+(ey/2.0)*sin(th);

  double A = cos(th);
  double B = sin(th);

  if(A<0) {
          line->set(-A,-B,R);
  } else {
          line->set(A,B,-R);
  }
}

///////////////////////////////

inline void calculateLine(double *X, double *Y, int * admit,
                          int lineNumber, int pointCount, GFLine * line)
{
  double ex=0.0, ey=0.0, ex2=0.0, ey2=0.0, exy=0.0, xc, yc, lb, le;
  int n=0, PC =pointCount;
  int p1 = 0;
  while(PC--)
  {
    if(admit[PC] == lineNumber)
    {
      xc=X[PC];
      yc=Y[PC];
      ex += xc;
      ex2 += xc * xc;
      ey += yc;
      exy += xc * yc;
      ey2 += yc * yc;
      p1 = PC; // first line point
      n++;
    }
  }
  double th = atan2(2.0 * ex * ey - 2.0 * n * exy,
                    ex * ex - ey * ey - n * (ex2 - ey2)) / 2.0;
  double R = (ex/(float)n) * cos(th) + (ey/(float)n) * sin(th);

  double A = cos(th);
  double B = sin(th);
  double C;

  if(A < 0)
  { // make A positive or zero
    A = -A;
    B = -B;
    C = R;
  } else
  {
    A = A;
    B = B;
    C = -R;
  }
  // find end points
  lb = X[p1] * B - Y[p1] * A;
  le = lb;
  while(p1 <  pointCount)
  {
    if(admit[p1] == lineNumber)
    {
      double l = X[p1] * B - Y[p1] * A;
      if (l>le)
        le=l;
      else if (l<lb)
        lb=l;
    }
    p1++;
  }
  if (line != NULL)
  {
    line->set(A, B, C, n);
    line->setEndpoints(lb, le);
  }
}

///////////////////////////////////////////////////////////////

/**
Search the line for splits - assuming data from a scanner, i.e. points are in angle order.
\param (X,Y) are the point position
\param admit is the list of markers for the points admitted to this line
\param lineNumber is the number of this line
\param pntCnt is the number of points in x,y array.
\param line is the line data - that may need recalculation.
\param enableSplit if true, then only the largest supporter group are maintained
\param splitDist is the maximum opening in supporting points for the line
\param minSplitCnt is the minimum number of points in a split group.
\returns the new number of points supporting the line. */
int splitLine(double * X, double * Y, int * admit,
              int lineNumber, int pntCnt, GFLine * line,
              int enableSplit, double splitDist, int minSplitCnt)
{
  int i2 = -1;
  int g0 = 0, g1 = 0;
  int gm0 = 0, gm1 = 0, gmCnt =0;
  int grpCnt = 0;
  double dist;
  int newCnt = line->edgeCount;
  //
  for(int i = 0; i <= pntCnt; i++)
  { // test all points - and one extra to close any open groups
    if (i == pntCnt || admit[i] == lineNumber)
    { // point belong to this line (or finished)
      if (i2 >= 0)
      { // not first point, so test for breaks
        if (i == pntCnt)
          // last point is a break per definition
          dist = 1e3;
        else
          // distance from last point
          dist = hypot(Y[i] - Y[i2], X[i] - X[i2]);
        if (dist > splitDist)
        { // a split is detected
          if (grpCnt <= minSplitCnt)
          { // remove points
            for (int j = g0; j <= g1; j++)
            { // remove if part of line
              if (admit[j] == lineNumber)
                admit[j] = -1;
            }
            newCnt -= grpCnt;
          }
          else if (grpCnt > gmCnt)
          { // best group so far
            gmCnt = grpCnt;
            gm0 = g0;
            gm1 = g1;
          }
          grpCnt = 0;
          g0 = i;
        }
      }
      g1 = i;
      grpCnt++;
      i2 = i;
    }
  }
  if (enableSplit)
  {
    newCnt = 0;
    for (int i = 0; i < pntCnt; i++)
    {
      if (admit[i] == lineNumber)
      {
        if (i < gm0 || i > gm1)
          // remove support outside best group
          admit[i] = -1;
        else
          // count new support
          newCnt++;
      }
    }
  }
  if (newCnt != line->edgeCount)
  { // we need a new line calculation
    calculateLine(X, Y, admit, lineNumber, pntCnt, line);
    line->edgeCount = newCnt;
  }
  return newCnt;
}

