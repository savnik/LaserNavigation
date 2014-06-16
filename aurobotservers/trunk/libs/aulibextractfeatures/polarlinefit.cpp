/***************************************************************************
 *                                                                         *
 *   \file              polarlinefit.cpp                                   *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Dec 2006                                           *
 *   \brief             Fits a data range to a (polar) line                *
 *                                                                         *
 *                                                                         *
 *                      Copyright (C) 2006 by DTU                          *
 *                      rse@oersted.dtu.dk                                 *
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
 


using namespace std;

// #include "polarline.h"
#include "polarlinefit.h"


/// \brief Constructor
PolarLineFit::PolarLineFit()
{
  start_th = 0;
  end_th = 0;
  r0 = 0;
  th0 = 0;
}



/**
  \brief Fit range to a polar line
  
  \param *range_, first_, last_ \n
  Pointer to the data set and the first and last point in between which the line
  should be fittet  
*/
void PolarLineFit::fitRangeOld(RangeData *range_, int first_, int last_)
{
  double p,q,th,r;
  int i,j;

  range = range_;
  
  if (first_ < last_) {  
    first = first_;
    last = last_;
  }
  else {
    first = last_;
    last = first_;
  }
  
  p=0;
  q=0;
  
  // Calc p and q
  
  for (i=first; i<=last; i++) {
    for (j=i+1; j<=last; j++) {
      p += range->point_r[i] * range->point_r[j] * sin(range->point_th[i] + range->point_th[j]);
      q += range->point_r[i] * range->point_r[j] * cos(range->point_th[i] + range->point_th[j]);
    }
  }
  
  p *= 2.0;
  q *= 2.0;
  
  for (i=first; i<=last; i++) {
    p += (1-(last-first+1)) * range->point_r[i] * range->point_r[i] * sin(2 * range->point_th[i]);
    q += (1-(last-first+1)) * range->point_r[i] * range->point_r[i] * cos(2 * range->point_th[i]);
  }
  
  // Calc line theta
  th = atan2(p,q) / 2.0;  
  
  // Calc line radius
  r = 0;  
  for (i=first; i<=last; i++) {
    r += range->point_r[i] * cos(range->point_th[i] - th);
  }
  
  r = r / (last-first+1);
  
  if (r<0){
    r = -r;
    th += M_PI;
  }
  
  r0 = r;
  th0 = th;
  
  start_th = projectToLine(range->point_r[first], range->point_th[first]);
  end_th = projectToLine(range->point_r[last], range->point_th[last]);
  
  setLength();
  
  msq = calcMSQ();
  
}



/**
  \brief Fit range to a polar line
  
  \param *range_, first_, last_ \n
  Pointer to the data set and the first and last point in between which the line
  should be fittet  
*/
void PolarLineFit::fitRange(RangeData *range_, int first_, int last_)
{
  double n,x,y;
  double sumx,sumy,sumxx,sumyy,sumxy;
  int i;
  
  range = range_;
  
  if (first_ < last_) {  
    first = first_;
    last = last_;
  }
  else {
    first = last_;
    last = first_;
  }
  
  n = last-first+1;
  
  sumx = 0;
  sumy = 0;
  sumxx = 0;
  sumyy = 0;
  sumxy = 0;
  
  // Calc sums
  for (i=first; i<=last; i++) {
    x = range->point_x[i];
    y = range->point_y[i];
    sumx += x;
    sumy += y;
    sumxx += x * x;
    sumyy += y * y;
    sumxy += x * y;
  }
 
  // Calc line theta
  th0 = atan2(2*(sumx*sumy - sumxy*n), 
              sumx*sumx - sumy*sumy + n*(sumyy - sumxx)) * 0.5;  
    
  // Calc line radius
  r0 = (sumx*cos(th0) + sumy*sin(th0))/n;
  
  if (r0 < 0) {
    r0 = -r0;
    th0 += M_PI;
  }
  
  // Ensure th0 is in range -pi to pi
  if (th0 > M_PI)
    th0 -= 2*M_PI;
  
  if (th0 < -M_PI)
    th0 += 2*M_PI;
    
  
  start_th = projectToLine(range->point_r[first], range->point_th[first]);
  end_th = projectToLine(range->point_r[last], range->point_th[last]);
  
  setLength();
  
  msq = calcMSQ();
  
}

/**
  \brief Writes line parameters as polar coordinates to string in XML format
  \param *target \n
  writes parameters to target */
void PolarLineFit::toStringPolar(char *target)
{
  sprintf(target, "<line start_th=\"%g\" end_th=\"%g\" r=\"%g\" th=\"%g\" msq=\"%g\" l=\"%g\" dots=\"%d\" />\n",
    start_th, end_th, r0, th0, msq, length, last-first+1);
}

/**
  \brief Writes line parameters as cartesian coordinates to string in XML format
  \param *target \n
  writes parameters to target */
void PolarLineFit::toStringCart(char *target)
{
  double x1, y1, r;
  
  r = r0/cos(th0 - start_th);  
  x1 = r * cos(start_th);
  y1 = r * sin(start_th);

  
  sprintf(target, "<line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\" />\n",
    x1, y1, slope, length, msq);
}

/**
  \brief converts the estimated line segment to cartesian coordinates
  \param [out] x,y start position of line
  \param [out] th,lng orientation and length
  \param [out] res square error for measurement to line estimate
   */
bool PolarLineFit::asCart(double * x, double * y, double * th, double * lng, double * res)
{
  if (th0 == start_th)
    return false;
  else
  {
    double r = r0/cos(th0 - start_th);
    *x = r * cos(start_th);
    *y = r * sin(start_th);
    *th = slope;
    *lng = length;
    *res = msq;
    return true;
  }
}

/// \brief calculates the MSQ for all points to the line
/// \return MSQ
double PolarLineFit::calcMSQ()
{
  double sq=0, d;
  
  maxdist = 0;
  
  for (int i=first; i<=last; i++) {
    d = fabs(distToLine(range->point_r[i], range->point_th[i]));
    
    if (d>maxdist)
      maxdist = d;
      
    sq += d*d;
  } 
  
  return sq / (last-first+1);

}


/**
  \brief Set length of line segment and the slope of the line */
void PolarLineFit::setLength()
{
  double x1, y1, x2, y2, a, b;
  
  a = r0/cos(th0 - start_th);  
  x1 = a * cos(start_th);
  y1 = a * sin(start_th);

  a = r0/cos(th0 - end_th);
  x2 = a * cos(end_th);
  y2 = a * sin(end_th);

  a = (y2-y1);
  b = (x2-x1);
  slope = atan2(a, b);
  
  length = sqrt(a * a + b * b);
  
}

/**
  \brief Calculates the distance from a polar point to the line
  
  \param r_, th_ \n
  The range and angle of the polar point
  
  \return Distance to the line
*/
double PolarLineFit::distToLine(double r_, double th_)
{
  double adj, dist;
  
  adj = cos(th_ - th0) * r_;
  dist = r0 - adj;  
  
  return dist;
  
}

/**
  \brief projectToLine projects the point (r, th) to line

  \param r, th
  The range and angle of the polar point.
  
  \return Angle of projection
*/
double PolarLineFit::projectToLine(double r, double th)
{
  double p,Q;
  
  p = sin(th - th0) * r;
  Q = th0 + atan2(p, r0);
  
  if (Q > M_PI) 
    Q -= 2*M_PI;
  
  return Q;
  
}


// Get functions


/**
  \brief Get start angle
  \returns Start angle */
double PolarLineFit::getStartTh()
{
  return start_th;  
}


/**
  \brief Get end angle
  \returns End angle */
double PolarLineFit::getEndTh()
{
  return end_th;
}


/**
  \brief Get minimum range
  \returns Minimum range */
double PolarLineFit::getR0()
{
  return r0;  
}


/**
  \brief Get angle for minimum range
  \returns Minimum range angle */
double PolarLineFit::getTh0()
{
  return th0;  
}

/**
  \brief Get length of line segment
  \returns Length of line segment */
double PolarLineFit::getLength()
{
  return length;
}

/**
  \brief Get slope of line segment
  \returns slope of line segment */
double PolarLineFit::getSlope()
{
  return slope;
}




/// \return MSQ
double PolarLineFit::getMSQ()
{
  return msq;
}

/// \return Maximum distance
double PolarLineFit::getMaxDist()
{
  return maxdist;
}

/// \return First datapoint (index)
int PolarLineFit::getFirst()
{
  return first;
}

/// \return Last datapoint (index)
int PolarLineFit::getLast()
{
  return last;
}



