/***************************************************************************
 *                                                                         *
 *   \file              circlefit.cpp                                      *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Dec 2006                                           *
 *   \brief             Implementation of circlefit class                  *
 *                                                                         *
 *   Fits a circle through line segments.                                  *
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



#include "circlefit.h"

#include <stdio.h>

/// \brief Constructor
CircleFit::CircleFit()
{
  msq = 0;
  sumsq = 0;
  sumdist = 0;
  points = 0;
  cx = 0;
  cy = 0;
  radius = 0;
  cr = 0;
  cth = 0;
}


/**
  \brief Makes a circle of two PolarLineFit objects
  
  Fit a circle to the two the lines given by the PolarLineFit objects. \n
  the center for the circle is found in the perpendicular median vector 
  intersection and the radius is adjusted by the average distance to the center
  
  \param *range_, *linefit1_, *linefit2_ \n
  range_: Pointer to RangeData object.  \n
  linefitX_: Pointer to PolarLineFit object.
*/
void CircleFit::fitLines(RangeData *range_, PolarLineFit *linefit1_, PolarLineFit *linefit2_)
{
  // Reset vars
  sumsq = 0;
  sumdist = 0;
  points = 0;

  range = range_;
  
  if (linefit1_->getFirst() > linefit2_->getFirst()){
    first = linefit2_->getFirst();
    last = linefit1_->getLast();
  }
  else {
    first = linefit1_->getFirst();
    last = linefit2_->getLast();
  } 
  
  matchLines(linefit1_, linefit2_);
  
  addLine(linefit1_);
  addLine(linefit2_);

}



/**
  \brief Add line segment to circle

  Add one PolarLineFit object (line segment) to an existing circle
  
  \param *line_ \n
  Pointer to PolarLineFit object.
*/
void CircleFit::absorbLine(PolarLineFit *line_)
{
  
  if (first > line_->getFirst())
    first = line_->getFirst();
  else if (last < line_->getLast())
    last = line_->getLast();
  
  addLine(line_);
  
  
}



/**
  \brief Get the distance to the center for a PolarLineFit object
  
  \param *line_ \n
  Pointer to PolarLineFit object.
  
  \return 
  The distance from the line midpoint to the circle circumference.
*/
double CircleFit::testDistToCenter(PolarLineFit *line_)
{
  double r, th;
  // Find midpoint of line segment
  th = (line_->getStartTh() + line_->getEndTh())/2.0;
  r = line_->getR0()/(cos(th - line_->getTh0()));
  
  
  // find distance from center to line midpoint, should be approx = radius
  return (distToCirc(r, th) + radius);
  
}


/**
  \brief Squared distance from polar point to circle center
  
  \param r_, th_ \n
  Polar point defined by range r_ and angle th_. 
  
  \return Squared distance to center.
*/
double CircleFit::distToCenterSqr(double r_, double th_)
{
  double a, b;
  
  // Convert to cartesian
  a = cx - r_ * cos(th_);
  b = cy - r_ * sin(th_);
 
  return a * a + b * b;
  
}

/**
  \brief Distance from polar point to circumference

  \param r_, th_ \n
  Polar point defined by range r_ and angle th_. 
  
  \return Distance to circumference.
*/
double CircleFit::distToCirc(double r_, double th_)
{
 
  return sqrt(distToCenterSqr(r_, th_)) - radius;
  
}



/**
  \brief MSQ for a line segment matched to an existing circle

  \param *line_ \n
  Pointer to PolarLineFit object.
  
  \return 
  The MSQ between the PolarLineFit object and the circle circumference.
*/
double CircleFit::testMSQ(PolarLineFit *line_)
{
  double sq=0, d;
  int p;
  int i;

  for (i=line_->getFirst(); i<=line_->getLast(); i++) {
    d = distToCirc(range->point_r[i], range->point_th[i]);
    sq += d*d;
  } 

  p = (line_->getLast()-line_->getFirst()+1);
  
  return sq / p;
  
}



/**
  \brief Add the line attributes to an existing circle
  
  \param *line_ \n
  Pointer to PolarLineFit object.
*/
void CircleFit::addLine(PolarLineFit *line_)
{
  double d;
//   double r;
  
  for (int i=line_->getFirst(); i<=line_->getLast(); i++) {
    d = distToCenterSqr(range->point_r[i], range->point_th[i]);
    sumsq += d;

    sumdist += sqrt(d);    
  }   
  
  points += (line_->getLast()-line_->getFirst()+1);
  
  radius = sumdist/points;
  
  msq = sumsq/points - radius * radius;

}


/**
  \brief Match two PolarLine objects to a circle
  
  \param *line1, *line2 \n
  pointers to the two PolarLine objects that needs to be matched.
*/
void CircleFit::matchLines(PolarLineFit *line1, PolarLineFit *line2)
{
  double m1x, m1y, m2x, m2y, v1x, v1y, v2x, v2y;
  double r, th;
  
  // Find midpoint of lines in cartesian coordinates
  th = (line1->getStartTh() + line1->getEndTh())/2.0;
  r = line1->getR0()/(cos(th - line1->getTh0()));
  
  m1x = r * cos(th);
  m1y = r * sin(th);

  th = (line2->getStartTh() + line2->getEndTh())/2.0;
  r = line2->getR0()/(cos(th - line2->getTh0()));

  m2x = r * cos(th);
  m2y = r * sin(th);
  
  // Calculate the perpendicular median vectors 
  v1x = cos(line1->getTh0());
  v1y = sin(line1->getTh0());
  v2x = cos(line2->getTh0());
  v2y = sin(line2->getTh0());

  // find median intersection
  r = (m2x - m1x) * v2y - (m2y - m1y) * v2x;
  r = r / (v1x * v2y - v1y * v2x);

  inverted = r < 0;
  
  // find center
  cx = m1x + r * v1x;
  cy = m1y + r * v1y;
  
  // calc radius
  r = (cx - m1x);
  th = (cy - m1y);
  
  radius = sqrt(r * r + th * th);  

  // Do polar coordinates also
  cr = sqrt(cx * cx + cy * cy);
  cth = atan2(cy, cx);
  
}



/// \brief Write center dist, angle and radius to target as XML
/// \param *target \n Pointer to string. \n Overflow is NOT checked.
void CircleFit::toStringPolar(char *target)
{
  sprintf(target, "<circle center_r=\"%g\" center_th=\"%g\" radius=\"%f\" msq=\"%g\" coverage=\"%g\" dots=\"%d\"/>\n",
          cr, cth, radius, msq, this->getCoverage(),abs(first-last)+1);
}

/// \brief Write center coordinates and radius to target as XML
/// \param *target \n Pointer to string. \n Overflow is NOT checked.
void CircleFit::toStringCart(char *target)
{
  sprintf(target, "<circle x=\"%g\" y=\"%g\" radius=\"%g\" msq=\"%g\" coverage=\"%g\" dots=\"%d\"/>\n",
          cx, cy, radius, msq, this->getCoverage(),abs(first-last)+1);
}


/// \brief Radians of the circle covered by scan points
/// \return Radians
double CircleFit::getCoverage()
{
  // Find angle of first point to center and last point to center
  
  double a;
  
  a = - atan2(range->point_y[last] - cy, range->point_x[last] - cx)
      + atan2(range->point_y[first] - cy, range->point_x[first] - cx);
  
  if (a > 2*M_PI)
    a -= 2*M_PI;

  if (a < 0)
    a += 2*M_PI;

  return a;
}



// Simple get functions

/// \brief Get center x
/// \return cx
double CircleFit::getCx()
{
  return cx;  
}

/// \brief Get center y
/// \return cy
double CircleFit::getCy()
{
  return cy;  
}

/// \brief Get center r
/// \return cr
double CircleFit::getCr()
{
  return cr;  
}

/// \brief Get center th
/// \return cth
double CircleFit::getCth()
{
  return cth;  
}

/// \brief Mean squared error
/// \return MSQ
double CircleFit::getMSQ()
{
  return msq;  
}

double CircleFit::getRadius()
{
  return radius;
}

/// \brief True if inverted
double CircleFit::getInverted()
{
  return inverted;  
}

/// \brief First data point in circle segment
/// \return First data point in circle from RangeData
int CircleFit::getFirst()
{
  return first;
}

/// \brief Last data point in circle segment
/// \return Last data point in circle from RangeData
int CircleFit::getLast()
{
  return last;
}

