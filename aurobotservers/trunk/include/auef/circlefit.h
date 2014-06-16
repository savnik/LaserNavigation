/***************************************************************************
 *                                                                         *
 *   \file              circlefit.h                                        *
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

#ifndef EXTRACTCIRCLE_H
#define EXTRACTCIRCLE_H


#include "rangedata.h"
#include "polarlinefit.h"

#define CIRCLE_LINES_MAX 5

/**
  \brief CircleFit class

  The CircleFit class implements a method for fittig PolarLineFit objects to a
  circle.

*/
class CircleFit
{

public:

  CircleFit();

  void fitLines(RangeData *range_, PolarLineFit *linefit1_, PolarLineFit *linefit2_);

  void absorbLine(PolarLineFit *line_);

  double testDistToCenter(PolarLineFit *line_);
  double testMSQ(PolarLineFit *line_);

  double getCx();
  double getCy();
  double getCr();
  double getCth();

  double getMSQ();
  double getRadius();
  double getCoverage(); //!< radians of the circle covered by scan points
  double getInverted();

  int getFirst();
  int getLast();

  void toStringPolar(char *target);
  void toStringCart(char *target);

private:
  RangeData *range;
  int first;
  int last;
  int points;
  bool inverted;

  double sumsq;
  double sumdist;

  double msq;

  double cx;
  double cy;
  double radius;
  double cr;
  double cth;

  // functions
  // Matches a circle close the two line segments, preferable through the end points
  void matchLines(PolarLineFit *line1, PolarLineFit *line2);

  void addLine(PolarLineFit *line_);

  // Distance from a polar point to the circumference of the circle
  double distToCirc(double r, double th);
  double distToCenterSqr(double r_, double th_);

};


#endif
