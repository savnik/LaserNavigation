/***************************************************************************
 *                                                                         *
 *   \file              polarlinefit.h                                     *
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


#ifndef POLARLINEFIT_H
#define POLARLINEFIT_H

#include "rangedata.h"


/// \brief PolarLineFit fits a polar line to a section of points,
/// in a RangeData object
///
class PolarLineFit
{

public:

  PolarLineFit();

  void fitRange(RangeData *range_, int first_, int last_);
  void fitRangeOld(RangeData *range_, int first_, int last_);
  double distToLine(double r_, double th_);

  double getMSQ();
  double getMaxDist();
  double getStartTh();
  double getEndTh();
  double getR0();
  double getTh0();
  double getLength();
  double getSlope();


  int getFirst();
  int getLast();

  // Writes the line vars as an XML tag to target
  void toStringPolar(char *target);
  void toStringCart(char *target);
  bool asCart(double * x, double * y, double * th, double * lng, double * res);

private:
  RangeData *range;
  int first;
  int last;
  double msq;
  double maxdist;

  double start_th;       //!< Start angle
  double end_th;         //!< End angle
  double r0;             //!< Minimum distance to line
  double th0;            //!< Angle of minimum distance
  double length;         //!< Length of line segment
  double slope;           //!< Slope of the line from (x1, y1) to (x2, y2)

  // functions
  double calcMSQ();
  double projectToLine(double r, double th); //!< returns theta
  void setLength();

};

#endif
