/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef AU2DLINESEG_H
#define AU2DLINESEG_H

#include <ugen4/udatabase.h>
#include <ugen4/u2dline.h>

/**
2D line segment - invented as result type for 'scan features', and is compatible with inter resource data transfer (as it has a UDataBase source type

	@author Christian <chrand@mail.dk>
*/
class AU2DLineSeg : public UDataBase
{
public:
  /**
  Constructor */
  AU2DLineSeg();
  /**
  Destructor */
  virtual ~AU2DLineSeg();
  /**
  \brief Get type of this structure */
  virtual const char * getDataType()
  { return "2dLineSeg"; };
  /**
  \brief Code line as an XML tag to this target string.
  \param target char buffer, where to put coded data.
  \param targetCnt is the length of the target buffer.
  \returns a pointer to to the target string. */
  char * toXMLString(char *target, const int targetCnt, const char * extra = NULL);
  /** \brief set line from direct values */
  void setLine(double vx, double vy, double vth, double vlen, double vresSQ);
  /** \brief get x,y coordinates ot the other end of line */
  void getOtherEnd(double * x2, double * y2);

public:
  /// start point of line x in meter
  double x;
  /// start point of line y in meter
  double y;
  /// orientation of line in radians
  double th;
  /// length of line in meters
  double length;
  /// mean square error (square residual) when fitted
  double resSQ;
};

#endif
