/***************************************************************************
 *                                                                         *
 *   \file              aucircle.h                                         *
 *   \author            Lars Valdemar Mogensen                             *
 *   \date              Nov 2007                                           *
 *   \brief             Circle class implementation                        *
 *                                                                         *
 *                      Copyright (C) 2007 by DTU                          *
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

#ifndef AUCIRCLE_H
#define AUCIRCLE_H

#include "aufeature.h"

/**
  \brief AUCircle class for the return of data features

 */
class AUCircle : public AUFeature
{
  public:
  /**
    Constructor */
    AUCircle();
  /**
    Destructor */
    virtual ~AUCircle();
  /**
    Return name of this object type */
    virtual const char * getType();
  /**
  Print status in XML format */
    virtual void print();
  /**
  Clear internal variables */
    void clear();

  public:

    bool polarValid;
    // Polar representation
    double cr;              //!< Distance to center of circle
    double cth;             //!< Angle of circle center

    bool cartValid;
    // Cartesian representation
    double cx,cy;            //!< Coordinate of circle

    // Common data
    double r;              //!< Circle radius
    double msq;            //!< Mean Square Error for the line fit
    double coverage;       //!< Radians of the circle covered by scan points
    int points;            //!< Number of data points used for fit
};

#endif
