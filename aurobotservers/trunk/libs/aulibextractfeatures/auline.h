/***************************************************************************
 *                                                                         *
 *   \file              auline.h                                           *
 *   \author            Lars Valdemar Mogensen                             *
 *   \date              Nov 2007                                           *
 *   \brief             Line class implementation                          *
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

#ifndef AULINE_H
#define AULINE_H

#include "aufeature.h"

/**
  \brief AULine class for the return of data features
  
 */
class AULine : public AUFeature
{
  public:
  /**
    Constructor */
    AULine();
  /**
    Destructor */
    virtual ~AULine();
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
    double start_th;       //!< Start angle
    double end_th;         //!< End angle
    double r0;             //!< Minimum distance to line
    double th0;            //!< Angle of minimum distance
    
    bool cartValid;
    // Cartesian representation
    double x1,y1;          //!< Start point
    double x2,y2;          //!< End point
    
    // Common data
    double length;         //!< Length of line segment
    double slope;          //!< Slope of the line from (x1, y1) to (x2, y2)
    double msq;            //!< Mean Square Error for the line fit
    int points;            //!< Number of data points used for fit
};

#endif
