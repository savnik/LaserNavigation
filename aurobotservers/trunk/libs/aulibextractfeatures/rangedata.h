/***************************************************************************
 *                                                                         *
 *   \file              rangedata.h                                        *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Dec 2006                                           *
 *   \brief             Rangedata class implementation                     *
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


#ifndef RANGEDATA_H
#define RANGEDATA_H

#include <cstdlib>

#include <stdio.h>
#include <math.h>

#include "auline.h"
#include "aucircle.h"

// RangeData class holds the laser range data
//
#define POINTS_MAX 1000
#define OUT_OF_RANGE -1  // out of range (meters)

#define TABLE_MAX 2000

/**
  \brief RangeData class

*/
class RangeData
{
public:

  // Import data from ULaserData object to internal format (SI units)
  //void retrieveData(ULaserData * data);
  void polarToCart();

  int points;                            //!< Points in scan
  double point_th[POINTS_MAX];           //!< Angles
  double point_r[POINTS_MAX];            //!< Ranges

  double point_x[POINTS_MAX];            //!< Point x coordinate
  double point_y[POINTS_MAX];            //!< Point y coordinate

};

#endif
