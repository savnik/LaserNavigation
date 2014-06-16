/***************************************************************************
 *                                                                         *
 *   \file              rangedata.cpp                                      *
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

#include "rangedata.h"

/////////////////////////////////
// RangeData implementation
//


/**
  \brief Saves range data from the laser scanner
  
  \param *data \n pointer to ULaserData 
  
*/
#if(0)
void RangeData::retrieveData(ULaserData * data)
{
  int r;
  double d;
  
  points = data->getRangeCnt();
  
  for (int i = 0; i < points; i++) {
    // Get angle as radians
    point_th[i] = data->getAngleRad(i);

    // Get range and translate to meters
    r = *data->getRange(i);
    
    if (r < 20) { 
      // less than 20 units is a flag value for URG scanner
      d = OUT_OF_RANGE;
    }
    else {
      switch (data->getUnit()) {
        case 0: d = double(r) * 0.01; break; // cm
        case 1: d = double(r) * 0.001; break; // mm
        case 2: d = double(r) * 0.1; break; // 10cm
        default: d = OUT_OF_RANGE;
      }
    }
    
    point_r[i] = d;
    
  }
  
  this->polarToCart();
  
}
#endif

void RangeData::polarToCart()
{
  for (int i=0; i<points; i++) {
    // Convert point to cartesian
    point_x[i] = point_r[i] * cos(point_th[i]);
    point_y[i] = point_r[i] * sin(point_th[i]);
  }

}
