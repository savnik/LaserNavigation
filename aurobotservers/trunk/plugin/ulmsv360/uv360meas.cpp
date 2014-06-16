/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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
#include "uv360meas.h"

////////////////////////////////////////////////////////

UV360Meas::UV360Meas()
{
}

////////////////////////////////////////////////////////

UV360Meas::~UV360Meas()
{
}

////////////////////////////////////////////////////////

void UV360Meas::set(double rng, double ang, UTime t)
{
  position.x = cos(ang) * rng;
  position.y = sin(ang) * rng;
  range = rng;
  angle = ang;
  updTime = t;
}

////////////////////////////////////////////////////////

void UV360Meas::set(UPosition pos, double rng, double ang, UTime t)
{
  position = pos;
  range = rng;
  angle = ang;
  updTime = t;
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


UV360Sect::UV360Sect()
{
  measCnt = 0;
}

////////////////////////////////////////////////////////

UV360Sect::~UV360Sect()
{
}

////////////////////////////////////////////////////////

void UV360Sect::addMeas(double rng, double angle, UTime t)
{
  UV360Meas * m1;
  //
  m1 = getMeasSlot(rng, t);
  if (m1 != NULL)
    // set the new data at this position 
    m1->set(rng, angle, t);
}

////////////////////////////////////////////////////////

bool UV360Sect::addMeas(UPosition pos, double rng, double angle, UTime t)
{ // position known already 
  UV360Meas * m1;
  //
  m1 = getMeasSlot(rng, t);
  if (m1 != NULL)
    // set the new data at this position 
    m1->set(pos, rng, angle, t);
  //
  return m1 != NULL;
}

/////////////////////////////////////////////////////////

UV360Meas * UV360Sect::getMeasSlot(double rng, UTime t)
{
  int n = 0;
  UV360Meas * m1 = meas;
  //
  while (n < measCnt)
  { // test age -- newest in front 
    if (t < m1->getT())
    { // is older, so not in front
      m1++;
      n++;
    }
    else
      break;
  }
  while (n < measCnt)
  { // if same age - then sort on range
    if ((t == m1->getT()) and (rng > m1->getRange()))
    { // same age, but at longer range 
      m1++;
      n++;
    }
    else
      break;
  }
  if (n < measCnt)
  { // move measurements by one
    if (measCnt < measMaxCnt)
    { // move all measurements by one 
      memmove(&meas[n+1], m1, sizeof(UV360Meas) * (measCnt - n));
      measCnt++;
    }
    else if (n < (measMaxCnt - 1))
      // overwrite last measurement - the oldest and furthest away 
      memmove(&meas[n+1], m1, sizeof(UV360Meas) * (measCnt - n - 1));
  }
  else if (measCnt < measMaxCnt)
    // no move - just add
    measCnt++;
  //
  if (n == measMaxCnt)
    // is too far away or too old
    return NULL;
  else
    return m1;
}

/////////////////////////////////////////////////////////////////

double UV360Sect::getRange()
{
  double result = 0.0;
  if (measCnt > 0)
    result = meas[0].getRange();
  return result;
}

