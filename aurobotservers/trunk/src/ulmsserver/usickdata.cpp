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

#include <pthread.h>

#include "usickdata.h"

////////////////////////////////////////////////

USickData::USickData()
{
  lockInit();
  valid = false;
}

////////////////////////////////////////////////

USickData::~USickData()
{
}

////////////////////////////////////////////////

bool USickData::getDataTo(ULaserData * dest) 
{
  bool result = valid;
  int i;
  int * md = dest->getRange(0);
  int * mf = dest->getFlags(0);
  unsigned char * ps = getFirstVal();
  //
  for (i = 0; i < getValueCount(); i++)
  {
    *md++ = (ps[1] & 0x1f) * 256 + ps[0];
    *mf++ = ps[1] >> 5;
    ps += 2;
  }
  dest->setRangeCnt(getValueCount());
  dest->setUnit(getUnit());
  dest->setScanTime(scanTime);
  dest->setValid(result);
  //valid = false;
  return result;
}

////////////////////////////////////////////////
