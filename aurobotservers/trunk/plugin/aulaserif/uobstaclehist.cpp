/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <stdio.h>
#include <math.h>

#include "uobstaclehist.h"

UObstacleHist::UObstacleHist()
{
  groupsCnt = 0;
  groupsNewest = 0;
}


///////////////////////////////////////////

UObstacleHist::~UObstacleHist()
{
}

///////////////////////////////////////////

UObstacleGroup * UObstacleHist::getGroup(unsigned long serial, bool mayCreate)
{
  UObstacleGroup * og;
  bool found = false;
  int i;
  //
  if (serial == 0)
    og = &fixeds;
  else
  {
    og = groups;
    for (i = 0; i < groupsCnt; i++)
    {
      found = og->getSerial() == serial;
      if (found)
        break;
      og++;
    }
    if (not found)
    { // no souch group - create as the newest
      if (mayCreate)
      {
        if (groupsCnt < getGroupsMax())
        { // just take the next
          og = &groups[groupsCnt];
          groupsNewest = groupsCnt++;
        }
        else
        { // reuse the oldest
          groupsNewest++;
          if (groupsNewest >= getGroupsMax())
            groupsNewest = 0;
          og = &groups[groupsNewest];
          if (not og->tryLock())
          { // is locked, this is not so good -- assume user has crashed
            // obstacle groups must be in time order, so we must unlock.
            printf("UObstacleGroup: ***** obstacle group %d found locked - but wery old?! - releases.\n", groupsNewest);
          }
          og->unlock();
          og->clear();
        }
        og->setSerial(serial);
      }
      else
        og = NULL;
    }
  }
  return og;
}

//////////////////////////////////////

UObstacleGroup * UObstacleHist::getGroupNewest(int cnt)
{
  int i;
  UObstacleGroup * result = NULL;
  //
  i = groupsNewest - cnt;
  if (i < 0)
    i += groupsCnt;
  if (i >= 0 and i < groupsCnt)
    result = &groups[i];
  return result;
}

