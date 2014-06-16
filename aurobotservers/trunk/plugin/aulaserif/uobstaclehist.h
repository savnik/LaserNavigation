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

#ifndef UOBSTACLEHIST_H
#define UOBSTACLEHIST_H

#include <cstdlib>

#include <urob4/uobstacle.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Holds the most recent obstacle history
Initially for reactive behaviour only.

@author Christian Andersen
*/
class UObstacleHist
{
public:
  /**
  Constructor */
  UObstacleHist();
  /**
  Destructor */
  virtual ~UObstacleHist();
  /**
  get number of obstacle groups */
  inline int getGroupsCnt()
  { return groupsCnt; };
  /**
  get number of obstacle groups */
  inline UObstacleGroup * getGroup(int idx)
  { return &groups[idx]; };
  /**
  get maximum number of storable groups */
  inline int getGroupsMax()
  { return MAX_OBST_GRPS; };
  /**
  get index of newest obstacle group */
  inline int getGroupNewest()
  { return groupsNewest; };
  /**
  Get pointer to the obstacle group with the fixed obstacles */
  inline UObstacleGroup * getGroupFixed()
  { return &fixeds; };
  /**
  get obstacle group from newest and back with this count*/
  UObstacleGroup * getGroupNewest(int cnt);
  /**
  get obstacle groups with this serial number */
  UObstacleGroup * getGroup(unsigned long serial, bool mayCreate);
  /**
  Empty the obstacle histort data */
  inline void clear()
  { groupsCnt = 0; };

public:
  /** max number of obstacle groups that can be stored in this class */
  static const int MAX_OBST_GRPS = 100;
  /**
  Lock to ensure integrity in obstacle history */
  ULock ogLock;
  
protected:
  /**
  array of pointert to the newest obstacle groups. */
  UObstacleGroup groups[MAX_OBST_GRPS];
  /**
  Obstacle group for fixed obstacles. */
  UObstacleGroup fixeds;
  /**
  Number of used groups */
  int groupsCnt;
  /**
  Newest group */
  int groupsNewest;
  /**
  Updated with this timestamp */
  UTime updTime;
};

#endif

