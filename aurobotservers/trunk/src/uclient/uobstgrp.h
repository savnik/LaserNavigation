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
#ifndef UOBSTGRP_H
#define UOBSTGRP_H

#include <urob4/uobstacle.h>
#include <urob4/uclientfuncbase.h>

/**
Group of obstacles (just an array, not the full story as used in mmrd)

@author Christian Andersen
*/
class UObstHist : public UOnEvent
{
public:
  /**
  constructor */
  UObstHist();
  /**
  destructor */
  ~UObstHist();
  /**
  Get count of obst groups in hist */
  inline int getGrpsCnt()
  { return grpsCnt; };
  /**
  Get one group */
  inline UObstacleGroup * getGroup(int idx)
  { return grps[idx]; };
  /**
  Get actual group count */
  inline void setGroupsCnt(int value)
  { grpsCnt = value; };
  /**
  Get maximum count of group buffers */
  inline int getGrpsMaxCnt()
  { return MAX_OBST_GRPS; };
  /**
  Get next obstacle group, and create 
  if not created already */
  UObstacleGroup * getNewGrp();
  
public:
    static const int MAX_OBST_GRPS = 200;
protected:
  /**
  Group of obstacles, i.e. set of polygons */
  UObstacleGroup * grps[MAX_OBST_GRPS];
  /**
  Number of groups loaded into obstGrps array */
  int grpsCnt;

};

#endif
