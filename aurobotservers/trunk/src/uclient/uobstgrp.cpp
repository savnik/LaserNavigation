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
#include "uobstgrp.h"

UObstHist::UObstHist()
{
  int i;
  //
  for (i = 0; i < MAX_OBST_GRPS; i++)
    grps[i] = NULL;
  grpsCnt = 0;
}

////////////////////////////////////

UObstHist::~UObstHist()
{
  int i;
  for (i = 0; i< MAX_OBST_GRPS; i++)
    if (grps[i] != NULL)
    {
      delete grps[i];
      grps[i] = NULL;
    }
  grpsCnt = 0;
}

/////////////////////////////////

UObstacleGroup * UObstHist::getNewGrp()
{
  UObstacleGroup * result = NULL;
  //
  if (grpsCnt < MAX_OBST_GRPS)
  {
    result = grps[grpsCnt];
    if (result == NULL)
    { // create new structure
      result = new UObstacleGroup();
      grps[grpsCnt] = result;
    }
    grpsCnt++;
  }
  return result;
}

