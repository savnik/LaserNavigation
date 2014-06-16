/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                        *
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
#include "uvisdata.h"

UVisData::UVisData()
{
  poly = &polyData;
  polyLocalCoordinates = true;
  polyNew = false;
  //visRngs = NULL;
}

//////////////////////////////////////////

UVisData::~UVisData()
{
  //if (visRngs != NULL)
  //  delete visRngs;
}

//////////////////////////////////////////

//   bool UVisData::makeRangeIntervals(double startRange, UPoseTime * robPose, UPose * destPose)
//   {
//     bool result = false;
//     UPose dp;
//     //
//     if (visRngs == NULL)
//       visRngs = new UVisRangeSet();
//     if (visRngs != NULL)
//     {
//       visRngs->lock();
//       if (destPose != NULL)
//       {
//         dp = robPose->getMapToPosePose(destPose);
//         result = visRngs->makeRangeIntervals(poly, startRange, &dp, robPose);
//       }
//       else
//         result = visRngs->makeRangeIntervals(poly, startRange, NULL, robPose);
//       result = visRngs->moveToMap(robPose);
//       visRngs->unlock();
//     }
//     return result;
//   }
