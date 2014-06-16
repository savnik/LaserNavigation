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

#include "uimage.h"
#include "uimgstat.h"

////////////////////////////////////////////////////

UImgStat::UImgStat()
{
  valid = false;
  yHigh = 200;
  step = 4;
  avgY = 0;
  avgHighY = 0;
  avgLowY = 0;
  cntLow = 0;
  cntHigh = 0;
}

////////////////////////////////////////////////////

UImgStat::~UImgStat()
{
}

///////////////////////////////////////////////////////////

/*
void UImgStat::clear()
{
  valid = false;
}
*/

////////////////////////////////////////////////////

bool UImgStat::evaluateStatistics(UImage * rawImg, int iStep /*= 4*/)
{
  bool result;
  unsigned int r,c;
  unsigned int yHighSum = 0;
  unsigned int yHighCnt = 0;
  unsigned int yLowSum = 0;
  unsigned int yLowCnt = 0;
  unsigned char * py;
  //int yMax = 0;
  //
  //printf("UImgStat::evaluateStatistics start\n");
  valid = false;
  step = iStep;
  result = (rawImg != NULL);
  if (result)
    result = (rawImg->valid);
  //
  if (result)
  { // do not use forst few rows, as these may be influenced
    // by old gain settings
    maxY = 0;
    for (r = 20; r < rawImg->getHeight(); r += step)
    {
      py = rawImg->getYline(r);
      for (c = 1; c < rawImg->getWidth(); c += step)
      { // normal max
        if (*py > maxY)
          maxY = *py;
        // high-low stats
        if (*py > yHigh)
        {
          yHighSum += *py;
          yHighCnt++;
        }
        else
        {
          yLowSum += *py;
          yLowCnt++;
        }
        py += step;
      }
    }
    avgY = (yHighSum + yLowSum)/(yHighCnt + yLowCnt);
    //
    cntHigh = yHighCnt;
    cntLow = yLowCnt;
    if (yHighCnt > 0)
      avgHighY = yHighSum / yHighCnt;
    else
      avgHighY = 0;
    //
    if (yLowCnt > 0)
      avgLowY = yLowSum / yLowCnt;
    else
      avgLowY = 255;
    //
    valid = true;
  }
  else
    valid = false;
  //
  //printf("UImgStat::evaluateStatistics end %s\n", bool2str(result));
  return result;
}

///////////////////////////////////////////////////////////

