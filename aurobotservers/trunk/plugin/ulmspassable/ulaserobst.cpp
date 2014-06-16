/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ulaserobst.h"

#include "ulaserpi.h"
#include "ulaserscan.h"

ULaserObst::ULaserObst()
{
  leftIdx = 0;
  rightIdx = 0;
  lineFitVariance = 0.0;
}

/////////////////////////////////////////////////

ULaserObst::~ULaserObst()
{
}

/////////////////////////////////////////////////

bool ULaserObst::isValid()
{
  if (scan != NULL)
    return (leftIdx > 0) and (leftIdx < scan->getDataMax());
  else
    return false;
}

/////////////////////////////////////////////////
  
int ULaserObst::getIndexMax()
{
  if (scan != NULL)
    return scan->getDataMax();
  else
    return 0;
}

/////////////////////////////////////////////////

bool ULaserObst::findEdgeObstacle(int refIdx, bool toTheLeft,
                                  ULaserScan * scanRef, ULaserPi * neighbourPi)
{ // NB! all is in robot coordinates
  bool result = false;
  int step;
  int obstStart;
  int obstEnd = 0;
  int i;
  UPosition p1, p2;
  double d1, d2; // distance to robot
  const double DIST_OBST_LIMIT = 0.30; // meter
  const double DIST_OBST_END_LIMIT = 0.20; // meter
  const double DIST_OBST_LIMIT_IN_X = 0.20; // meter
  ULaserPoint * pp;
  int nIdx = -1; // neighbour index
  bool isPi = false;
  //
  leftIdx = 0;
  rightIdx = 0;
  scan = scanRef;
  obstStart = refIdx;
  if (scan != NULL)
  {
    if (toTheLeft)
    {
      step = 1;
      if (neighbourPi != NULL)
        nIdx = neighbourPi->getRight();
    }
    else
    {
      step = -1;
      if (neighbourPi != NULL)
        nIdx = neighbourPi->getLeft();
    }
    // find if there is an obstacle
    pp = &scan->data[obstStart];
    p1 = pp->pos;
    result = absi(nIdx - refIdx) > 1;
    if (not result)
    { // may be treated as partially passable - if needed
      closePi = neighbourPi;
      closePiDist = p1.dist(scan->getPos(nIdx));
      if (((closePiDist > DIST_OBST_LIMIT) and
            (p1.x > scan->getPos(nIdx).x)) or
            (p1.x - closePi->getLeftPos().x > DIST_OBST_LIMIT_IN_X) or
            (p1.x - closePi->getRightPos().x > DIST_OBST_LIMIT_IN_X))
      { // must be treated as an obstacle
        obstStart = nIdx;
        if (toTheLeft)
          obstEnd = neighbourPi->getLeft();
        else
          obstEnd = neighbourPi->getRight();
        isPi = true;
        result = true;
      }
    }
    else
    {
      closePi = NULL;
      closePiDist = 1.0;
    }
  }
  if (result and not isPi)
  { // look for obstacle (if not found already as another PI)
    result = false;
    d1 = p1.dist();
    for (i = 0; i < 20; i++)
    { // advance to next beam
      obstStart += step;
      if ((obstStart >= (scan->dataCnt - 1)) or (obstStart <= 0))
        // no more data
        break;
      pp = &scan->data[obstStart];
      if ((pp->passQ == PQ_EASY) or (pp->passQ == PQ_ROUGH))
        // already part of object
        break;
      if (pp->isValid())
      { // ignore dazzeled values (range == 0.0)
        p2 = pp->pos;
        d2 = p2.dist();
        // an obstacle is closer in x and closer to robot
        if (((p1.x - p2.x) > DIST_OBST_LIMIT_IN_X) and
              ((d1 - d2) > DIST_OBST_LIMIT))
        { // an obstacle is found
          result = true;
          break;
        }
      }
    }
  }
  // then look for obstacle end
  if (result and not isPi)
  { // an obstacle is found - find extend
    obstEnd = obstStart;
    for (;;)
    { // advance to next beam
      obstEnd += step;
      p1 = p2; // save reference position
      if ((obstEnd >= scan->dataCnt - 1) or (obstEnd <= 0))
        // no more data
        break;
      pp = &scan->data[obstEnd];
      if ((pp->passQ == PQ_EASY) or (pp->passQ == PQ_ROUGH))
      { // already part of object
        // printf("Q-break - part of other interval\n");
        obstEnd -= step;
        break;
      }
      // get next position
      p2 = pp->pos;
      d2 = p2.dist(p1);
      if (d2 > DIST_OBST_END_LIMIT)
      { // an obstacle end is found
        obstEnd -= step; // last is not to be included
        break;
      }
    }
  }
  // evaluate obstacle
  if (result)
  { // save left and right beam number
    if (toTheLeft)
    {
      leftIdx = obstEnd;
      rightIdx = obstStart;
    }
    else
    {
      leftIdx = obstStart;
      rightIdx = obstEnd;
    }
  }
  //
  // debug
  //printf("---obst: from %d left (%s), result %s: obst from %d to %d\n",
  //   refIdx, bool2str(toTheLeft), bool2str(result), rightIdx, leftIdx);
  // debug end
  //
  if (result)
  { // find a linefit for obstacle
    if (absi(obstEnd - obstStart) == 0)
    { // too small for line fit
      center = scan->getPos(obstStart);
      p1 = center.scaled(2.0); // pointing away from robot
      lineFit.setFromPoints(&center, &p1);
      lineFit.length = 0.05; // default length for one point
    }
    else
    {
      lineFit = scan->getLineSegmentFit(leftIdx, rightIdx, &lineFitVariance, &center);
      /** @todo her mangler nok bedre estimaering af obstacle type wall, human, ...*/
    }
  }
  return result;
}

/////////////////////////////////////////////////

void ULaserObst::moveToMap(UPose odoPose)
{
  UPosition p1;
  UPosition p2;
  // move approximated line
  p1 = odoPose.getPoseToMap(lineFit.pos);
  p2 = odoPose.getPoseToMap(lineFit.getOtherEnd());
  lineFit.setFromPoints(p1, p2);
  // move center position
  center = odoPose.getPoseToMap(center);
}

//////////////////////////////////////////////

ULaserPi * ULaserObst::getClosebyPi()
{
  return closePi;
}


