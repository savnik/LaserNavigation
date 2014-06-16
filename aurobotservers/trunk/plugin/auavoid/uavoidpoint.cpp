/** *************************************************************************
*   Copyright (C) 2010 by DTU (Christian Andersen)
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
*   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
*   Boston, MA 02110-1301, USA.
***************************************************************************/

#include "uavoidpoint.h"

#include "uavoidobst.h"

UAvoidPoint::UAvoidPoint()
{
  clear();
  // clear also ooobstacle pointers and flags
  aob = NULL;
  avoidLeft = false;
  followLineLastPose = true;
  aPos.clear();
  serial = 0;
  generation = 0;
}

/////////////////////////////////////////

UAvoidPoint::~UAvoidPoint()
{
//  printf("UAvoidPoint - destructor\n");
}

/////////////////////////////////////////

void UAvoidPoint::clear()
{
  midCnt = 0;
  next = NULL;
  prev = NULL;
  oob = NULL;
  prePoint = false;
  useTight = false;
  mCent.clear();
  mid.clear();
  oPos.clear();
  tPos.clear();
}

///////////////////////////////////////////

UAvoidPoint * UAvoidPoint::insertAfter(UAvoidPoint * newNext)
{
  // set links
  newNext->prev = this;
  newNext->next = next;
  if (next != NULL)
    next->prev = newNext;
  next = newNext;
  // set angle to new point
  setAngNext();
  if (next->next != NULL)
    next->setAngNext();
  return next;
}

///////////////////////////////////////////

UAvoidPoint *  UAvoidPoint::unlinkNext()
{
  UAvoidPoint * result = next;
  //
  if (next != NULL)
  {
    next = next->next;
    if (next != NULL)
    {
      next->prev = this;
      setAngNext();
    }
  }
  result->next = NULL;
  result->prev = NULL;
  return result;
}

///////////////////////////////////////////

void UAvoidPoint::setAvoidObst(UAvoidObst * ob, int idx , bool cvLeft)
{
  aob = ob;
  aPos = aob->obst->getPoint(idx);
  avoidLeft = cvLeft;
}

////////////////////////////////////////////

void UAvoidPoint::setAngNext()
{
  if (next != NULL)
    angNext = atan2(next->aPos.y - aPos.y, next->aPos.x - aPos.x);
  else
  {
    printf("UAvoidPoint::setAngNext has no next!\n");
    angNext = 0.0;
  }
}

////////////////////////////////////////////////////

bool UAvoidPoint::tPointVisible(bool * knifeOpening, bool * opposingFirst)
{
  double dta;
  bool result = true;
  bool samePoint;
  U2Dseg seg;
  double t, d;
  // the tPos is tPos marks the narrowest point opposing on the a (this) obstacle
  // The tPos may be away from the aPos - find distance
  dta = hypot(aPos.y - tPos.y, aPos.x - tPos.x);
  // now, the line from past pose, may or may not
  // require an additional waypoint before this mid-point.
  // If we appraoch the obstacle from behind (aPos is earlier than the tPos) then a pre-point may be needed, if we appraoch the obstacle facing the tPos (tPos is before aPos) then no pre-point is needed.
  samePoint = (dta < 0.01);
  seg.setFromPoints(prev->aPos.x, prev->aPos.y, aPos.x, aPos.y);
  if (not samePoint)
  { // get line segment from last position
    t = seg.getPositionOnLine(tPos.x, tPos.y);
    if (t > seg.length)
      // we are approaching from behind - tPos is hidden
      result = false;
    // else
    // tPos is visible from previous point
  }
  if (opposingFirst != NULL)
  {
    d = seg.distanceSigned(oPos.x, oPos.y);
    t = seg.getPositionOnLine(oPos.x, oPos.y);
    if (avoidLeft)
      *opposingFirst = (t < seg.length and d > -0.0001);
    else
      *opposingFirst = (t < seg.length and d < 0.0001);
    // debug
    printf("UAvoidPoint::tPointVisible:opposingFirst (%s): visseg %.3fx,%.3fy to %.3fx,%.3fy d=%g t=%g\n",
           bool2str(*opposingFirst), seg.getFirstEnd().x, seg.getFirstEnd().y, seg.getOtherEnd().x, seg.getOtherEnd().y, d, t);
    // debug end
  }
  if (knifeOpening != NULL)
    *knifeOpening = (samePoint and oVertex) or (not samePoint and tVertex);
  // else
  // tPoint and aPoint is the same - so counted as visible
  return result;
}

/////////////////////////////////////////

void UAvoidPoint::setTurnCentreFromMidPose(double turnCentreRadius, bool cvLeft)
{
  if (cvLeft)
    mCent = mid.getPoseToMapPose(0.0, -turnCentreRadius, M_PI / 2.0);
  else
    mCent = mid.getPoseToMapPose(0.0, turnCentreRadius, -M_PI / 2.0);
}

/////////////////////////////////////////

void UAvoidPoint::setTurnCentreFromMidPose(double turnCentreRadius)
{
  if (avoidLeft)
    mCent = mid.getPoseToMapPose(0.0, -turnCentreRadius, M_PI / 2.0);
  else
    mCent = mid.getPoseToMapPose(0.0, turnCentreRadius, -M_PI / 2.0);
}
