/***************************************************************************
 *   Copyright (C) 2007-2008 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
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
#include "uroadline.h"

URoadLine::URoadLine()
{
  clear(-1);
} 

////////////////////////////////////////

URoadLine::~URoadLine()
{
}

/////////////////////////////////////////
void URoadLine::clear(int asEdge)
{
  edge = asEdge;
  xR = 0.5; // initial estimate of edge estimation SD
  pR = sqr(0.5); // start estimate on state noice
  pL = sqr(0.5); // start estimate on line covariance
  scanSerial = 0;
  valid = false;
  updateCnt = 0;
  lineSerial = 0;
}

////////////////////////////////////////

double URoadLine::getDistanceSD(UPosition * pos)
{
  double result;
  result = line.getDistanceSq(pos);
  result /= xR;
  return sqrt(result);
}

////////////////////////////////////////

void URoadLine::update(UPosition pos, UPoseTime pose, unsigned long scan, bool moving, int piIdx)
{
  double d, t1, t3;
  UPosition p2, p3, p4, pOld;
  ULineSegment l2;
  double k;
  const double rR = sqr(0.25); // measurement noice estimate (on variance)
  const double qR = sqr(0.1); // state noice estimate (on variance)
  const double rL = sqr(0.5); // measurement noice estimate (on line)
  const double updateDistNotMoving = 0.05; // when not moving
  const int updateCntNotMoving = 7; //
  //
  scanSerial = scan;
  pisIdx = piIdx;
  // get distance to line
  t1 = line.getPositionOnLine(pos);
  p2 = line.getPositionOnLine(t1);
  l2.setFromPoints(p2, pos);
  d = l2.length;
  // do not update existing lines if not moving - all measurements are too correlated
  if (moving or (fabs(d) > updateDistNotMoving) or (updateCnt < updateCntNotMoving))
  { // update error estimate
    pR = pR + qR; // project covariance since last update
    k = pR/(pR + rR); // get a new gain
    xR = xR + k*(d - xR); // project forward
    pR = (1 - k) * pR; // update covariance
    // now update the line estimate
    pL = pL + sqr(xR);
    k = pL / (pL + rL);
    t3 = l2.length * k; // get part of line segment to be used for update
    pL = (1 - k) * pL; // update covariance
    p3 = l2.getPositionOnLine(t3);
    // now get a reference position on line to maintain
    // go distance to line back.
    t3 = pose.getDistance(p2);
    t3 = t1 - t3;
    p4 = line.getPositionOnLine(t3);
    // now set new line estimate
    pOld = line.pos;
    line.setFromPoints(p4, p3);
    // debug
    if (updateCnt < 3)
      t3 -= 0.01;
    // debug end
    if (t3 < 0)
    { // line segment is too short to use new root point (p4)
      line.pos = pOld;
      line.length = hypot(p3.x - pOld.x, p3.y - pOld.y);
    }
    // increase update cnt
    updateCnt++;
  }
  //
  odoPose = pose;
}

////////////////////////////////////////

void URoadLine::setNew(UPosition pos, UPoseTime pose, unsigned long scan, int piIdx, unsigned long roadSerial)
{
  scanSerial = scan;
  pisIdx = piIdx;
  updateCnt = 1;
  line.set2D(pos.x, pos.y, pose.h);
  line.length = 0.0;
  valid = true;
  xR = 1.0;
  pR = sqr(0.5);
  pL = sqr(1.0);
  odoPose = pose;
  lineSerial = roadSerial;
}

////////////////////////////////////////

void URoadLine::postUpdate(unsigned int scan, UPoseTime * pose, bool moving)
{
  double d;
//  double DISTANCE_DROP_LIMIT = 8.0; // m-- should not close side roads
  const int minConsecUpds = 3; // needs consecutive updates to survive
  const int minConsecUpdsNotMoving = 6; // needs consecutive updates to survive when not moving
  const int maxConsecMisses = 15;
  const double qRadd = 0.15; // estimated increase in SD on line estimate
  //
  if (scan != scanSerial)
  {
    if (updateCnt < minConsecUpds)
      // needs consecutive updates to survive
      valid = false;
    else if ((not moving) and (updateCnt < minConsecUpdsNotMoving))
      valid = false;
    else if (absi(scan - scanSerial) > mini(maxConsecMisses, updateCnt / 2))
      valid = false;
/*    else
    { // drop if last update distance is too far away
      d = odoPose.getDistance(line.pos);
      if (d > DISTANCE_DROP_LIMIT)
        valid = false;
    }*/
    if (valid)
    { // failed to get update - decrease quality
      d = qRadd * (pose->t - odoPose.t);
      if (d > qRadd or d < 0.0)
        d = qRadd;
      xR += d; // increase expected line variance
      // and update the line estimate variance
      pL = pL + sqr(xR);
      // set also pose
      odoPose = *pose;
    }
  }
}

