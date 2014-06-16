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

#include "uavoidparams.h"

UAvoidParams::UAvoidParams()
{
  obstMinDist = 0.5; // used by old method only
  obstMinMinDist = 0.4; // used by old method only
  doCrashTest = false;
  maxNestedLevels = 1;
  maxAvoidLoops = 8;
  maxSpawnCnt = 6;
  frontLeft.set(0.4,0.3,0.0);
  frontRight.set(0.4,-0.3,0.0);
  maxAcceleration = 0.5;
  maxTurnAcceleration = 0.3;
  minTurnRadius = 0.3;
  obsts = NULL;
  roads = NULL;
  logAvoid = NULL;
  avoidSerial = -1;
  obstClearanceDesired = 0.2; // meter
  obstClearanceMinimum = 0.03; // meter
  useDriveon = true;
  driveonGA = 2.0;
  driveonGD = 0.75;
  maxTangentDepth = 10;
  followLineLastPose = true;
  cellRev2 = false;
  startPose.clear();
  exitPose.clear();
}

//////////////////////////////////////////////////////////////

double UAvoidParams::getSafeDistance(bool avoidLeft, bool inner,
                                     bool tight, bool linear)
{
  double d, cy, rp, tr;
  //
  // turn radius
  tr = getMinTurnRad();
  // linear distance towards obstacle
  if ((avoidLeft and inner) or (not avoidLeft and not inner))
    d =  -frontRight.y;
  else
    d =  frontLeft.y;
  // get radius of most exposed corner of robot
  if (not linear and not inner)
  { // turning is relevant in outher distance only
    if (avoidLeft)
    { // keep left of an obstacle, i.e. a right turn
      // get y-value of minimum turn circle
      cy = -tr;
      // get radius of most extreme point on robot
      rp = hypot(frontLeft.y - cy, frontLeft.x);
    }
    else
    { // turning left, so the right-fromt corner is in danger
      cy = tr;
      // get radius of most extreme point on robot
      rp = hypot(frontRight.y - cy, frontRight.x);
    }
    d = rp - tr;
  }
  // add clearence margin
  if (tight)
  { // minimum clearence
    d += obstClearanceMinimum;
  }
  else
  { // normal clearence
    d += obstClearanceDesired;
  }
  // return result
  return d;
}

////////////////////////////////////////////////////////////

double UAvoidParams::getDiagonal(bool avoidLeft, bool tight, double * diagAngle)
{
  double d1, d2; //short sides of triangle
  //
  if (tight)
    // minimum clearence
    d1 = obstClearanceMinimum;
  else
    // normal clearence
    d1 = obstClearanceDesired;
  // plus width of vehicle
  d1 += frontLeft.y - frontRight.y;
  // forward distance
  if (avoidLeft)
    d2 =  frontLeft.x;
  else
    d2 =  frontRight.x;
  if (diagAngle != NULL)
    *diagAngle = atan(d1/d2);
  // return result
  return hypot(d1, d2);
}

////////////////////////////////////////////////////////////

double UAvoidParams::getDistToCorner(bool left, bool justMax)
{
  double d1, d2; //short sides of triangle
  //
  // plus width of vehicle
  if (left or justMax)
    d1 = hypot(frontLeft.y, frontLeft.x);
  else
    d1 = 0.0;
  if (not left or justMax)
    d2 = hypot(frontRight.y, frontRight.x);
  else
    d2 = 0.0;
  return fmax(d1, d2);
}

////////////////////////////////////////////////////////////

double UAvoidParams::getWallWallRadius(bool avoidLeft, double turnRadius)
{
  double d1, d2; //short sides of triangle
  //
  if (avoidLeft)
  {
    d2 =  frontLeft.x;
    d1 =  frontLeft.y + turnRadius;
  }
  else
  {
    d2 =  frontRight.x;
    d1 = -frontRight.y + turnRadius;
  }
  // return result
  return hypot(d1, d2);
}

/////////////////////////////////////////////////////////////

double UAvoidParams::getMinTurnRad()
{
  double result;
  //
  if (useDriveon)
    // use the driveon minimum turn radius
    result = driveonGA * M_PI / (driveonGD * 2.0);
  else
    result = minTurnRadius;
  //
  return result;
}

/////////////////////////////////////////////////////////////

double UAvoidParams::getMinOpening()
{
  double result;
  //
  result = frontLeft.y - frontRight.y + 2.0 * obstClearanceMinimum;
  //
  return result;
}

bool UAvoidParams::withinRobotOutline(UPose robot, UPosition pos, double margin)
{
  bool result = false;
  UPosition p;
  p = robot.getMapToPose(pos);
  if (p.y < frontLeft.y + margin and
      p.y > frontRight.y - margin and
      p.x > -margin)
  { // is inside envelope - or in front
    if (p.y > 0.0)
      result = (p.x < frontLeft.x + margin);
    else
      result = (p.x < frontRight.x + margin);
  }
  return result;
}

