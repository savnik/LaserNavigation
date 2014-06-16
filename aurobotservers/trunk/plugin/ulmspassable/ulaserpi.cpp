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
#include "ulaserpi.h"

#include "ulaserscan.h"
#include "urespassable.h"

ULaserPi::ULaserPi()
{
  scan = NULL;
  left = 0;
  right = 0;
  yw = -1.0;
}

/////////////////////////////////////////////////////////

ULaserPi::~ULaserPi()
{
}

/////////////////////////////////////////////////////////

void ULaserPi::setInterval(int rightMostPoint,
                           int leftMostPoint,
                           ULaserScan * sourceScan,
                           double varMin,
                           double varMin2)
{
  UPosition p1, p2;
  //
  left = leftMostPoint;
  right = rightMostPoint;
  scan = sourceScan;
  varianceMin = varMin;
  varianceMin2 = varMin2;
  // set also line segment
  if (true)
  { // set line segment
    segment = scan->getLineSegmentFit(left, right, &variance, NULL);
    p1 = segment.pos;
    p2 = segment.getOtherEnd();
    // tilt rotation around x-axis
    tilt = atan2(p2.z - p1.z, p2.y - p1.y);
    yw = p2.y - p1.y;
  }
}

/////////////////////////////////////////////////////////

void ULaserPi::setCenter(double segmentCenter, bool valid)
{
  UPosition pos;
  //
//  center = centerIdx;
//  pos = scan->getPos(center);
  centerPosition = segmentCenter;
  centerValid = valid;
}

/////////////////////////////////////////////////////////

UPosition ULaserPi::getLeftPos()
{
  return scan->getPos(left);
}

/////////////////////////////////////////////////////////

UPosition ULaserPi::getRightPos()
{
  return scan->getPos(right);
}

/////////////////////////////////////////////////////////

UPosition ULaserPi::getCenterPos()
{
  return segment.getPositionOnLine(centerPosition);
}

/////////////////////////////////////////////////////////

UPosition ULaserPi::getTop()
{
  return segment.getPositionOnLine(centerPosition);
}


/////////////////////////////////////////////////////////

void ULaserPi::moveToMap(UPose * odoPose)
{ // move segment to line
  UPosition p1;
  UPosition p2;
  //
  p1 = odoPose->getPoseToMap(segment.pos);
  p2 = odoPose->getPoseToMap(segment.getOtherEnd());
  segment.setFromPoints(p1, p2);
  // move also obstacles
  if (obstLeft.isValid())
    obstLeft.moveToMap(odoPose);
  if (obstRight.isValid())
    obstRight.moveToMap(odoPose);
}

/////////////////////////////////////////////////////////

int ULaserPi::findEdgeObstacles(FILE * logo, UResPassable * resp)
{ // find potential obstacles left and right
  int result = 0;
  int i;
  ULaserPi * pi;
  UPosition pnt;
  UPose pose;
  //
  pi = resp->findPi(left + 1);
  if (obstLeft.findEdgeObstacle(left, true, scan, pi))
  {
    result++;
    //
    // debug print
    //printf("Setting PQ_OBST to the left %d to %d\n",
    //          obstLeft.getRightIndex(), obstLeft.getLeftIndex());
    // debug print end
    //
    for (i = obstLeft.getRightIndex(); i <= obstLeft.getLeftIndex(); i++)
    {
      if (scan->getQ(i) < PQ_ROUGH)
        // is passable interval, do not reclassify
        break;
      scan->setQ(i, PQ_OBST);
    }
/*    if (logo != NULL)
    {
      pose = scan->odoPose;
      for (i = obstLeft.getRightIndex(); i <= obstLeft.getLeftIndex(); i++)
      {
        pnt = scan->getPos(i);
        pnt = pose.getPoseToMap(pnt);
        fprintf(logo, "%lu %03d 0 %8.2f %8.2f %8.2f\n", scan->getSerial(),
                i, pnt.x, pnt.y, pnt.x);
      }
    }*/
  }
  pi = resp->findPi(right - 1);
  if (obstRight.findEdgeObstacle(right, false, scan, pi))
  {
    result++;
    //
    // debug print
    //printf("Setting PQ_OBST to the right %d to %d\n",
    //     obstRight.getRightIndex(), obstRight.getLeftIndex());
    // debug print end
    //
    for (i = obstRight.getLeftIndex(); i >= obstRight.getRightIndex(); i--)
    {
      if (scan->getQ(i) < PQ_ROUGH)
        // is passable interval, do not reclassify
        break;
      scan->setQ(i, PQ_OBST);
    }
/*    if (logo != NULL)
    {
      pose = scan->odoPose;
      for (i = obstLeft.getRightIndex(); i <= obstLeft.getLeftIndex(); i++)
      {
        pnt = scan->getPos(i);
        pnt = pose.getPoseToMap(pnt);
        fprintf(logo, "%lu %03d 1 %8.2f %8.2f %8.2f\n", scan->getSerial(),
                i, pnt.x, pnt.y, pnt.x);
      }
    }*/
  }
  return result;
}

/////////////////////////////////////////////////////////

void ULaserPi::print(const char * preString)
{
  const int MSL = 300;
  char s[MSL];
  //
  print(preString, s, MSL);
  printf("%s", s);
}

/////////////////////////////////////////////////////////

void ULaserPi::print(const char * preString, char * buff, const int buffCnt)
{
  //const int MSL = 30;
  //char st[MSL];
  UPosition pR, pL;
  //
  //scan->time.getTimeAsString(st, true);
  //scan->odoPose.snprint(s, "pose", MSL);
  pL = getLeftPos();
  pR = getRightPos();
  snprintf(buff, buffCnt, "%sscan %4lu - (%.2fx, %.2fy) to (%.2fx, %.2fy)\n",
         preString, scan->getSerial(), pR.x, pR.y, pL.x, pL.y);
}

/////////////////////////////////////////////////////////

double ULaserPi::getTopHeight(double laserTilt)
{
  double result;
  double d, d1, d2;
  UPosition pr, pc;
  // laser tilt is a negative angle in radians
  //pr.set(scan->odoPose.x, scan->odoPose.y, 0.0);
  // measured distance
  //d1 = pr.dist(getCenterPos());
  d1 = getCenterPos().dist();
  // fitted line
  //d2 = pr.dist(getTop());
  d2 = getTop().dist();
  // distance relative to fittet line
  d = d1 - d2;
  result = d * sin(laserTilt);
  //
  return result;
}

/////////////////////////////////////////////////////////

U2Dline ULaserPi::get2DLine()
{
  U2Dline result;
  result.setPV(segment.pos.x, segment.pos.y,
               segment.vec.x, segment.vec.y);
  return result;
}

//////////////////////////////////////////////////////////

double ULaserPi::get2DDistToCross(U2Dline line,
                                  UPose fromPose,
                                  bool * behind)
{
  double result = 0.0;
  float x,y;
  U2Dline l2d;
  UPosition p1, p2;
  //
  l2d = get2DLine();
  if (l2d.getCrossing(line, &x, &y))
  {
    p1.set(x, y, 0.0);
    p2 = fromPose.getMapToPose(p1);
    result = hypot(p2.y, p2.x);
  }
  if (behind != NULL)
    *behind = (p2.x < 0.0);
  return result;
}

/////////////////////////////////////////////////////////

UTime ULaserPi::getDetectTime()
{
  UTime t;
  if (scan != NULL)
    t = scan->time;
  return t;
}

