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

#include "uavoidobst.h"

#include "uavoidlink.h"


/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////

bool UAvoidLnkSeq::pathCrossing(ULineSegment seg)
{
  bool xing = false;
  ULineSegment seg2;
  UPosition x;
  UAvoidLnkSeq * ls;
  //
  ls = this;
  while (ls != NULL)
  {
    seg2 = ls->tangLine->getTangentLine(ls->tangIdx);
    if (ls->next == NULL)
      // shorten the last tangent line a bit to avoid crossing
      // in the destination point, as this is OK.
      seg2.length -= 3e-3;
    xing = seg.getSegmentCrossingXY(&seg2, &x);
    if (xing)
      break;
    ls = ls->next;
  }
  //
  return xing;
}

/////////////////////////////////////////

double UAvoidLnkSeq::getDistance(double entryAngle,
                                 double exitAngle,
                                 double * turnAngleSum)
{
  UAvoidLnkSeq * ls;
  double d, dist = 0.0;
  double angSum = 0.0;
  ULineSegment seg1; //, seg2;
  int i1, i2 = -1, v, m;
  bool cv = false;
  UPosition p1, p2;
  double a1, a2, a;
  //
  a1 = entryAngle;
  ls = this;
  while (ls != NULL)
  {
    //seg2 = seg1;
    // get exit tangent as line segment
    seg1 = ls->tangLine->getTangentLine(ls->tangIdx);
    // add also followed obstacle edges - if not first tangent
    if (ls != this)
    { // not first tangent line in sequence
      // get new exit index
      i1 = ls->tangLine->idx[ls->tangIdx];
      // start with entry vertex
      v = i2;
      // get entry position
      p1 = ls->tangLine->tob->obst->getPoint(v);
      m = ls->tangLine->tob->obst->getPointsCnt();
      while (v != i1)
      { // not same entry and exit
        p2 = p1;
        a2 = a1;
        if (cv)
        { // follow index CV (decreasing)
          v--;
          if (v < 0)
            v = m - 1;
        }
        else
        { // follow obstacle index (CCV) increasing
          v++; // get next point
          if (v >= m)
            v = 0;
        }
        p1 = ls->tangLine->tob->obst->getPoint(v);
        // get angle of next tangent
        a1 = atan2(p1.y - p2.y, p1.x - p2.x);
        // get turn angle
        a = limitToPi(a1 - a2);
        angSum += fabs(a);
        // get edge distance
        d = hypot(p1.y - p2.y, p1.x - p2.x);
        dist += d;
      }
    }
/*    else
      // and entry angle is needed
      a1 = atan2(seg1.vec.y, seg1.vec.x);*/
      // get exit angle
    a2 = a1;
    a1 = atan2(seg1.vec.y, seg1.vec.x);
    // get turn angle
    a = limitToPi(a1 - a2);
    // add to turn sum
    angSum += fabs(a);
    // add distance of exit tangent line
    dist += seg1.length;
    // get entry orientation to next obstacle
    cv = (ls->tangIdx % 2 == 0);
    // get entry index on next obstacle
    i2 = ls->tangLine->aobIdx[ls->tangIdx];
    //
    ls = ls->next;
  }
  // add also angle to exit pose
  a2 = a1;
  a1 = exitAngle;
    // get turn angle
  a = limitToPi(a1 - a2);
    // add to turn sum
  angSum += fabs(a);
  // return result
  if (turnAngleSum != NULL)
    *turnAngleSum = angSum;
  return dist;
}


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


void UAvoidLink::visibilityTest(UAvoidLink * other)
{
  UPosition a1, a2, b1, b2, t1, t2;
  U2Dline la, lb, lt;
  float x, y;
  bool isOK, isX;
  int i;
  bool missingTangentCV;
  bool missingTangentCCV;
  bool sourceObstIsPoint;
  double /*d = 0.0,*/ a;
  bool lax = -1, lbx = -1, ltx;
  //
  sourceObstIsPoint = (tob->obst->getPointsCnt() == 1);
  if (other->aob->obst->getPointsCnt() > 1)
  { // if other obstacle has one point only, then
    // it is not obstructive
    //
    missingTangentCV = false;
    missingTangentCCV = false;
    // get line for tangents touching this obstacle clockwise
    // index 0 and 1 seen
    if (valid[2] or valid[3])
    { // get width segment of other obstacle seen from
      // this obstacle touching the other clockwise (index 2 of if invalid use 0)
      if (other->valid[2])
        a1 = other->aob->obst->getPoint(other->aobIdx[2]);
      else if (other->valid[0] and sourceObstIsPoint)
        a1 = other->aob->obst->getPoint(other->aobIdx[0]);
      else
        missingTangentCV = true;
      // get counter clockwise tangent (index 3 or if invalid 1)
      if (other->valid[3]) // either index 1 or 2 must be valid
        a2 = other->aob->obst->getPoint(other->aobIdx[3]);
      else if (other->valid[1] and sourceObstIsPoint)
        a2 = other->aob->obst->getPoint(other->aobIdx[1]);
      else
        missingTangentCCV = true;
      //
      if (missingTangentCV and missingTangentCCV)
      {
        //None of the tangents are valid - removed already, so
        // it should not be in the way for these tangent - or
        // a closer obstacle is more in the way.
      }
      else if (missingTangentCV or missingTangentCCV)
      { // One of the tangents are missing - get the point most distant
        // to the one we have got - this should be (almost) safe
        // b1 is the root point on this obstacle - used for heading only
        if (valid[2])
          t1 = tob->obst->getPoint(idx[2]);
        else if (valid[3])
          t1 = tob->obst->getPoint(idx[3]);
        if (missingTangentCV)
        { // getMostDistantVertexXY(double x, double y, double h,
          // int side (1=left, 0=right), UPosition * posOnPolygon)
          a = atan2(a2.y - t1.y, a2.x - t1.x);
          other->aob->obst->getMostDistantVertexXY(a2.x, a2.y, a, 1, &a1);
          if (fabs(a1.x - a2.x) + fabs(a1.y - a2.y) < 0.01)
          { // same point - a situation where we look at the wrong side
            other->aob->obst->getMostDistantVertexXY(a2.x, a2.y, a, 0, &a1);
            // printf("wrong side a2-t1\n");
          }
        }
        else
        { // missing CCV point
          a = atan2(a1.y - t1.y, a1.x - t1.x);
          other->aob->obst->getMostDistantVertexXY(a1.x, a1.y, a, 0, &a2);
          if (fabs(a1.x - a2.x) + fabs(a1.y - a2.y) < 0.01)
          { // same point - a situation where we look at the wrong side
            other->aob->obst->getMostDistantVertexXY(a1.x, a1.y, a, 1, &a2);
            // printf("wrong side a1-t1\n");
          }
        }
      }
      // make a line spanning the other obstacle to see if it blocks the tangent
      la.set2P(a1.x, a1.y, a2.x, a2.y);
      lax = fabs(a1.x - a2.x) > fabs(a1.y - a2.y);
    }
    // now get a line as seen from the tangens touching this CCV
    missingTangentCV = false;
    missingTangentCCV = false;
    //
    if (valid[0] or valid[1])
    { // get width segment of other obstacle seen from
      // counter clockvise tangent
      if (other->valid[0])
        b1 = other->aob->obst->getPoint(other->aobIdx[0]);
      else if (other->valid[2] and sourceObstIsPoint)
        b1 = other->aob->obst->getPoint(other->aobIdx[2]);
      else
        missingTangentCV = true;
      //
      if (other->valid[1])
        b2 = other->aob->obst->getPoint(other->aobIdx[1]);
      else if (other->valid[3] and sourceObstIsPoint)
        b2 = other->aob->obst->getPoint(other->aobIdx[3]);
      else
        missingTangentCCV = true;
      if (missingTangentCV and missingTangentCCV)
      { //None of the tangents are valid - removed already
      } // no action needed
      else if (missingTangentCV or missingTangentCCV)
      { // One of the tangents is missing - get the point most distant
      // to the one we have got - this should be (almost) safe
      // t1 is the root point on this obstacle - used for heading only
        if (valid[0])
          t1 = tob->obst->getPoint(idx[0]);
        else if (valid[1])
          t1 = tob->obst->getPoint(idx[1]);
        if (missingTangentCV)
        { // getMostDistantVertexXY(double x, double y, double h,
        // int side (1=left, 0=right), UPosition * posOnPolygon)
          a = atan2(b2.y - t1.y, b2.x - t1.x);
          other->aob->obst->getMostDistantVertexXY(b2.x, b2.y, a, 1, &b1);
          if (fabs(b1.x - b2.x) + fabs(b1.y - b2.y) < 0.01)
          { // same point - a situation where we look at the wrong side
            other->aob->obst->getMostDistantVertexXY(b2.x, b2.y, a, 0, &b1);
            // printf("wrong side b2-t1\n");
          }
        }
        else
        { // missing CCV point
          a = atan2(b1.y - t1.y, b1.x - t1.x);
          other->aob->obst->getMostDistantVertexXY(b1.x, b1.y, a, 0, &b2);
          if (fabs(b1.x - b2.x) + fabs(b1.y - b2.y) < 0.01)
          { // same point - a situation where we look at the wrong side
            other->aob->obst->getMostDistantVertexXY(b1.x, b1.y, a, 1, &b2);
            // printf("wrong side b1-t1\n");
          }
        }
      }
      lb.set2P(b1.x, b1.y, b2.x, b2.y);
      lbx = fabs(b1.x - b2.x) > fabs(b1.y - b2.y);
    }
    //
    for (i = 0; i < tangentCnt; i++)
    {
      if (valid[i])
      {
        t1 = aob->obst->getPoint(aobIdx[i]);
        t2 = tob->obst->getPoint(idx[i]);
        lt.set2P(t1.x, t1.y, t2.x, t2.y);
        ltx = fabs(t1.x - t2.x) > fabs(t1.y - t2.y);
        if (i > 1)
        { // tangent 2 and 3 should be tested against the a1-a2 diagonal
          isOK = lt.getCrossing(la, &x, &y);
          if (lax)
            isX = isOK and isWithin(x, a1.x, a2.x);
          else
            isX = isOK and isWithin(y, a1.y, a2.y);
        }
        else
        { // tangent 0 and 1 should be tested against the b1-b2 diagonal
          isOK = lt.getCrossing(lb, &x, &y);
          if (lbx)
            isX = isOK and isWithin(x, b1.x, b2.x);
          else
            isX = isOK and isWithin(y, b1.y, b2.y);
        }
        if (isX)
        { // the line (lt) is crossing the other obstacle, but is it on the
          // way to this obstacle
          if (ltx)
            isX = isOK and isWithin(x, t1.x, t2.x);
          else
            isX = isOK and isWithin(y, t1.y, t2.y);
        }
        if (isX)
        {
          valid[i] = false;
          if (mirror != NULL)
          { // remove from mirrir links too
            if (i == 0 or i == 3)
              mirror->valid[i] = false;
            else if (i == 1)
              mirror->valid[2] = false;
            else
              mirror->valid[1] = false;
          }
        }
      }
    }
  }
}

/////////////////////////////////////////

UPosition UAvoidLink::getOtherEnd(int k)
{
  UPosition result;
  //
  if ((k >= 0) and (k < 4) and valid[k])
    result = aob->obst->getPoint(aobIdx[k]);
  //
  return result;
}

/////////////////////////////////////////

int UAvoidLink::getOtherEndIdx(int k)
{
  int result;
  //
  result = k;
  if (k == 1)
    result = 2;
  else if (k == 2)
    result = 1;
  //
  return result;
}

/////////////////////////////////////////

UPosition UAvoidLink::getThisEnd(int k)
{
  UPosition result;
  //
  if ((k >= 0) and (k < 4) and valid[k])
    result = tob->obst->getPoint(idx[k]);
  //
  return result;
}

/////////////////////////////////////////

ULineSegment UAvoidLink::getTangentLine(int idx)
{
  UPosition p1, p2;
  ULineSegment result;
  //
  if ((idx >= 0) and (idx < 4) and valid[idx])
  {
    p1 = getThisEnd(idx);
    p1.z = 0.0;
    p2 = getOtherEnd(idx);
    p2.z = 0.0;
    result.setFromPoints(p1, p2);
  }
  else
    result.length = -1.0;
  //
  return result;
}

/////////////////////////////////////////

void UAvoidLink::setBadEdgeAndVertex(double margin)
{
  double d;
  int i1, i2, m;
  UPosition p1;
  bool otherInsideThis;
  bool lastIn = false;
  //
  m = tob->obst->getPointsCnt();
  if (valid[1] and valid[2])
  { // outher tangents are valid, this is required for valid
    // nogo stttings
    // test if other obstacle is predominantly inside this
    // the test is not floorless, as COG may just happen to fall inside this
    // obstacle, in wich case some nogo lines may be falsely allowed.
    p1 = aob->obst->getCogXY();
    d = tob->obst->getDistance(p1.x, p1.y);
    otherInsideThis = d < 0.0;
    i2 = idx[1];
    i1 = (i2 + 1) % m;
    while (i1 != idx[2])
    { // start CCV
      p1 = tob->obst->getPoint(i1);
      d = aob->obst->getDistance(p1.x, p1.y);
      if (otherInsideThis)
      { // special rule apply
        if (d < 0.0)
        { // if outside - it is most likely that
          // it penetrades the other obstacle, and thus
          // is available for tangents, but inside the other is not.
          tob->nogoEdge[i2] = true;
          if (lastIn)
            tob->nogoVertex[i2] = true;
          lastIn = true;
        }
        else if (lastIn)
        { // last was inside so edge is nogo-for tangents
          tob->nogoVertex[i2] = true;
          tob->nogoEdge[i2] = true;
          lastIn = false;
        }
      }
      else if (d < margin)
      {
        if (i2 != idx[1])
        { // mark following edge only
          // for first point
          tob->nogoEdge[i2] = true;
          tob->nogoVertex[i1] = true;
        }
        tob->nogoEdge[i1] = true;
      }
      i2 = i1;
      i1 = (i1 + 1) % m;
    }
  }
}

/////////////////////////////////////

bool UAvoidLink::isWithin(double x, double lim1, double lim2)
{
  bool result;
  const double eps = 1e-4;
  double a = fmin(lim1, lim2);
  double b = fmax(lim1, lim2);
  result  = x > a + eps and x < b - eps;
  return result;
}
