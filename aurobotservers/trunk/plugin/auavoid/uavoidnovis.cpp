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
#include "uavoidnovis.h"

ULineSegment UAvoidNoVis::getNoVisSegment(int idx)
{
  ULineSegment seg;
  UPosition p1, p2;
  //int a, b, i;
  //
  // find the most (index-wise) central point of the near vertices
  /*  a = first;
  b = last;
  if (a > b)
    a -= aobThis->obst->getPointsCnt();
  i = (a + b) / 2;
  if (i < 0)
    i += aobThis->obst->getPointsCnt();*/
  //Get the vertex point
  //p1 = aobThis->obst->getPoint(i);
  //p1 = aobPos;
  //p1.z = 0.0;
  // get the COG point
  if (idx == 0)
    // even, so use this COG
    p2 = aobThis->obst->getCogXY();
  else
    // uneven index get to the other obstacles COG
    p2 = aobOther->obst->getCogXY();
  p2.z = 0.0;
  p1 = aobPos;
  p1.z = 0.0;
  // make line segment
  seg.setFromPoints(p1, p2);
  return seg;
}

////////////////////////////////////////////////////

ULineSegment UAvoidNoVis::getNoVisSegment()
{
  ULineSegment seg;
  UPosition p1, p2;
  //int a, b, i;
  // get a position on the other obstacle (not 1cm away)
  aobOther->obst->getClosestDistance(aobPos.x, aobPos.y, 100.0, &p1);
  // move point into other obstacle by a bit (1cm)
  if (aobOther->obst->getPointsCnt() > 1)
  { // if not a point
    seg.setFromPoints(p1, aobOther->obst->getCogXY());
    p1 = seg.getPositionOnLine(0.01);
  }
  p1.z = 0.0;
  // get the COG point in this obstacle
  p2 = aobThis->obst->getCogXY();
  p2.z = 0.0;
  // make line segment
  seg.setFromPoints(p1, p2);
  return seg;
}

/////////////////////////////////////////

void UAvoidNoVis::setNoVisSegment(int firstNear, int lastNear, UAvoidObst * thisObst, UAvoidObst * otherObst)
{
  UPosition p1;
  int edge, vert;
  //double d;
  int a, b, i;
  //
  a = firstNear;
  b = lastNear;
  if (a > b)
    a -= aobThis->obst->getPointsCnt();
  i = (a + b) / 2;
  if (i < 0)
    i += aobThis->obst->getPointsCnt();
  //first = firstNear;
  aobPos = thisObst->obst->getPoint(i);
  //last = lastNear;
  aobThis = thisObst;
  aobOther = otherObst;
  //
  if (false)
  { // this is now set when creating links to an obstacle in the same group
  //
  // mark a side on the other obstacle as inpassable too
  p1 = thisObst->obst->getPoint(firstNear);
  // get closest edge on other obstacle
  otherObst->obst->getDistance(p1.x, p1.y, &edge, &vert);
  // mark the found edge as not passable
  otherObst->nogoEdge[edge] = true;
  // a side has to be marked on this obstacle too.
  if (vert < 0)
    vert = edge;
  // get closest vertex (or first end if an edge)
  p1 = otherObst->obst->getPoint(vert);
  // get closest edge on this obstacle
  thisObst->obst->getDistance(p1.x, p1.y, &edge, &vert);
  // mark the vertex as not passable for a path
  thisObst->nogoEdge[edge] = true;
  }
}

/////////////////////////////////////////

void UAvoidNoVis::setNoVisSegment(UPosition posX, UAvoidObst * thisObst, UAvoidObst * otherObst)
{
  UPosition p1;
  ULineSegment s1;
  //
  p1 = thisObst->obst->getCogXY();
  // make line segment with origin in posX
  s1.setFromPoints(posX, p1);
  // get position on line 4mm away from posX towards COG of this obstacle
  aobPos = s1.getPositionOnLine(0.004);
  // set also the two obstacles involved
  aobThis = thisObst;
  aobOther = otherObst;
}
