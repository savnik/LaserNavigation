/** *************************************************************************
*   Copyright (C) 2010 by DTU (Christian Andersen)
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
*   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
*   Boston, MA 02110-1301, USA.
***************************************************************************/

#include "uavoidlink.h"
#include "uavoidobst.h"
#include "uavoidnovis.h"

bool UAvoidObst::addToGroupIfWithinMargin2(UAvoidObst * other, double margin,
                                           bool noAdd) // bool allowMerge
{
  UAvoidObst * aog = this;
  //int vTh, vOt;
  bool result = false;
  bool overlap;
  ULineSegment seg;
  //int n;
  UAvoidNoVis * nv;
  UPosition posX;
  //
  while (aog != NULL)
  { // test for overlap
    overlap = aog->obst->isOverlappingXYconvex2(other->obst, margin, &posX);
    if (overlap)
    { // new obstacle is member of this group.
      //
      // add no-visibility lines between obstacles.
        // vTh is the first vertex of this polygon, taht is too close to the other
        // vOt is the first vertex of the other obstacle that is too close.
        // any of them may be -1 to indicate that no vertex is within margin of the other.
        // add a no visibility line between the two obstacles
        if (aog->noVisCnt < NO_VIS_SEG_CNT)
        {
          nv = &aog->noVis[aog->noVisCnt++];
          nv->clear();
          nv->setNoVisSegment(posX, aog, other);
        }
        else
          printf("UAvoidObst::addToGroupIfWithinMargin2 - out of no-vis-lines\n");
        // and a link from other to this too
        if (other->noVisCnt < NO_VIS_SEG_CNT)
        {
          other->obst->isOverlappingXYconvex2(aog->obst, margin, &posX);
          nv = &other->noVis[other->noVisCnt++];
          nv->clear();
          nv->setNoVisSegment(posX, other, aog);
        }
        else
          printf("UAvoidObst::addToGroupIfWithinMargin2 - other out of no-vis-lines\n");
      //
      result = true;
      break;
    }
    // move to next obstacle in group
    aog = aog->grp;
  }
  // if no-add, then this new obstacle combines two groups and should not be added here
  if (result and not noAdd)
  { // insert the tested obstacle to this group (as new number 2)
    other->grp = grp;
    other->grpIdx = grpIdx;
    grp = other;
  }
  return result;
}

/////////////////////////////////////////


int UAvoidObst::getTangentVertexFrom(UPosition pos, int * idxCv, int * idxCcv)
{
  UPosition * pb1, *pb2;
  U2Dlined lb1, lb2;
  int n, j;
  bool b1, b2;
  int tangentCnt = 0;

  //
  n = obst->getPointsCnt(); // number of vertices in polygon
  if (n == 1)
  { // one point only , so tangent poitns are both the same
    *idxCv = 0;
    *idxCcv = 0;
  }
  else
  {
    pb1 = obst->getPoints();
    pb2 = &pb1[n - 1];
    // set line to previous point
    lb1.set2P(pb1->x, pb1->y, pb2->x, pb2->y);
    for (j = 0; j < n; j++)
    { // get next line in other polygon
      pb2 = pb1;
      if (j < n - 1)
        pb1++;
      else
        pb1 = obst->getPoints(); // first point again
      lb2 = lb1;
      // get line to next point
      lb1.set2P(pb1->x, pb1->y, pb2->x, pb2->y);
      // test on wich side the other point (pb2) is relative to lines on this polygon (la1 and la2)
      b1 = lb1.distanceSigned(pos.x, pos.y) >= 0.0; // if both negative then this (pa2) is behind obstacle
      b2 = lb2.distanceSigned(pos.x, pos.y) >  0.0; // if just one of (b1,b2) is positive then a tangent line
      if (b1 and not b2)
      { // a CV tangent
        *idxCv = j;
        tangentCnt++;
      }
      else if (b2 and not b1)
      { // a CCV tangent
        *idxCcv = j;
        tangentCnt++;
      }
    }
  }
  return tangentCnt;
}


/////////////////////////////////////////

bool UAvoidObst::addtangentLines(UAvoidObst * other, UAvoidLink * ala, UAvoidLink * alb,
                                 UAvoidLink * alc, UAvoidLink * ald)
{
  int i, j, n = 0, im, jm;
  U2Dline la1, la2, lb1, lb2;
  UPosition *pa1, *pa2, *pb1, *pb2;
  bool a1 = false, a2 = false, b1 = false, b2 = false;
  double da1, da2, db1, db2;
  int outher1Used = false, outher2Used = false;
  bool result = true;
  //
  pa1 = obst->getPoints(); // first polygon point
  im = obst->getPointsCnt() - 1; // index of last number
  jm = other->obst->getPointsCnt() - 1; // index of last point
  if (im > 0)
  { // more than one point in this obstacle
    pa2 = &pa1[im]; //  last polygon point
    la1.set2P(pa1->x, pa1->y, pa2->x, pa2->y); // line between first and last
  }
  else
  { // this obstacle has one point only, so any line is a tangent line
    a1 = true;
    a2 = false;
  }
  if (jm == 0)
  { // other obstacle has one point only, so any line is a tangent line
    b1 = true;
    b2 = false;
  }
  for (i = 0; i <= im; i++)
  { // get line to next point
    pa2 = pa1;
    if (im > 0)
    {
      if (i < im)
        pa1++;
      else
        pa1 = obst->getPoints(); // first point again
      la2 = la1;
      la1.set2P(pa1->x, pa1->y, pa2->x, pa2->y);
    }
    // get first line in other polygon
    pb1 = other->obst->getPoints();
    if (jm > 0)
    { // more than one point in other obstacle
      pb2 = &pb1[jm];
      lb1.set2P(pb1->x, pb1->y, pb2->x, pb2->y);
    }
    for (j = 0; j <= jm; j++)
    { // get next line in other polygon
      pb2 = pb1;
      if (jm > 0)
      {
        if (j < jm)
          pb1++;
        else
          pb1 = other->obst->getPoints(); // first point again
        lb2 = lb1;
        lb1.set2P(pb1->x, pb1->y, pb2->x, pb2->y);
      }
      // test on which side the other point (pb2) is relative to lines on this polygon (la1 and la2)
      if (im > 0)
      { // this obstacle is not a point obstacle
        da1 = la1.distanceSigned(pb2->x, pb2->y);
        da2 = la2.distanceSigned(pb2->x, pb2->y);
        a1 = da1 >= 0.005; // if both negative then others pb2 is behind obstacle
        a2 = da2 >  0.0; // if one positive, then tangent line
      }
      // test on which side this point (pa2) is relative to lines on this polygon (lb1 and lb2)
      if (jm > 0)
      { // other obstacle is not a point obstacle
        db1 = lb1.distanceSigned(pa2->x, pa2->y);
        db2 = lb2.distanceSigned(pa2->x, pa2->y);
        b1 = db1 >= 0.005; // if both negative then this (pa2) is behind other obstacle
        b2 = db2 >  0.0; // if one is positive then a tangent line
      }
      // the sign determines if the line is a tangent line
      // valid only if tangent line for both ends simultaniously
      if ((a1 != a2) and (b1 != b2) and not nogoVertex[i] and not other->nogoVertex[j])
      { // this is a tangent line, so insert into the
        // 4 tangent line descriptors (and count)
        if (a1)
        { // other to this is left
          if (b1)
          { // this to other is left
            ala->idx[0] = i;
            ala->aobIdx[0] = j;
            ala->valid[0] = true;
            alb->idx[0] = j;
            alb->aobIdx[0] = i;
            alb->valid[0] = true;
            n++;
          }
          else
          { //
            if (not outher1Used)
            {
              ala->idx[1] = i;
              ala->aobIdx[1] = j;
              ala->valid[1] = true;
              alb->idx[2] = j;
              alb->aobIdx[2] = i;
              alb->valid[2] = true;
              n++;
              outher1Used = true;
            }
            else
            {
              result=false;
              if (alc != NULL and ald != NULL)
              {
                alc->idx[1] = i;
                alc->aobIdx[1] = j;
                alc->valid[1] = true;
                ald->idx[2] = j;
                ald->aobIdx[2] = i;
                ald->valid[2] = true;
                alc->tangentCnt = maxi(alc->tangentCnt, 2);
                ald->tangentCnt = 3;
              }
            }
          }
        }
        else
        { // other to this is right
          if (b1)
          { // this to other is left
            if (not outher2Used)
            {
              ala->idx[2] = i;
              ala->aobIdx[2] = j;
              ala->valid[2] = true;
              alb->idx[1] = j;
              alb->aobIdx[1] = i;
              alb->valid[1] = true;
              n++;
              outher2Used = true;
            }
            else
            {
              result = false;
              if (alc != NULL and ald != NULL)
              {
                alc->idx[2] = i;
                alc->aobIdx[2] = j;
                alc->valid[2] = true;
                ald->idx[1] = j;
                ald->aobIdx[1] = i;
                ald->valid[1] = true;
                alc->tangentCnt = 3;
                ald->tangentCnt = maxi(ald->tangentCnt, 2);
              }
            }
          }
        else
          {
            ala->idx[3] = i;
            ala->aobIdx[3] = j;
            ala->valid[3] = true;
            alb->idx[3] = j;
            alb->aobIdx[3] = i;
            alb->valid[3] = true;
            n++;
          }
        }
      }
      if (n >= 4)
        // we'r finished
        break;
    }
    if (n >= 4)
      break;
  }
  // add the result to the onstacle descriptor
  ala->mirror = alb;
  ala->tob = this;
  ala->aob = other;
/*  ala->next = links;
  links = ala;*/
  //
  alb->mirror = NULL;
  alb->tob = other;
  alb->aob = this;
/*  alb->next = other->links;
  other->links = alb;*/
  if (alc != NULL and ald != NULL)
  { // additional set of outher links are available - are they used
    if (alc->tangentCnt > 0)
    { // finish the remaining pointers for this set too
      alc->mirror = ald;
      alc->tob = this;
      alc->aob = other;
/*      alc->next = links;
      links = alc;*/
      //
      ald->mirror = NULL;
      ald->tob = other;
      ald->aob = this;
/*      ald->next = other->links;
      other->links = ald;*/
    }
  }
  // set max tangent count for first set of links
  if (n == 4)
  {
    ala->tangentCnt = 4;
    alb->tangentCnt = 4;
  }
  else if (n > 0)
  { // some are valid, so reduce count as much as possible
    for (i = 3; i >= 0; i--)
    {
      if (ala->valid[i])
      {
        ala->tangentCnt = i + 1;
        break;
      }
    }
    for (i = 3; i >= 0; i--)
    {
      if (alb->valid[i])
      {
        alb->tangentCnt = i + 1;
        break;
      }
    }
  }
  else
  { // obstacles has complete overlap and all
    // links are - as they should be - invalid.
    // printf("UAvoidObst::addtangentLines - two obstacles with complete overlap! (should be OK)\n");
  }
  return result;
}

/////////////////////////////////////////

void UAvoidObst::testVisibility(FILE * logdbg)
{
  UAvoidLink * la, *lb;
  int n;
  //
  la = links;
  while (la != NULL)
  { // test all links against all other links
    lb = links;
    while (lb != NULL)
    {
      if ((la != lb) and (la->mirror != NULL))
      { // the candidate obstacles that can obstruct
        // visibily has already link-lines defined to this
        // obstacle, use this to test if they obstruct
        // visibility -
        // but ignore if the obstacle embeds start or end
        if (logdbg != NULL)
        {
          fprintf(logdbg, "  testing links from obst %d.%lu to %d.%lu  against %d.%lu (pre ",
                  grpIdx, obst->getSerial(), la->aob->grpIdx, la->aob->obst->getSerial(), lb->aob->grpIdx, lb->aob->obst->getSerial());
                  for (n = 0; n < 4; n++)
                    fprintf(logdbg, "%d ", la->valid[n] and n < la->tangentCnt);
                  fprintf(logdbg, " -> (post ");
        }
        if (not (
           ((isStart or la->aob->isStart) and lb->aob->embedStart) or
           ((isExit or la->aob->isExit) and lb->aob->embedEnd)
                ))
          la->visibilityTest(lb);
        else
        {
          // debug
          printf("UAvoidObst::testVisibility: ignored obstacle %lu as "
              "it embeds start %s or end %s\n",
              lb->aob->obst->getSerial(),
              bool2str(isStart or la->aob->isStart),
              bool2str(isExit or la->aob->isExit));
          // debug end
        }
        if (logdbg != NULL)
        { // save also links after test
          for (n = 0; n < 4; n++)
            fprintf(logdbg, "%d ", la->valid[n] and n < la->tangentCnt);
          fprintf(logdbg, ")\n");
        }
      }
      lb = lb->next;
    }
    la = la->next;
  }
}


/////////////////////////////////////////

bool UAvoidObst::validTangent(bool entryCV, int entryVertex, UPosition p1)
{ // test if a valid exit tangent can be found
  UAvoidLink * lnk;
  bool result;
  int i, k; //, m, n;
  UPosition pv, p2, p3, cog;
  U2Dlined lin1, lin2;
  //const int MOA = 4;
  //UAvoidObst * obst2[MOA];
  //int obst2Cnt;
  //bool isOk, side1, side2;
  //double aEntry, aExit, aCog1, aCog2;
  //double aMin, aMax;
  //
  lnk = links;
  // get number of points in this obstacle
  if (obst->getPointsCnt() == 1)
    // no hidden edges in a one-point obstacle
    result = true;
  else if (nogoVertex[entryVertex])
    // this vertex is too close to another obstacle to pass
    result = false;
  else
  { // defaulet is no exit tangent
    result = false;
    while (lnk != NULL)
    { // entry tangent seen from
      if (entryCV)
        k = 2; // exit must be 2 or 3
      else
        k = 0; // exit must be 0 or 1
      for (i = 0; i < 2; i++)
      { // get number of possiblle exit tangents
        if (lnk->valid[k])
        {
          if ((entryVertex == lnk->idx[k]) and (not nogoVertex[entryVertex]))
            // entry and exit vertex is the same, and this is not a vertex
            // that is too close to other bstacles.
            result = true;
          else if (not passingNoGoEdge(entryCV, entryVertex, lnk->idx[k]))
          { // a possible route is found, but a no-visibility line may have
            // been crossed at enrty to the obstacle
            // we may be finished
            result = true;
          }
          if (result)
            // exit tangent is usable
            break;
        }
        //
        k++;
      }
      if (result)
        break;
      lnk = lnk->next;
    }
  }
  return result;
}

//////////////////////////////////////////////////////

bool UAvoidObst::passingNoGoEdge2(bool entryCV, int entryVertex, int exitVertex,
                                 UPosition pPrev, UPosition pNext,
                                  bool isStart, bool isExit)
{
  bool nogo;
  double aEntry[2] = {0.0, 0.0};
  double aExit[2];
  bool nogoBelow[2] = {false, false};
  bool nogoBetween[2] = {false, false};
  bool nogoAbove[2] = {false, false};
  int i, i2, i3;
  UPosition p1, p2, p3, p4;
  bool samePoint;
  double a, d, ds, de;
  //
  if (noVisCnt > 0)
  { // there is lines
    samePoint = (entryVertex == exitVertex);
    p1 = obst->getPoint(entryVertex);
    p4 = obst->getPoint(exitVertex);
    // entry angle at entry point
    aEntry[0] = atan2(pPrev.y - p1.y, pPrev.x - p1.x);
    // exit angle at exit point
    aExit[0] = atan2(pNext.y - p4.y, pNext.x - p4.x);
    aExit[1] = aExit[0];
    if (isStart)
      ds = hypot(pPrev.y - p1.y, pPrev.x - p1.x);
    else
      ds = 1e10;
    if (isExit)
      de = hypot(pNext.y - p4.y, pNext.x - p4.x);
    else
      de = 1e10;
    if (not samePoint)
    { // need 2 tests at entry and at exit
      // exit now uses index 1
      // needs now exit at entry point and
      // entry to exit point
      if (entryCV)
      {
        i2 = entryVertex - 1;
        if (i2 < 0)
          i2 = obst->getPointsCnt() - 1;
        i3 = (exitVertex + 1) % obst->getPointsCnt();
      }
      else
      {
        i2 = (entryVertex + 1) % obst->getPointsCnt();
        i3 = exitVertex - 1;
        if (i3 < 0)
          i3 = obst->getPointsCnt() - 1;
      }
      p2 = obst->getPoint(i2);
      aExit[0] = atan2(p2.y - p1.y, p2.x - p1.y);
      p3 = obst->getPoint(i3);
      aEntry[1] = atan2(p3.y - p4.y, p3.x - p4.y);
    }
    // mark for all no-vis lines
    for (i = 0; i < noVisCnt; i++)
    { // now p1 is entry into this obstacle
      //     p4 is exit point on this obstacle
      p2 = noVis[i].aobOther->obst->getCogXY();
      a = atan2(p2.y - p1.y, p2.x - p1.x);
      if (isStart or isExit)
        d = hypot(p2.y - p1.y, p2.x - p1.x);
      else
        d = 0.0;
      if (d < ds)
      { // not prev or next inside obstacle group
        if (a < aEntry[0] and a < aExit[0])
          nogoBelow[0] = true;
        else if (a < aEntry[0] or a < aExit[0])
          nogoBetween[0] = true;
        else
          nogoAbove[0] = true;
      }
      if (not samePoint and d < de)
      { // needed for exit point too
        a = atan2(p2.y - p4.y, p2.x - p4.x);
        if (a < aEntry[1] and a < aExit[1])
          nogoBelow[1] = true;
        else if (a < aEntry[1] or a < aExit[1])
          nogoBetween[1] = true;
        else
          nogoAbove[1] = true;
      }
    }
    // test for result
    for (i = 0; i < 2; i++)
    { // test outher part of path - must be free of group members
      if (obst->getPointsCnt() == 1)
      { // both ways (CV and CCV must be blocked to be a nogo
        nogo = (nogoBelow[0] or nogoAbove[0]) and nogoBetween[0];
      }
      else if (entryCV)
      { // entry is CV - that is angles from entry down to exit must be free
        if (aEntry[0] < aExit[0])
          nogo = nogoBelow[0] or nogoAbove[0];
        else
          nogo = nogoBetween[0];
      }
      else
      { // entry is CCV, so angle from entry up to exit must be free
        if (aEntry[0] > aExit[0])
          nogo = nogoBelow[0] or nogoAbove[0];
        else
          nogo = nogoBetween[0];
      }
      if (samePoint or nogo)
        break;
    }
  }
  else
    nogo = false;
  return nogo;
}

//////////////////////////////////////////////

bool UAvoidObst::passingNoGoEdge(bool entryCV, int entryVertex, int exitVertex)
{
  int e0, e1, e2, en;
  bool nogo;
  // set start (e1) and end vertex (e2)
  if (entryCV)
  { // exit has lowest vertex number
    e0 = exitVertex;
    e2 = entryVertex;
  }
  else
  { // CCV, so entry has lowest vertex number
    e0 = entryVertex;
    e2 = exitVertex;
  }
  nogo = false;
          // get end vertex
  e1 = e0;
  en = obst->getPointsCnt();
  if (en == 0)
    printf("Error in polygon size?!!!\n");
  else
    while (e1 != e2)
    {
      if (nogoEdge[e1])
      { // the segment connectiong vertex e1 to e1+1
        // is a no-go segment (too close to other obstacle)
        nogo = true;
        break;
      }
      e1 = (e1 + 1) % en;
    }
  //
  return nogo;
}


////////////////////////////////////////////

bool UAvoidObst::crossingNonVisibilityLine(ULineSegment * visLine, FILE * logdbg)
{
  UAvoidObst * aoga;
  int j, k;
  UPosition pos;
  bool isX = false;
  UAvoidNoVis * snv;
  ULineSegment seg;
  double t1 = 0.0, t2 = 0.0; // parameter line parameter [in meter]
  const double tanEndMargin = 0.002; // tangent line end limits
  //
  aoga = this;
  while (aoga != NULL)
  {
    snv = aoga->noVis;
    for (j = 0; j < aoga->noVisCnt; j++)
    {
      for (k = 0; k < 2; k++)
      {
        seg = snv->getNoVisSegment(k);
        //isX = seg.getSegmentCrossingXY(visLine, &pos);
        isX = seg.getXYCrossing(*visLine, &pos);
        if (isX)
        { // is the cross on the tangent line
          t1 = visLine->getPositionOnLine(&pos);
          isX = (t1 > tanEndMargin) and (t1 < (visLine->length - tanEndMargin));
        }
        if (isX)
        { // is the cross on the no-vis segmemnt
          t2 = seg.getPositionOnLine(&pos);
          isX = (t2 >= 0) and (t2 <= seg.length);
        }
        if (logdbg != NULL and isX)
        {
          fprintf(logdbg, " # obst %lu visline (%.2fx,%.2fy,%.2fm,%.2fvx,%.2fvy)\n",
          aoga->obst->getSerial(), visLine->pos.x, visLine->pos.y, visLine->length, visLine->vec.x, visLine->vec.y);
          fprintf(logdbg, " --> crossed novis line %d part %d (%.2fx,%.2fy,%.2fm,%.2fvx,%.2fvy) at t1=%.3f t2=%.3f\n",
                j, k, seg.pos.x, seg.pos.y, seg.length, seg.vec.x, seg.vec.y, t1, t2);
        }
        if (isX)
          break;
      }
      if (isX)
        break;
      snv++;
    }
    if (isX)
      break;
    //
    aoga = aoga->grp;
  }
  return isX;
}

/////////////////////////////////////////

double UAvoidObst::getClosestPoint(UPosition to, double distLimit, UPosition * closest, bool * atVertex)
{
  double d1, d2;
  bool v1, v2;
  //
  d1 = obst->getClosestDistance(to.x, to.y, distLimit, closest, &v1);
  if (grp != NULL)
  { // test the remaining part of the group
    d2 = grp->getClosestPoint(to, mind(distLimit, d1), closest, &v2);
    if (d2 < d1)
    {
      d1 = d2;
      v1 = v2;
    }
  }
  if (atVertex != NULL)
    *atVertex = v1;
  return d1;
}

/////////////////////////////////////////

bool UAvoidObst::isInSameGroup(UAvoidObst * other)
{
  if (grp != NULL)
    return grp->isInSameGroup(other);
  else
    return (grp == other);
}
