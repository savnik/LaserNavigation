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
#include "upolygon.h"
#include "u2dline.h"
#include "uline.h"

bool UPolygon::setSize(int maxPoints)
{
  if (maxPoints > pointsMax)
  {
    points = (UPosition*) realloc(points, maxPoints * sizeof(UPosition));
    ppoints = (UPosition**) realloc(ppoints, maxPoints * sizeof(UPosition*));
    if (points != NULL and ppoints != NULL)
      pointsMax = maxPoints;
    else
      pointsMax = 0;
  }
  return (pointsMax >= maxPoints);
}

///////////////////////////////////////////////////

void UPolygon::print(const char * prestring)
{
  int i;
  UPosition * pos = points;
  const int MSL = 15;
  char s[MSL];
  //
  fprintf(stdout, " %s has %d points (max %d)\n", prestring,
          pointsCnt, pointsMax);
  for (i = 0; i < pointsCnt; i++)
  {
    snprintf(s, MSL, " - %3d ", i);
    pos->print(s);
    pos++;
  }
}

/////////////////////////////////////////

bool UPolygon::isAlsoA(const char * typeString)
{
  bool result;
  result = (strcmp(UPolygon::getDataType(), typeString) == 0);
  if (not result)
      // ask the ancestor if type is known
    result = UDataBase::isAlsoA(typeString);
  return result;
}

///////////////////////////////////////////////////////

void UPolygon::snprint(const char * prestring, char * buff, const int buffCnt)
{
  int i;
  UPosition * pos = points;
  const int MSL = 15;
  char s[MSL];
  char *p1;
  int n;
  //
  snprintf(buff, buffCnt, "%s has %d/%d points ispolyline=%s COGvalid=%s cog=(%.2fx,%.2fy) maxDist=%.2f\n",
           prestring, pointsCnt, pointsMax, bool2str(isPolyline()),
           bool2str(cogXYvalid), cogXY.x, cogXY.y, cogXYmaxDist);
  n = strlen(buff);
  p1 = &buff[n];
  for (i = 0; i < pointsCnt; i++)
  {
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(s, MSL, " - %3d ", i);
    pos->snprint(s, p1, buffCnt - n);
    pos++;
  }
}

///////////////////////////////////////////////////////

void UPolygon::printSort(const char * prestring)
{
  int i, n;
  UPosition pos;
  const int MSL = 15;
  char s[MSL];
  //
  fprintf(stdout, " %s has %d sorted points of %d (max %d)\n", prestring,
          ppointsCnt, pointsCnt, pointsMax);
  for (i = 0; i < ppointsCnt; i++)
  {
    n = mini(pointsCnt - 1, maxi(0, ppoints[i] - points));
    pos = points[n];
    snprintf(s, MSL, " - %3d (%2d)", i, n);
    pos.print(s);
  }
}

///////////////////////////////////////////////////////

bool UPolygon::add(UPosition point)
{
  bool result;
  //
  result = (pointsCnt < pointsMax);
  if (result)
    points[pointsCnt++] = point;
  return result;
}

///////////////////////////////////////////////////////

bool UPolygon::add(UPosition * point)
{
  bool result;
  //
  result = (pointsCnt < pointsMax);
  if (result)
    points[pointsCnt++] = *point;
  setCogXYvalid(false);
  return result;
}

///////////////////////////////////////////////////////

bool UPolygon::add(UPolygon * poly)
{
  bool result = true;
  int i;
  UPosition * ps;
  //
  ps = poly->getPoints();
  for (i = 0; i < poly->getPointsCnt(); i++)
  {
    result = add(ps++);
    if (not result)
      break;
  }
  setCogXYvalid(false);
  return result;
}

///////////////////////////////////////////////////////

UPosition * UPolygon::getLowerLeftXY()
{
  UPosition * pc = points;
  UPosition * pnt = points;
  int i;
  //
  pnt++;
  for (i = 1; i < pointsCnt; i++)
  {
    if (pnt->y < pc->y)
      pc = pnt;
    else if (pnt->y == pc->y)
      if (pnt->x < pc->x)
        pc = pnt;
    pnt++;
  }
  return pc;
}

////////////////////////////////////////////////////////

int angleSort(const void * first, const void * other)
{
  int result;
  double ** a = (double **)first;
  double ** b = (double **)other;
  //
  if (**a > **b)
    result = 1;
  else if (**a < **b)
    result = -1;
  else
    result = 0;
  //
  return result;
}

////////////////////////////////////////////////////////////////

// void UPolygon::sortByAngleXYTo(UPosition * pc)
// {
//   printf("Should not be called, as there is no elements in polygon here\n");
// }
// modified to dynamic allocation
void UPolygon::sortByAngleXYTo(UPosition * pc)
{
  double * angData = (double*) malloc(pointsCnt * sizeof(double));
  double ** pAngData = (double**) malloc(pointsCnt * sizeof(double*));
  double * a = angData;
  int i;
  unsigned long  j;
  UPosition * pos = points;
  //
  if (angData != NULL and pAngData != NULL)
  {
    ppointsCnt = 0;
    for (i = 0; i < pointsCnt; i++)
    { // calculate angles
      pAngData[ppointsCnt++] = a;
      *a = atan2(pos->y - pc->y, pos->x - pc->x);
      a++;
      pos++;
    }
    qsort(pAngData, pointsCnt, sizeof(void*), angleSort);
    //
    // arrange pointers to UPosition (pdata) in the same way
    a = angData;
    ppointsCnt = 0;
    for (i = 0; i < pointsCnt; i++)
    { // get index to element i
      j = (pAngData[i] - a); // / sizeof(double);
      // index to element i is j
      pos = &points[j];
      if (pos != pc)
        ppoints[ppointsCnt++] = &points[j];
    }
  }
  else
    perror("UPolygon::sortByAngleXYTo: memory allocation error");
  if (angData != NULL)
    free(angData);
  if (pAngData != NULL)
    free(pAngData);
}


////////////////////////////////////////////////////////////////


UPosition UPolygon::pop()
{
  UPosition result;
  //
  if (pointsCnt > 0)
    result = points[--pointsCnt];
  setCogXYvalid(false);
  return result;
}

/////////////////////////////////////////////////////////

void UPolygon::remove(int index)
{ // remove the position at index - moving the rest down
  int n;
  //
  n = pointsCnt - index - 1;
  //
  if ((n > 0) and (n < (pointsCnt - 1)))
    memmove(&points[index], &points[index + 1], n * sizeof(UPosition));
  if ((n >= 0) and (n < pointsCnt))
  { // remove last position
    pointsCnt--;
    setCogXYvalid(false);
  }
}

/////////////////////////////////////////////////////////

bool UPolygon::insert(UPosition pos, int index)
{ // insert position at index - moving the rest up, and discard last point if no more space
  int n;
  bool result;
  //
  n = pointsCnt - index;
  result = (pointsCnt < pointsMax);
  if (not result)
    // no space - discard last point
    n--;
  //
  if ((n > 0) and (n < pointsCnt))
  {
    memmove(&points[index + 1], &points[index], n * sizeof(UPosition));
    points[index] = pos;
    setCogXYvalid(false);
  }
  if (result)
    pointsCnt++;
  return result;
}

/////////////////////////////////////////////////////////

UPosition * UPolygon::getFromTop(int fromTop)
{
  UPosition * result = NULL;
  //
  if (pointsCnt > fromTop)
    result = &points[pointsCnt - 1 - fromTop];
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::extractConvexTo(UPolygon * destination)
{
  bool result = true;
  UPosition *pc; // corner points top, right, bottom, left
  UPosition * ntt; // next to top
  UPosition * top; // top of list
  UPosition * npi; // next point[i]
  UPolygon * dest = destination;
  int i;
  double a, a0, a1, dx, dy;
  //
  dest->clear();
  if (pointsCnt == 1)
    result = dest->add(points[0]);
  else if (pointsCnt == 2)
  {
    result = dest->add(points[0]);
    result = dest->add(points[1]);
  }
  else
  { // at least 3 points, so do full convex hull algorithem
    pc = getLowerLeftXY();
    // sort by angle
    sortByAngleXYTo(pc);
    // add start point
    dest->push(pc);
    dest->push(ppoints[0]);
    dest->push(ppoints[1]);
    for (i = 2; i < ppointsCnt; i++)
    {
      npi = ppoints[i];
      while (true)
      { // remove last point if concave (right turn)
        if (dest->getPointsCnt() < 2)
        {
          //printf("UPolygon::extractConvexTo: error - "
          //       "has removed too many points - why? exact same position, its OK\n");
          break;
        }
        ntt = dest->getFromTop(1);
        top = dest->getFromTop(0);
        a0 = atan2(ntt->y - top->y, ntt->x - top->x);
        dy = npi->y - top->y;
        dx = npi->x - top->x;
        if (fabs(dx) + fabs(dy) > 1e-10)
        { // npi and top is not the same point
          a1 = atan2(dy, dx);
          a = a1 - a0;
          if (a < 0.0)
            a += 2.0 * M_PI;
          if (a >= M_PI)
            break;
        }
        // not needed or 2 points at the same position
        dest->pop();
      }
      dest->push(ppoints[i]);
    }
  }
  strncpy(dest->color, color, 8);
  return result;
}

//////////////////////////////////////////

bool UPolygon::copySortedTo(UPolygon * destination)
{
  bool result = true;
  destination->clear();
  int i;
  //
  for (i = 0; i < ppointsCnt; i++)
  {
    result = destination->add(*ppoints[i]);
    if (not result)
    {
      printf("UPolygon::copySortedTo overload!\n");
      break;
    }
  }
  strncpy(destination->color, color, 8);
  return result;
}

//////////////////////////////////////////

bool UPolygon::copyPointsTo(UPolygon * destination)
{
  bool result = true;
  destination->clear();
  int i;
  //
  for (i = 0; i < pointsCnt; i++)
  {
    result = destination->add(points[i]);
    if (not result)
    {
      printf("UPolygon::copySortedTo overload!\n");
      break;
    }
  }
  return result;
}

//////////////////////////////////////////

bool UPolygon::copyTo(UPolygon * destination)
{
  bool result = true;
  destination->clear();
  int i;
  //
  for (i = 0; i < pointsCnt; i++)
  {
    result = destination->add(&points[i]);
    if (not result)
    {
      printf("UPolygon::copyTo overload!\n");
      break;
    }
  }
  destination->setAsPolygon(aPolygon);
  strncpy(destination->color, color, 8);
  return result;
}

////////////////////////////////////////////////

bool UPolygon::copy(CvPoint * source, int sourceCnt)
{
  bool result = ((sourceCnt + pointsCnt) <= pointsMax);
  UPosition pos;
  CvPoint * p = source;
  int i;
  //
  for (i = 0; i < sourceCnt; i++)
  {
    pos.x = p->x;
    pos.y = p->y;
    add(pos);
    if (pointsCnt >= pointsMax)
      break;
    p++;
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::getLimits(double * minx, double * miny,
                         double * maxx, double * maxy)
{
  bool result = pointsCnt > 0;
  int i;
  UPosition * p1;
  *minx = points->x;
  *maxx = points->x;
  *miny = points->y;
  *maxy = points->y;
  p1 = points + 1;
  for (i = 1; i < pointsCnt; i++)
  {
    if (p1->x < *minx)
      *minx = p1->x;
    if (p1->y < *miny)
      *miny = p1->y;
    if (p1->x > *maxx)
      *maxx = p1->x;
    if (p1->y > *maxy)
      *maxy = p1->y;
    p1++;
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::isOverlappingXY(UPolygon * poly2, int * xcnt, UPosition xes[], const int xesCnt)
{
  bool result;
  double minx1, maxx1, miny1, maxy1;
  double minx2, maxx2, miny2, maxy2;
  int i, j, cnt;
  UPosition *p1, *p2, *p3, *p4, px;
  ULineSegment seg1, seg2;
  //
  getLimits(&minx1, &miny1, &maxx1, &maxy1);
  poly2->getLimits(&minx2, &miny2, &maxx2, &maxy2);
  //
  result = hasOverlap(minx1, miny1, maxx1, maxy1,
                      minx2, miny2, maxx2, maxy2);
  cnt = 0;
  if (result)
  { // there is an overlap in bounding box, test more closely
    result = false;
    p1 = poly2->getPoints();
    for (i = 0; i < poly2->pointsCnt; i++)
    { // test all line segments in polygon2
      p2 = p1++;
      if (p1 > poly2->getFromTop(0))
        p1 = poly2->getPoints();
      // test for crossings
      seg1.setFromPoints(p1, p2);
      // get points from this segment
      p3 = points;
      for (j = 0; j < pointsCnt; j++)
      { // test line segment against line segments
        // in this polygon
        p4 = p3++;
        if (p3 > getFromTop(0))
          p3 = getPoints();
        seg2.setFromPoints(p3, p4);
        result = seg1.getSegmentCrossingXY(&seg2, &px);
        if (result)
        { // we need to count crossings, so continue
          if (xes != NULL and cnt < xesCnt)
            xes[cnt] = px;
          cnt++;
        }
      }
      if (result and xcnt == NULL)
        break;
    }
  }
  if (xcnt != NULL)
    *xcnt = cnt;
  return cnt > 0;
}

/////////////////////////////////////////////////////////////

bool UPolygon::isOverlappingXYtype(UPolygon * poly2, int * xcnt, UPosition xes[], const int xesCnt, int type[])
{
  bool result;
  double minx1, maxx1, miny1, maxy1;
  double minx2, maxx2, miny2, maxy2;
  int cnt;
  UPosition px;
  ULineSegment seg1, seg2;
  //
  getLimits(&minx1, &miny1, &maxx1, &maxy1);
  poly2->getLimits(&minx2, &miny2, &maxx2, &maxy2);
  //
  result = hasOverlap(minx1, miny1, maxx1, maxy1,
                      minx2, miny2, maxx2, maxy2);
  cnt = 0;
  if (result)
  { // there is an overlap in bounding box, test more closely
    result = false;
    for (int i = 0; i < poly2->pointsCnt; i++)
    { // test all line segments in polygon2
      seg1 = poly2->getSegment(i);
      // get points from this segment
      for (int j = 0; j < pointsCnt; j++)
      { // test line segment against line segments
        // in this polygon
        seg2 = getSegment(j);
        result = seg1.getSegmentCrossingXY(&seg2, &px);
        if (result)
        {
          if (type != NULL)
          { // find also crossing type - seen in positive x direction
            // type = 4 is a dead end as higher x-values will be inside both polygons
            // type = 5 is a new opening as higher x-values will be outside both polygons
            // type = 6 is an upper xrossing (positive y side of both polygons)
            // type = 7 is a lower xrossing (negative y-side of both polygons)
            if (( seg1.vec.x <= 0.0) and
                ( seg2.vec.x <= 0.0))
            { // is an upper crossing - both segments has negative X-vector (CCV polygon)
              type[cnt] = 6;
            }
            else if (seg1.vec.x * seg2.vec.x < 0.0)
            { // is an opening or a close
              if (seg1.vec.x > 0)
              { // seg1 is an underside of a polygon (positive x and CCV polygon)
                // reverse vector for obstacle 2 and compare y-part
                if (-seg2.vec.y > seg1.vec.y)
                  type[cnt] = 4;
                else
                  type[cnt] = 5;
              }
              else
              { // seg2 is an underside of a polygon (positive x and CCV polygon)
                // reverse vector for obstacle 1 and compare y-part
                if (-seg1.vec.y > seg2.vec.y)
                  type[cnt] = 4;
                else
                  type[cnt] = 5;
              }
            }
            else
              // must be that both segments has positive X-vector, and thus
              // a crossing below obstacles.
              type[cnt] = 7;
          }



          // we need to count crossings, so continue
          if (xes != NULL and cnt < xesCnt)
            xes[cnt] = px;
          cnt++;
        }
      }
      if (result and xcnt == NULL)
        break;
    }
  }
  if (xcnt != NULL)
    *xcnt = cnt;
  return cnt > 0;
}

/////////////////////////////////////////////////////////

int UPolygon::getCrossings(ULineSegment * seg, UPosition xes[], const int xesCnt)
{
  UPosition *p3, *p4, px;
  int cnt = 0;
  ULineSegment seg2;
  //
  p3 = points;
  for (int j = 0; j < pointsCnt; j++)
  { // test line segment against line segments
    // in this polygon
    p4 = p3++;
    if (p3 > getFromTop(0))
      p3 = getPoints();
    seg2.setFromPoints(p3, p4);
    if (seg->getSegmentCrossingXY(&seg2, &px))
    { // there is a crossing within segment at px
      if (xes != NULL and cnt < xesCnt)
        xes[cnt] = px;
      cnt++;
    }
  }
  return cnt;
}

/////////////////////////////////////////////////////////

bool UPolygon::isOverlappingXYconvex2(UPolygon * poly2, double margin, UPosition * close)
{
  bool result;
  double minx1, maxx1, miny1, maxy1;
  double minx2, maxx2, miny2, maxy2;
  int i, j;
  UPosition *p1, *p2, *p3, *p4, px;
  ULineSegment seg1, seg2;
  int ve1, ve2, eg1, eg2, w;
  double d, t;
  UPosition pClose;
  //
  getLimits(&minx1, &miny1, &maxx1, &maxy1);
  minx1 -= margin;
  maxx1 += margin;
  maxy1 += margin;
  miny1 -= margin;
  poly2->getLimits(&minx2, &miny2, &maxx2, &maxy2);
  //
  result = hasOverlap(minx1, miny1, maxx1, maxy1,
                      minx2, miny2, maxx2, maxy2);
  if (result)
  { // there is an overlap, test more closely
    result = false;
    if (poly2->getPointsCnt() == 1)
    { // special rule here
      p1 = poly2->getPoints();
      d = getDistance(p1->x, p1->y, &eg1, &ve1);
      if (d < margin)
      { // we are finished
        pClose = *p1;
        result = true;
      }
    }
    else if (getPointsCnt() == 1)
    { // this is a one-pointer, so get 
      p1 = getPoints();
      d = poly2-> getDistance(p1->x, p1->y, &eg1, &ve1);
      if (d < margin)
      { // we are finished
        if (ve1 >= 0)
          // vertex is closer, so use
          pClose = poly2->getPoint(ve1);
        else if (eg1 >= 0)
        { // closer to a side - get closer point on side
          seg1 = poly2->getSegment(eg1);
          t =seg1.getPositionOnLine(p1);
          pClose = seg1.getPositionOnLine(t);
        }
        else
          printf("Polygon error\n");
        result = true;
      }
    }
    else
    { // continue
      p1 = poly2->getPoints();
      d = getDistance(p1->x, p1->y, &eg1, &ve1);
      for (i = 0; i < poly2->pointsCnt; i++)
      { // test all line segments in polygon2
        p2 = p1++;
        eg2 = eg1;
        ve2 = ve1;
        if (p1 > poly2->getFromTop(0))
          p1 = poly2->getPoints();
        d = getDistance(p1->x, p1->y, &eg1, &ve1);
        if (d < margin)
        { // we are finished
          pClose = *p1;
          result = true;
          break;
        }
        if (eg1 != eg2 or ve1 != ve2)
        { // the line from p1 to p2 may cross obstacle 2
          seg1.setFromPoints(p1, p2);
          // get points from this segment
          p3 = points;
          for (j = 0; j < pointsCnt; j++)
          { // test line segment against line segments
            // in this polygon
            p4 = p3++;
            if (p3 > getFromTop(0))
              p3 = getPoints();
            seg2.setFromPoints(p3, p4);
            result = seg1.getSegmentCrossingXY(&seg2, &px);
            if (result)
            {
              pClose = px;
              break;
            }
          }
          if (not result)
          { // not crossing
            // see if any of points in this polygon
            // is sufficient close to candidate segment (seg1)
            p3 = points;
            for (j = 0; j < pointsCnt; j++)
            {
              d = seg1.getDistanceXYSigned(*p3, &w);
              if (d > 0.0 and d < margin)
              {
                result = true;
                pClose = *p3;
                break;
              }
              p3++;
            }
          }
        }
        if (result)
          break;
      }
    }
  }
  if (close != NULL)
    *close = pClose;
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::isOverlappingXYconvex(UPolygon * poly2, double margin, int * closeThis, int * closePoly2)
{
  bool result;
  double minx1, maxx1, miny1, maxy1;
  double minx2, maxx2, miny2, maxy2;
  int i;
  UPosition *p1, *p2;
  //
  getLimits(&minx1, &miny1, &maxx1, &maxy1);
  //
  minx1 -= margin;
  maxx1 += margin;
  maxy1 += margin;
  miny1 -= margin;
  //
  poly2->getLimits(&minx2, &miny2, &maxx2, &maxy2);
  // first test using square limits
  result = hasOverlap(minx1, miny1, maxx1, maxy1,
                      minx2, miny2, maxx2, maxy2);
  if (result)
  { // may not have overlap
    p1 = getPoints();
    p2 = poly2->getPoints();
    if ((pointsCnt == 1) and (poly2->getPointsCnt() == 1))
      result = (p1->dist(p2) < margin);
    else
    { // use distance to line method
      result = false;
      for (i = 0; i < pointsCnt; i++)
      { // are any vertices inside the other polygon
        result = poly2->isInsideConvex(p1->x, p1->y, margin);
        if (result)
        { // vertex i har overlap with poly2
          if (closeThis != NULL)
            *closeThis = i;
          break;
        }
        p1++;
      }
      if (not result)
      { // see if any of the verteces on the other polygon
        // are inside this
        for (i = 0; i < poly2->getPointsCnt(); i++)
        {
          result = isInsideConvex(p2->x, p2->y, margin);
          if (result)
          { // vertex i har overlap with this polygon
            if (closePoly2 != NULL)
              *closePoly2 = i;
            break;
          }
          p2++;
        }
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::isOverlappingXYconvexSeg(UPolygon * other, double margin,
                                        int closeThis[6], int * closeThisCnt,
                                        int closeOther[6], int * closeOtherCnt)
{
  bool result;
  double minx1, maxx1, miny1, maxy1;
  double minx2, maxx2, miny2, maxy2;
  int i;
  UPosition *p1, *p2;
  bool firstInside;
  bool isIn, isInLast;
  // there should never be more than 2 sets, and one split the first index, that is max used 3*2 = 6 points
  const int MAX_PTS = 6;
  //
  *closeThisCnt = 0;
  *closeOtherCnt = 0;
  //
  getLimits(&minx1, &miny1, &maxx1, &maxy1);
  //
  minx1 -= margin;
  maxx1 += margin;
  maxy1 += margin;
  miny1 -= margin;
  //
  other->getLimits(&minx2, &miny2, &maxx2, &maxy2);
  //
  result = hasOverlap(minx1, miny1, maxx1, maxy1,
                      minx2, miny2, maxx2, maxy2);
  if (result)
  { // may not have overlap
    p1 = getPoints();
    p2 = other->getPoints();
    if ((pointsCnt == 1) and (other->getPointsCnt() == 1))
    { // points could overlap within a margin
      result = (p1->dist(p2) < margin);
      if (result and margin < 0.01)
        // mark one as beeing embedded in the other
        (*closeThisCnt)--;
    }
    else
    { // use distance to line method
      result = false;
      firstInside = false;
      isInLast = false;
      for (i = 0; i < pointsCnt; i++)
      { // are any vertices inside the other polygon
        isIn = other->isInsideConvex(p1->x, p1->y, margin);
        if (isIn)
        { // vertex i har overlap with poly2
          if (i == 0)
            firstInside = true;
          if (not isInLast)
          {
            if (*closeThisCnt >= MAX_PTS)
              break;
            closeThis[(*closeThisCnt)++] = i;
            isInLast = true;
            result = true;
          }
        }
        else if (isInLast)
        {
          closeThis[(*closeThisCnt)++] = i;
          isInLast = false;
        }
        p1++;
      }
      if (isInLast and *closeThisCnt < MAX_PTS)
      { // may end here or as part of forst near segment
        if (firstInside)
          closeThis[0] = closeThis[--(*closeThisCnt)];
        else
          closeThis[(*closeThisCnt)++] = i;
        if (closeThisCnt == 0)
          // mark as totally inside this obstacle
          (*closeThisCnt)--;
      }
      // see if any of the verteces on the other polygon
      // are inside this
      firstInside = false;
      isIn = false;
      isInLast = false;
      for (i = 0; i < other->getPointsCnt(); i++)
      {
        isIn = isInsideConvex(p2->x, p2->y, margin);
        if (isIn)
        { // vertex i har overlap with this polygon
          if (i == 0)
            firstInside = true;
          if (not isInLast)
          {
            if (*closeOtherCnt >= MAX_PTS)
              break;
            closeOther[(*closeOtherCnt)++] = i;
            isInLast = true;
            result = true;
          }
        }
        else if (isInLast)
        {
          closeOther[(*closeOtherCnt)++] = i;
          isInLast = false;
        }
        p2++;
      }
      if (isInLast and *closeOtherCnt < MAX_PTS)
      { // may end here or as part of forst near segment
        if (firstInside)
          closeOther[0] = closeOther[--(*closeOtherCnt)];
        else
          closeOther[(*closeOtherCnt)++] = i;
        if (closeOtherCnt == 0)
          // mark as totally inside this obstacle
          (*closeOtherCnt)--;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UPolygon::isInsideConvex(double x, double y, double margin)
{
  bool result;
  U2Dseg side;
  UPosition *p1, *p2;
  int i, w;
  double d;
  //
  p1 = getPoints();
  if (pointsCnt == 1)
  {
    d = hypot(x - p1->x, y - p1->y);
    result = d < margin;
  }
  else if (pointsCnt == 2)
  {
    p2 = p1++;
    side.setFromPoints(p1->x, p1->y, p2->x, p2->y);
    d = side.getDistanceSigned(x, y, &w);
    result = (w == 0) and (fabs(d) <= margin);
  }
  else if (pointsCnt > 2)
  {
    result = true;
    for (i = 0; i < pointsCnt; i++)
    {
      p2 = p1++;
      if (p1 > getFromTop(0))
        p1 = getPoints();
      side.setFromPoints(p1->x, p1->y, p2->x, p2->y);
      d = side.distanceSigned(x, y);
      result = d <= margin;
      if (not result)
        break;
    }
  }
  else
    // no points --- (assumed all over the place)
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////////

double UPolygon::getDistance(double x, double y,
                             int * closeEdge,
                            int * closeVertex)
{
  U2Dlined side;
  UPosition *p1, *p2, p3;
  int i, iMax = -1, vMax = -1, w;
  double d, dv, dMax = 0.0;
  ULineSegment seg;
//  bool increasing = false;
  //
  p1 = getPoints();
  if (pointsCnt == 1)
  { // one point only, so that is easy
    dMax = hypot(x - p1->x, y - p1->y);
    vMax = 0;
  }
  else if (pointsCnt > 1)
  {
    for (i = 0; i < pointsCnt; i++)
    {
      p2 = p1++;
      if (p1 > getFromTop(0))
        p1 = getPoints();
      side.set2P(p1->x, p1->y, p2->x, p2->y);
      d = side.distanceSigned(x, y);
      if (i == 0)
      {
        dMax = d;
        iMax = i;
      }
      else if (d > dMax)
      {
        dMax = d;
        iMax = i;
        //increasing = true;
      }
/*      else if (increasing)
        // we have passed the top, so no better
        // value is to be found in a convex polygon
        break;*/
    }
    // test if point is closer to a vertex.
    // get best line segment
    p2 = &points[iMax];
    i = iMax + 1;
    if (i >= pointsCnt)
      i = 0;
    p1 = &points[i];
    p1->z = 0.0;
    p2->z = 0.0;
    seg.setFromPoints(p1, p2);
    p3.set(x, y, 0.0);
    dv = seg.getDistanceFromSeg(p3, &w);
    if (w > 0)
    { // closer to one of the ends
      dMax = dv;
      if (w == 1)
        // closer to start (p1)
        vMax = i;
      else
        // closer to other end (p2)
        vMax = iMax;
    }
  }
  //
  if (closeEdge != NULL)
    *closeEdge = iMax;
  if (closeVertex != NULL)
    *closeVertex = vMax;
  //
  return dMax;
}


/////////////////////////////////////////////////////////

int UPolygon::getMostDistantXY(ULineSegment * line, bool leftSide, bool insideSegment, double * distance, int exclude, double * tPos)
{
  UPosition *p1;
  int i, imax = -1;
  double d, min = 1e50, max = -1e50, t = 0.0, tMax = 0.0;
  U2Dlined lxy;
  bool isInside = true;
  //
  lxy.setPV(line->pos.x, line->pos.y, line->vec.x, line->vec.y);
  p1 = getPoints();
  for (i = 0; i < pointsCnt; i++)
  {
    if (exclude != i)
    {
      if (insideSegment or tPos != NULL)
      {
        t = line->getPositionOnLine(p1);
        if (insideSegment)
          isInside = (t >= 0.0) and (t <= line->length);
      }
      if (isInside)
      {
        d = lxy.distanceSigned(p1->x, p1->y);
        if (leftSide)
        { // d is positive if to the left
          if (d > max)
          { // this is more to the left
            max = d;
            imax = i;
            tMax = t;
          }
        }
        else
        { // d is negative to the right
          if (d < min)
          {
            min = d;
            imax = i;
            tMax = t;
          }
        }
      }
    }
    p1++;
  }
  //
  if (distance != NULL)
  {
    if (leftSide)
      *distance = max;
    else
      *distance = min;
  }
  if (tPos != NULL)
    *tPos = tMax;
  return imax;
}

/////////////////////////////////////////////////////////

double UPolygon::getMostDistantVertexXY(double x, double y, double h,
                                        int side,
                                     UPosition * posOnPolygon)
{
  UPosition * p1, * pm = NULL;
  int i;
  double d, maxD = 0.0;
  U2Dlined line;
  //
  line.setPV(x,y, cos(h), sin(h));
  p1 = getPoints();
  for (i = 0; i < pointsCnt; i++)
  {
    d = line.distanceSigned(p1->x, p1->y);
    if (side == 0)
      d = -d;
    if ((d > maxD) or (i == 0))
    { // save more distant position
      maxD = d;
      pm = p1;
    }
    p1++;
  }
  if (posOnPolygon != NULL)
    *posOnPolygon = *pm;
  return maxD;
}

// debug
/*void * debug_lastPoly = NULL;
UPosition debug_lastPos;
int debug_lastPntCnt = 0;
void * debug_lastFromTop = NULL;
double debug_minDist = -1.0;
int debug_count = 0;*/
// debug end

////////////////////////////////////////////////////////

double UPolygon::getDistanceXYsigned2(UPosition pos,
                                   int * idx,
                                   UPosition * posOnPolygon, bool * vertex)
{
  double result = 1e20;
  ULineSegment side;
  UPosition *p1, *p2;
  int i, m;
  double d;
  double mind = 1e20;
  int minIdx = -1;
  int where;
  //
  p1 = getPoints();
  if (isPolygon())
    m = pointsCnt;
  else
    m = pointsCnt - 1;
  for (i = 0; i < m; i++)
  {
    p2 = p1++;
    if (p1 > getFromTop(0))
      p1 = getPoints();
    side.setFromPoints(p1, p2);
    d = fabs(side.getDistanceXYSigned(pos, NULL));
    if (d < mind)
    { // test end distance
      mind = d;
      minIdx = i;
    }
  }
  if (minIdx >= 0)
  { //result is valid
    // get result signed
    result = mind;
    p2 = &points[minIdx];
    i = minIdx + 1;
    if (i >= pointsCnt)
      i = 0;
    p1 = &points[i];
    side.setFromPoints(p1, p2);
    result = side.getDistanceXYSigned(pos, &where);
    if (where != 0)
      // closest to a vertex
      /// @todo NB! assumes that polygon is convex!!
      result = fabs(result);
    // return closest point
    if (posOnPolygon != NULL)
    {
      if (where == 1) // first end
        *posOnPolygon = *p1;
      else if (where == 2) // second end
        *posOnPolygon = *p2;
      else
      { // closest to a side
        // get line parameter value
        d = side.getPositionOnLine(pos);
        // get position for this parameter value
        *posOnPolygon = side.getPositionOnLine(d);
      }
    }
    // return index
    if (idx != NULL)
    { // return index to point or side closest to (x,y)
      // where is set to 0= on line, 1=point 1, 2= point 2
      if (where == 2)
      {
        if (minIdx == pointsCnt - 1)
          *idx = 0;
        else
          *idx = minIdx + 1;
      }
      else
        *idx = minIdx;
    }
    if (vertex != NULL)
      *vertex = (where > 0);
  }

  //UPosition debug_lastPos;
  //int debug_lastPntCnt = 0;
  // debug
/*  printf("UPolygon::getDistanceXYsigned %gx %gy %xthis %xlast %dcnt %gdist %d\n",
         pos.x, pos.y, this, getFromTop(0), pointsCnt, result, debug_count);
  if ((getFromTop(0) == debug_lastFromTop) and
       (this == debug_lastPoly) and
       (debug_lastPntCnt == pointsCnt) and
       (fabs(debug_lastPos.x - pos.x) < 0.0001) and
       (fabs(debug_lastPos.y == pos.y) < 0.0001) and
       (fabs(debug_minDist - result) < 0.0001))
  {
    debug_count++;
    if (debug_count > 8)
      printf("****ERROR %d UPolygon::getDistanceXYsigned - repeated call for same solution!!!\n", debug_count);
  }
  else
    debug_count = 0;
  debug_lastFromTop = getFromTop(0);
  debug_lastPoly = this;
  debug_lastPntCnt = pointsCnt;
  debug_lastPos = pos;
  debug_minDist = result;*/
  // debug end
  return result;
}

/////////////////////////////////////////////////////////

UPosition UPolygon::getCogXY()
{
  double a, aSum;
  int i;
  UPosition cg;
  UPosition cgSum;
  UPosition *p1, *p2, *p3;
  //
  if (not cogXYvalid)
  {
    if (pointsCnt == 1)
      // just a point
      cgSum = points[0];
    else if (pointsCnt == 2)
    { // just a line
      cgSum = points[0] + points[1];
      cgSum.scale(0.5);
    }
    else
    { // a triangle or more complex
      aSum = 0;
      p1 = &points[0];
      p2 = p1 + 1;
      p3 = p1 + 2;
      for (i = 2; i < pointsCnt; i++)
      {
        cg = *p1 + *p2 + *p3;
        a = fabs(p1->x * p2->y + p1->y * p3->x + p3->y * p2->x -
            p2->y * p3->x - p1->y * p2->x - p1->x * p3->y)/2.0;
        cg.scale(a/3.0);
        aSum += a;
        cgSum = cgSum + cg;
        p2 = p3;
        p3++;
      }
      if (aSum > 1e-50)
        cgSum.scale(1.0/aSum);
      else
        // must be a flat triangle (should be reduced to 2 points)
        // by convex hull function
        cgSum = cg;
    }
    cogXY = cgSum;
    p1 = points;
    cogXYmaxDist = 0.0;
    for (i = 0; i < pointsCnt; i++)
    {
      a = hypot(p1->y - cogXY.y, p1->x - cogXY.x);
      if (a > cogXYmaxDist)
        cogXYmaxDist = a;
      p1++;
    }
    cogXYvalid = true;
  }
  return cogXY;
}

///////////////////////////////////////////////////////////////

double UPolygon::getXYarea()
{
  double aSum = 0.0;
  int i;
  UPosition *p1, *p2;
  //
  if (pointsCnt > 2)
  { // a triangle or more complex
    p1 = &points[0];
    p2 = p1 + 1;
    for (i = 1; i < pointsCnt; i++)
    {
      aSum += p1->x * p2->y - p2->x * p1->y;
      p1 = p2;
      p2++;
    }
    // and the first point too, to close the polygon
    p2 = &points[0];
    aSum += p1->x * p2->y - p2->x * p1->y;
    aSum /= 2.0;
  }
  return aSum;
}

////////////////////////////////////////////

ULineSegment UPolygon::getSegment(int index)
{
  ULineSegment result;
  int next;
  //
  if (index >= pointsCnt)
    // prevent
    index = pointsCnt - 1;
  else if (index < 0)
    index = 0;
  // get next point
  next = index + 1;
  if (next >= pointsCnt)
    next = 0;
  // set segment from points
  result.setFromPoints(points[index], points[next]);
  //
  return result;
}

////////////////////////////////////////////

double UPolygon::getCogXYmaxDist()
{
  if (not cogXYvalid)
    getCogXY();
  return cogXYmaxDist;
}

/////////////////////////////////////////////////////////

int UPolygon::getCloseVertexCnt(double dist)
{
  int i, n = 0;
  UPosition * p1, *p2;
  double d;
  //
  p1 = points;
  p2 = p1++;
  for (i = 0; i < pointsCnt; i++)
  {
    d = hypot(p1->y - p2->y, p1->x - p2->x);
    if (d < dist)
      n++;
    p1++;
    if ((p1 - points) >= pointsCnt)
      p1 = points;
    p2++;
  }
  return n;
}

/////////////////////////////////////////////////////////

int UPolygon::removeNearVertex(double dist)
{
  int i, n = 0, result = 0;
  UPosition * p1, *p2;
  double d;
  bool last = false;
  //
  p1 = points; // foremost
  p2 = p1++;   // last
  for (i = 0; i < pointsCnt; i++)
  {
    d = hypot(p1->y - p2->y, p1->x - p2->x);
    if (d > dist)
    { // not removed
      p2++;
      if ((n++ != i) and not last)
        // copy value to not used index
        *p2 = *p1;
    }
    // test next value
    p1++;
    if ((p1 - points) >= pointsCnt)
    { // compare with
      p1 = points;
      last = true;
    }
  }
  if ((n != pointsCnt) and (n > 1))
  {
    result = pointsCnt - n;
    pointsCnt = n;
    setCogXYvalid(false);
  }
  return result;
}

///////////////////////////////////////////

bool UPolygon::isEmbedded(UPolygon * other, double * maxDist)
{
  int i;
  double dMax = -1e12, d;
  UPosition * p1;
  //
  p1 = other->getPoints();
  for (i = 0; i < other->getPointsCnt(); i++)
  {
    d = getDistance(p1->x, p1->y);
    if (i == 0)
      dMax = d;
    else
    {
      if (d > dMax)
        dMax = d;
    }
    p1++;
  }
  if (maxDist != NULL)
    *maxDist = dMax;
  return (dMax <= 0.0);
}



// bool UProbygon::moveToPose(UPose poseNow, UPose poseNew)
// {
//   bool res = true;
//   int n;
//   UPosition * pnt = points;
//   // new pose seen from present position
//   UPose dp = poseNew - poseNow;
//   //
//   if ((absd(dp.x) + absd(dp.y) + absd(dp.h)) > 0.001)
//   { // act only if robot has mooved
//     for (n = 0; n < pointsCnt; n++)
//     { // convert points as seen from new position
//       *pnt = dp.getMapToPose(*pnt);
//       pnt++;
//     }
//   }
//   //
//   return res;
// }

///////////////////////////////////////////

double UPolygon::getClosestDistance(double toX, double toY, double maxRelevantDist, UPosition * closest, bool * atVertex)
{
  double d, t;
  UPosition pos, to;
  int side, vertex = 0;
  ULineSegment seg;
  //
  pos = getCogXY();
  d = hypot(pos.x - toX, pos.y - toY);
  if ((d - getCogXYmaxDist()) < maxRelevantDist)
  { // there may be a point on polygon that is closer then maxRelevantDist
    d = getDistance(toX, toY, &side, &vertex);
    if (d < maxRelevantDist and closest != NULL)
    { // find position also
      if (vertex >= 0)
        *closest = getPoint(vertex);
      else
      { // closest to a side
        to.set(toX, toY, 0.0);
        seg = getSegment(side);
        t = seg.getPositionOnLine(to);
        *closest = seg.getPositionOnLine(t);
      }
      if (atVertex != NULL)
        *atVertex = (vertex >= 0);
    }
  }
  //
  return d;
}

///////////////////////////////////////////////

char * UPolygon::codeXml(char * buf, const int bufCnt, const char * extraAttr)
{
  char * p1;
  int i, n;
  const char * TAG_NAME = "polygon";
  const char * ea = "";
  //
  if (extraAttr != NULL)
    ea = extraAttr;
  snprintf(buf, bufCnt, "<%s cnt=\"%d\" %s>\n", TAG_NAME, pointsCnt, ea);
  n = strlen(buf);
  p1 = &buf[n];
  //
  for (i = 0; i < pointsCnt; i++)
  {
    points[i].codeXml(p1, bufCnt - n, NULL);
    n += strlen(p1);
    p1 = &buf[n];
  }
  // end tag
  snprintf(p1, bufCnt - n, "</%s>\n", TAG_NAME);
  //
  return buf;
}

/////////////////////////////////////////////////////////

bool UPolygon::cut(U2Dlined * line, UPolygon * source, UPolygon * dstL, UPolygon * dstR)
{
  bool result = false;
  int i, n;
  double d, d2;
  UPosition p1, p2, p3;
  U2Dlined l2;
  UPolygon * src = source;
  //
  n = src->getPointsCnt();
  if (src == dstL or src == dstR)
  { // a copy of source is needed
    if (n > 40)
      src = new UPolygon400();
    else
      src = new UPolygon40();
    source->copyTo(src);
  }
  if (dstL != NULL)
  {
    dstL->clear();
    dstL->setAsPolygon(src->isPolygon());
  }
  if (dstR != NULL)
  {
    dstR->clear();
    dstR->setAsPolygon(src->isPolygon());
  }
  d2 = 0.0;
  p2.clear();
  p3.z = 0.0;
  if (src->isPolygon())
    n++;
  for (i = 0; i < n; i++)
  {
    if (i < src->getPointsCnt())
      p1 = src->getPoint(i);
    else
      p1 = src->getPoint(0);
    d = line->distanceSigned(p1.x, p1.y);
    if (i > 0 and d * d2 <= 0.0 and d2 != 0.0)
    { // sign change - add crossing point
      l2.set2P(p2.x, p2.y, p1.x, p1.y);
      line->getCrossing(l2, &p3.x, &p3.y);
      if (dstL != NULL)
        dstL->add(p3);
      if (dstR != NULL)
        dstR->add(p3);
      result = true;
    }
    if (d > 0.0 and dstL != NULL)
      dstL->add(p1);
    if (d < 0.0 and dstR != NULL)
      dstR->add(p1);
    p2 = p1;
    d2 = d;
  }
  if (src != source)
    delete src;
  return result;
}

///////////////////////////////////////////////////////

int UPolygon::cutPoints(U2Dlined * line, U2Dpos * xPnts, int xPntsCnt )
{
  int i, n, cnt = 0;
  double d, d2;
  UPosition p1, p2, p3;
  U2Dlined l2;
  //
  n = getPointsCnt();
  d2 = 0.0;
  p2.clear();
  p3.z = 0.0;
  if (isPolygon())
    n++;
  for (i = 0; i < n; i++)
  {
    if (i < getPointsCnt())
      p1 = getPoint(i);
    else
      p1 = getPoint(0);
    d = line->distanceSigned(p1.x, p1.y);
    if (i > 0 and d * d2 <= 0.0 and d2 != 0.0)
    { // sign change - add crossing point
      l2.set2P(p2.x, p2.y, p1.x, p1.y);
      line->getCrossing(l2, &p3.x, &p3.y);
      if (cnt <= xPntsCnt)
        xPnts[cnt] = p3;
      cnt++;
    }
    p2 = p1;
    d2 = d;
  }
  return cnt;
}

////////////////////////////////////////////////////////

char * UPolygon::codeXml(const char * name, char * buf, int bufCnt,
                            const char * extra)
{
  char * p1;
  int i, n;
  const char * TAG_NAME = "polygon";
  UPosition pos;
  //
  snprintf(buf, bufCnt, "<%s", TAG_NAME);
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
    { // include a name
      snprintf(p1, bufCnt - n, " name=\"%s\"", name);
      n += strlen(p1);
      p1 = &buf[n];
    }
  if (extra != NULL)
  { // there may be extra attributes -- add these
    snprintf(p1, bufCnt - n, " %s", extra);
    n += strlen(p1);
    p1 = &buf[n];
  }
  // and closed, color and count of points
  snprintf(p1, bufCnt - n, " closed=\"%s\" color=\"%s\" cnt=\"%d\">\n",
           bool2str(isPolygon()), color, getPointsCnt());
  n += strlen(p1);
  p1 = &buf[n];

  for (i = 0; i < getPointsCnt(); i++)
  {
    pos = getPoint(i);
    pos.codeXml(p1, bufCnt - n, NULL);
    n += strlen(p1);
    p1 = &buf[n];
  }
  // end tag
  snprintf(p1, bufCnt - n, "</%s>\n", TAG_NAME);
  //
  return buf;
}

/////////////////////////////////////////////////////////

bool UPolygon::toConvex()
{
  UPolygon40 po40;
  bool result;
  result = extractConvexTo(&po40);
  po40.copyTo(this);
  return result;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void UPolygon40::sortByAngleXYTo(UPosition * pc)
{
  double angData[MAX_POINTS];
  double * pAngData[MAX_POINTS];
  double * a = angData;
  int i;
  unsigned long  j;
  UPosition * pos = points;
  //
  ppointsCnt = 0;
  for (i = 0; i < pointsCnt; i++)
  { // calculate angles
    pAngData[ppointsCnt++] = a;
    *a = atan2(pos->y - pc->y, pos->x - pc->x);
    a++;
    pos++;
  }
  qsort(pAngData, pointsCnt, sizeof(void*), angleSort);
  //
  // arrange pointers to UPosition (pdata) in the same way
  a = angData;
  ppointsCnt = 0;
  for (i = 0; i < pointsCnt; i++)
  { // get index to element i
    j = (pAngData[i] - a); // / sizeof(double);
    // index to element i is j
    pos = &points[j];
    if (pos != pc)
      ppoints[ppointsCnt++] = &points[j];
  }
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void UPolygon400::sortByAngleXYTo(UPosition * pc)
{
  double angData[MAX_POINTS];
  double * pAngData[MAX_POINTS];
  double * a = angData;
  int i;
  unsigned long  j;
  UPosition * pos = points;
  //
  ppointsCnt = 0;
  for (i = 0; i < pointsCnt; i++)
  { // calculate angles
    pAngData[ppointsCnt++] = a;
    *a = atan2(pos->y - pc->y, pos->x - pc->x);
    a++;
    pos++;
  }
  qsort(pAngData, pointsCnt, sizeof(void*), angleSort);
  //
  // arrange pointers to UPosition (pdata) in the same way
  a = angData;
  ppointsCnt = 0;
  for (i = 0; i < pointsCnt; i++)
  { // get index to element i
    j = (pAngData[i] - a); // / sizeof(double);
    // index to element i is j
    pos = &points[j];
    if (pos != pc)
      ppoints[ppointsCnt++] = &points[j];
  }
}



