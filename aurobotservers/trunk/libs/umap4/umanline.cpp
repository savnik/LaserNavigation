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

//#include <unistd.h>

#include "umanline.h"


UManLine::UManLine()
 : UManoeuvre()
{
  manType = MAN_LINE;
}

///////////////////////////////////

UManLine::~UManLine()
{
}

///////////////////////////////////

// double UManLine::getManTime(double startVel)
// { //
//   double ta;
//   // s = vt + �at�
//   // �at� + vt - s = 0
//   // t = (-v +- sqrt(v� + 2as))/a
//   // t = (sqrt(v�+2as) - v)/a
//   double result;
//   if (absd(acc) > 1e-10)
//   { // tome to accelerate
//     ta = absd(vel - vstart)/absd(acc);
//     sa = startV * ta + 0.5 * acc * sqr(ta);
//     if (sa > getDistance())
//       // acceleration based time only
//       result = (sqrt(maxd(0.0, sqr(startVel) + 2.0 * acc * dist)) - startVel)/acc;
//     else if (absd(vel) > 1e-3)
//       // full acceleration plus constant speed at rest
//       result = ta + (getDistance() - sa) / vel;
//     else
//       // long time as velocity is zero
//       result = 1e5;
//   }
//   else if (absf(startVel) > 1e-5)
//     // speed is non-zero
//     result = dist/startVel;
//   else
//     // no speed, so return a high value
//     result = 1e10;
//   return result;
// }

////////////////////////////////////////

UPose UManLine::getEndPose()
{ // relative end pose
  UPose result(dist, 0.0, 0.0);
  return result;
}

////////////////////////////////////////

UPoseV UManLine::getEndPoseV(UPoseV startPoseV)
{
  UPoseV result;
  //
  result = startPoseV + getEndPose();
  result.setVel(getEndV(startPoseV));
  //
  return result;
}

////////////////////////////////////////

UPoseV UManLine::getPoseV(double atManTime, UPoseV startPoseV, double endV)
{
  UPoseV result;
  UPose rel;
  double s, ta, tv;
  double v;
  //
  v = acc * sqr(atManTime) / 2.0;
  s = startPoseV.getVel() * atManTime;
  if ((v + startPoseV.getVel() * atManTime) > endV)
  { // accelerate part of the time only
    ta = fabs((endV - startPoseV.getVel())/acc);
    tv = atManTime - ta;
    s += acc * sqr(ta) / 2.0 + tv * (endV - startPoseV.getVel());
  }
  else
    s += acc * sqr(atManTime) / 2.0;
  //
  rel.set(s, 0.0, 0.0);
  result = startPoseV + rel;
  //
  return result;
}

////////////////////////////////////////

void UManLine::fprint(FILE * fd, const char * prestr, double * v)
{
  double t;
  if (v != NULL)
  {
    t = getManTime(*v);
    *v = getEndV(*v);
    fprintf(fd, "%s d=%.2fm, ev=%.2fm/s, tgt=%.2fm/s, acc=%.3fm/s2, dt=%.2fsec\n",
           prestr, dist, *v, vel, acc, t);
  }
  else
    fprintf(fd, "%s d=%.2fm, tgt=%.2fm/s, acc=%.3fm/s2\n",
         prestr, vel, acc, dist);
}

///////////////////////////////////////////////

const char * UManLine::print(const char * prestr, double * v, char * buff, const int buffCnt)
{
  double t;
  if (v != NULL)
  {
    t = getManTime(*v);
    *v = getEndV(*v);
    snprintf(buff, buffCnt, "%s d=%.2fm, ev=%.2fm/s, tgt=%.2fm/s, acc=%.3fm/s2, dt=%.2fsec\n",
            prestr, dist, *v, vel, acc, t);
  }
  else
    snprintf(buff, buffCnt, "%s d=%.2fm, tgt=%.2fm/s, acc=%.3fm/s2\n",
            prestr, vel, acc, dist);
  return buff;
}

///////////////////////////////////////////////

double UManLine::getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                UPosition * posOnSeg,
                                bool posIsRight,
                                int * whereOnMan,
                                UPose * poseOnMan)
{
  U2Dlined l1, l2;
  double d = 0.0, d2, d11, d12, d21, d22;
  UPose pm;
  int wS = -1, wM = -1;
  double x, y, t = 0.0;
  UPosition p1, p2, p3, ps;
  bool isOK;
  //
  p1 = seg->pos;
  p2 = seg->getOtherEnd();
  l1.set2P(p1.x, p1.y, p2.x, p2.y);
  l2.set2P(0.0, 0.0, dist, 0.0);
  isOK = l1.getCrossing(l2, &x, &y);
  pm.set(0.0, 0.0, 0.0);
  if (isOK)
  { // not parallel
    if ((x <= dist) and (x >= 0.0))
    {
      ps.set(x, y, 0.0);
      t = seg->getPositionOnLine(ps);
      if ((t <= seg->length) and (t >= 0.0))
      { // full crossing
        wM = 0;
        wS = 0;
        ps = seg->getPositionOnLine(t);
        pm.x = x;
        d = 0.0;
      }
    }
  }
  if ((wM == -1) and (wS == -1))
  { // not full crossing - get end
    // may be on streach of manoeuvre
    if ((p1.x <= dist) and (p1.x >= 0))
    { // yes, p1 is inside M range
      wM = 0;
      wS = 1;
      pm.x = p1.x;
      d = fabs(p1.y);
    }
    else
      d = 1e27;
    if ((p2.x <= dist) and (p2.x >= 0) and (d >= fabs(p2.y)))
    { // yes, p2 is inside M range
      wM = 0;
      wS = 2;
      pm.x = p2.x;
      d = fabs(p2.y);
    }
    // or may be on streach of segment
    p3.set(0.0, 0.0, 0.0);
    t = seg->getPositionOnLine(p3);
    if ((t >= 0.0) and (t < seg->length))
    { // near man-end (at 0,0) is inside segment
      p3 = seg->getPositionOnLine(t);
      d2 = hypot(p3.y, p3.x);
      if (d2 < d)
      {
        wM = 1;
        wS = 0;
        ps = p3;
        d = d2;
      }
    }
    p3.set(dist, 0.0, 0.0);
    t = seg->getPositionOnLine(p3);
    if ((t >= 0.0) and (t < seg->length))
    { // far man-end (at 0,dist) is inside segment
      p3 = seg->getPositionOnLine(t);
      d2 = hypot(p3.y, p3.x - dist);
      if (d2 < d)
      { // closer
        wM = 2;
        wS = 0;
        ps = p3;
        d = d2;
      }
    }
  }
  // end to end options
  // to end distances
  if ((wM == -1) and (wS == -1))
  { // dMS is M is a manoeuvre end and S is a segment end
    d11 = hypot(p1.x, p1.y);
    d12 = hypot(p2.x, p2.y);
    d21 = hypot(p1.x - dist, p1.y);
    d22 = hypot(p2.x - dist, p2.y);
    // wich is the closer at manoeuvre end
    if (mind(d11, d12) < mind(d21, d22))
      wM = 1; //  end at 0,0
    else
      wM = 2; // far end at o,dist
    // and wich is the closer of the segment end
    if (mind(d11, d21) < mind(d12, d22))
      wS = 1; // reference end (1)
    else
      wS = 2; // other end (2)
    // get distance (unsigned)
    if (wM == 1)
    { // from near manoeuvre end to segment
      if (wS == 1)
        d = d11;
      else if (wS == 2)
        d = d12;
    }
    else if (wM == 2)
    { // from far manoeuvre end to segment
      if (wS == 1)
        d = d21;
      else if (wS == 2)
        d = d22;
    }
  }
  // set endpoint hit position - man side
  if (wM == 1)
    pm.set(0.0, 0.0, 0.0);
  else if (wM == 2)
    pm.set(dist, 0.0, 0.0);
  // set endpoint hit position - segment side
  if (wS == 1)
    ps = p1;
  else if (wS == 2)
    ps = p2;
  // report results
  if (whereOnMan != NULL)
    *whereOnMan = wM;
  if (whereOnSeg != NULL)
    *whereOnSeg = wS;
  if (poseOnMan != 0)
    *poseOnMan = pm;
  if (posOnSeg != NULL)
    *posOnSeg = ps;
  // get sign in place - d is always positive
  // but should be negative, when near segment point
  // is negative
  if (ps.y < 0.0)
    d = -d;
  // or the other way if right is positive
  if (posIsRight)
    d = -d;
  return d;
}

/////////////////////////////////////////////////////

double UManLine::getDistanceXYSigned(UPosition pos, int * where,
                                           bool posIsRight,
                                           bool centerOnly,
                                           UPose * pHit,
                                           double * atT)
{
  double d = 0.0;
  //
  if (pos.x < 0.0)
  { // before start
    if (not centerOnly)
      d = sqrt(sqr(pos.x) + sqr(pos.y)) * signofd(pos.y);
    if (where != NULL)
      *where = 1;
    if (pHit != NULL)
      pHit->set(0.0, 0.0, 0.0);
  }
  else if (pos.x > dist)
  { // after end
    if (not centerOnly)
      d = sqrt(sqr(pos.x - dist) + sqr(pos.y)) * signofd(pos.y);
    if (where != NULL)
      *where = 2;
    if (pHit != NULL)
      pHit->set(dist, 0.0, 0.0);
  }
  else
  { // on line (center hit)
    d = pos.y;
    if (where != NULL)
      *where = 0;
    if (pHit != NULL)
      pHit->set(pos.x, 0.0, pos.z);
    if (atT != NULL)
      *atT = pos.x;
  }
  if (posIsRight)
    d = -d;
  return d;
}

////////////////////////////////////////////////////

bool UManLine::getSMRCLcmd(char * buf, int bufCnt)
{
  double d;
  if (vel < 0.0)
    d = - fabs(dist);
  else
    d = fabs(dist);
  snprintf(buf, bufCnt, "driveon @v%.2f @a%.3f :($drivendist > %f)",
           fabs(vel), maxd(getMinAcc(), fabs(acc)), d);
  return true;
}

////////////////////////////////////////////////////////

bool  UManLine::getSMRCLcmd2(char * buf, int bufCnt,
                            UPoseV * startPose,
                            bool * first, double firstDistance,
                            int * lineCnt, int lineCntMax,
                            double * distSum, double distSumMax,
                            UTime * t, FILE * logprim)
{
  UPoseV pv, pv2, pv3;
  double v;
  //double h;
  double dsum;
  char * p1;
  int n;
  double breakAt = distSumMax * 0.5;
  //
  if (fabs(vel) < 0.2)
    v = 0.25;
  else
    v = vel;
  dsum = *distSum + getDistance();
  //
  pv = getEndPoseV(*startPose);
  n = 0;
  p1 = buf;
  p1[0] = '\0';
  if (not *first or ((*distSum + getDistance()) > firstDistance))
  { // this is not too short for a first command
    if (*distSum < breakAt and dist + *distSum >= breakAt)
    { // order to break only
      pv2.set(breakAt - *distSum, 0.0, 0.0, 0.0);
      pv = *startPose + pv2;
      snprintf(p1, bufCnt - n, "driveon %.3f %.3f %.3f \"rad\" @v%.2f @a%.2f :($targetdist < 0.0)\n\t\n",
              pv.x, pv.y, pv.h, v, fabs(acc));
      n += strlen(p1);
      p1 = &buf[n];
      (*lineCnt)++;
      if (logprim != NULL)
      {
        fprintf(logprim, "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
                t->getSec(), t->getMicrosec(),
                          3,
                          startPose->x, startPose->y, startPose->h, startPose->vel,
                          pv.x, pv.y, pv.h, v);
      }
      // save start pose of next drive primitive
      pv3 = pv;
    }
    else
    {
      pv2.clear();
      // start pose of next primitive
      pv3 = *startPose;
    }
    // then the rest
    pv2.x = fmin(dist, distSumMax - *distSum);
    pv = *startPose + pv2;
    snprintf(p1, bufCnt - n, "driveon %.3f %.3f %.3f \"rad\" @v%.2f @a%.2f :($targetdist < 0.0)",
             pv.x, pv.y, pv.h, v, fabs(acc));
    n += strlen(p1);
    p1 = &buf[n];
    (*lineCnt)++;
    // first command is done
    *first = false;
    if (logprim != NULL)
    {
      fprintf(logprim, "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
              t->getSec(), t->getMicrosec(),
                        103,
                        pv3.x, pv3.y, pv3.h, pv3.vel,
                        pv.x, pv.y, pv.h, v);
    }
  }
  *distSum = dsum;
  *startPose = pv;
  return (n < bufCnt);
}

///////////////////////////////////////////////

/**
Get this manoeuvre as a polygin covering the robot foodprint
during the manoeuvre.
\param polyIncl is a polygon (convex) with sufficient corners for the manoeuvre.
\param polyExcl is a polygon (convex) covering the concavity of the
polyIncl that is not part of the manoeuvre footprint.
\param leftX is the forward distance of the front-left-most extreme point
of the robot (positive).
\param leftY is the x-coordinate of the position of the front-left-most
extreme point of the robot (should be positive).
\param rightX is the forward distance of the front-right-most extreme point
of the robot (positive).
\param rightY is the y-coordinate of the position of the front-right-most
extreme point of the robot (should be negative).
\param clearence is the minimum clearence from extreme points.
\param startPose is the start pose of the manoeuvre, indicating the coordinate system used.
\param allowedErr is the allowed deviation from true polygon in meters (must be > 0.0).
\returns true if polygon is valid. */
bool UManLine::getFoodprintPolygon(UPolygon * polyIncl,
                                 UPolygon * polyExcl,
                                 double leftX, double leftY,
                                 double rightX, double rightY,
                                  double clearence,
                                 UPose * startPose, double allowedErr)
{
  bool result = polyIncl != NULL;
  UPosition p1;
  //
  polyIncl->clear();
  p1 = startPose->getPoseToMap(0.0, rightY - clearence);
  polyIncl->add(p1);
  p1 = startPose->getPoseToMap(rightX + dist, rightY - clearence);
  polyIncl->add(p1);
  p1 = startPose->getPoseToMap(rightX + dist + clearence, rightY);
  polyIncl->add(p1);
  p1 = startPose->getPoseToMap(leftX + dist + clearence, leftY);
  polyIncl->add(p1);
  p1 = startPose->getPoseToMap(leftX + dist, leftY + clearence);
  polyIncl->add(p1);
  p1 = startPose->getPoseToMap(0.0, leftY + clearence);
  polyIncl->add(p1);
  //
  if (polyExcl != NULL)
    polyExcl->clear();
  //
  return result;
}

