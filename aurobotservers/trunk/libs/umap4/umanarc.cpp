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
#include "umanarc.h"

UManArc::UManArc()
 : UManoeuvre()
{
  manType = MAN_ARC;
}

////////////////////////////////////////////

UManArc::~UManArc()
{
}

////////////////////////////////////////////

double UManArc::getDistance()
{
  return absd(angle * radius);
}

////////////////////////////////////////////

// double UManArc::getManTime(double startVel)
// { //
//   // s = vt + �at�
//   // �at� + vt - s = 0
//   // t = (-v +- sqrt(v� + 2as))/a
//   // t = (sqrt(v�+2as) - v)/a
//   double result;
//   double dist;
//   double d;
//   //
//   dist = getDistance();
//   if (absd(acc) > 1e-10)
//   {
//     // acceleration is valid
//     d = sqr(startVel) + 2.0 * acc * dist;
//     if (d > 0.0)
//       result = (sqrt(sqr(startVel) + 2.0 * acc * dist) - startVel)/acc;
//     else
//       // NB! if direction changes this is not right, and can not be
//       // calculated, as distance traveled is not correct
//       result = absd(startVel / acc);
//   }
//   else if (absf(startVel) > 1e-5)
//     // no acceleration, but speed is non-zero
//     result = dist/startVel;
//   else
//     // no speed, no acceleration, so return a high value
//     result = 1e10;
//   return result;
// }

////////////////////////////////////////

UPose UManArc::getEndPose()
{ // relative end pose
  UPose result;
  //
  result.h = angle;
  result.x = radius * sin(absd(angle));
  result.y = radius * (1.0 - cos(angle));
  if (angle < 0.0)
    result.y *= -1.0;
  if (vel < 0.0)
  { // reverse, maintaining steering angle
    // changes angle and y-value only
    result.h *= -1.0;
    result.x *= -1.0;
  }
  return result;
}

////////////////////////////////////////

UPoseV UManArc::getEndPoseV(UPoseV startPoseV)
{
  UPoseV result;
  //
  result = startPoseV + getEndPose();
  result.setVel(vel);
  return result;
}

////////////////////////////////////////

UPoseV UManArc::getPoseV(double atManTime, UPoseV startPoseV, double endV)
{
  UPoseV result;
  UManArc sa;
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
  // make a copy of this manoeuvre
  sa = *this;
  // set turn angle
  sa.setTurnAngle(s / radius);
  // get end-pose by adding to start pose
  result = startPoseV + sa.getEndPose();
  //
  return result;
}

////////////////////////////////////////

void UManArc::fprint(FILE * fd, const char * prestr, double * v)
{
  double t;
  if (v != NULL)
  {
    t = getManTime(*v);
    *v = getEndV(*v);
    fprintf(fd, "%s r=%.2fm, arc=%.3frad, ev=%.2fm/s, tgt=%.2fm/s, acc=%.3fm/s2, dt=%.2fs\n",
           prestr, radius, angle, *v, vel, acc, t);
  }
  else
    fprintf(fd, "%s r=%.2fm, arc=%.3frad, tgt=%.2fm/s, acc=%.3fm/s2\n",
           prestr, radius, angle, vel, acc);
}

//////////////////////////////////////////

const char * UManArc::print(const char * prestr, double * v, char * buff, const int buffCnt)
{
  double t;
  if (v != NULL)
  {
    t = getManTime(*v);
    *v = getEndV(*v);
    snprintf(buff, buffCnt, "%s r=%.2fm, arc=%.3frad, ev=%.2fm/s, tgt=%.2fm/s, acc=%.3fm/s2, dt=%.2fs\n",
            prestr, radius, angle, *v, vel, acc, t);
  }
  else
    snprintf(buff, buffCnt, "%s r=%.2fm, arc=%.3frad, tgt=%.2fm/s, acc=%.3fm/s2\n",
            prestr, radius, angle, vel, acc);
  return buff;
}

///////////////////////////////////////////////

double UManArc::getDistanceXYSigned(UPosition pos, int * where,
                                    bool posIsRight,
                                    bool centerOnly,
                                    UPose * pHit,
                                    double * atT)
{
  double cy; // center
  double rx, ry;
  double d, dist = 0.0;
  double ang, aa, ea;
  int w = 3;
  const double halfPi = M_PI / 2.0;
  const double twoPi = M_PI * 2.0;
  //
  // find arc center
  if (angle > 0)
    cy = radius;
  else
    cy = -radius;
  ang = atan2(pos.y - cy, pos.x);
  // convert to angle relative to arc
  if (angle < 0)
    ang = halfPi - ang;
  else
    ang = halfPi + ang;
  // make all angles positive
  if (ang < 0.0)
    ang += twoPi;
  aa = absd(angle);
  // distance from turn center
  d = sqrt(sqr(pos.x) + sqr(pos.y - cy));
  // test for within arc - outside or inside
  if ((ang >= 0) and (ang <= aa))
  { // positive is to the right
    dist = (d - radius) * signofd(angle);
    w = 0;
    if (pHit != NULL)
    { // calculate hit position too.
      pHit->x = radius * sin(ang);
      if (angle < 0.0)
        pHit->y = cy + radius * cos(ang);
      else
        pHit->y = cy - radius * cos(ang);
      pHit->h = ang;
    }
    if (atT != NULL)
      // return distance into manoeuvre
      *atT = ang * radius;
  }
  else if (not centerOnly)
  { // closer to one of ends
    if ((ang - twoPi) < (aa - ang))
    { // high end is closer
      rx = radius * sin(aa);
      if (angle < 0.0)
        ry = cy + radius * cos(aa);
      else
        ry = cy - radius * cos(aa);
      dist = sqrt(sqr(pos.x - rx) + sqr(pos.y - ry));
      w = 2;
      if (pHit != NULL)
        *pHit = getEndPose();
      // get side of exit direction
      // get angle from exit position to position
      ea = atan2(pos.y - ry, pos.x - rx);
      if (ea > angle)
        // position is to the left of exit direction
        dist = -dist;
    }
    else
    { // distance from (0.0, 0.0)
      dist = sqrt(sqr(pos.x) + sqr(pos.y));
      w = 1;
      if (pHit != NULL)
        // hitpoint is 0,0
        pHit->set(0.0, 0.0, 0.0);
      if (pos.y > 0.0)
        // to the left of entry direction - is negative
        dist = -dist;
    }
  }
  if (where != NULL)
    *where = w;
  if (not posIsRight)
    dist = -dist;
  return dist;
}

////////////////////////////////////////////////////

double UManArc::getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                UPosition * posOnSeg,
                                bool posIsRight,
                                int * whereOnMan,
                                UPose * poseOnMan)
{
  int wS = -1, wM = -1, w1, w2;
//  double d11, d12, d21, d22;
  double cy, dc, at = 0.0;
  UPosition p1, p2, pc, ps, p3, pa2, pa1;
  double t, t2, dist = 0.0;
  double s1a, s2a; // angle to segment ends
  double da1s1, da1s2, da1s, da2s;
  double minA, maxA;
  UPose poseA;
  //
  p1 = seg->pos;
  p2 = seg->getOtherEnd();
  pa1.set(0.0, 0.0, 0.0); // start position on arc
  if (angle > 0)
  {
    cy = radius;
    minA = -M_PI / 2.0;
    maxA = angle - M_PI / 2.0;
    pa2.x = sin(angle) * radius; // end position on arc
    pa2.y = radius - cos(angle) * radius;
  }
  else
  {
    cy = -radius;
    minA = angle + M_PI / 2.0;
    maxA = M_PI / 2.0;
    pa2.x = -sin(angle) * radius; // end position on arc
    pa2.y = cos(angle) * radius - radius;
  }
  pc.set(0.0, cy, 0.0);
  dc = fabs(seg->getDistanceXYSigned(pc, &w1));
  if (dc > radius)
  {
    if (w1 == 1)
      p3 = p1;
    else if (w1 == 2)
      p3 = p2;
    else
    {
      t = seg->getPositionOnLine(pc);
      p3 = seg->getPositionOnLine(t);
    }
    at = atan2(p3.y - cy, p3.x);
    if ((at > minA) and (at < maxA))
    { // and is inside arc at angle at
      wM = 0;
      wS = w1;
      ps = p3;
      dist = dc - radius; // dist positive outside
    }
  }
  else
  { // at least one end of segment inside arc circle
    // test for actual crossing
    p3.set(0.0, cy, 0.0);
    seg->getCylinderCrossings(p3, radius, &t, &t2);
    wS = 0; // potential crossing
    if ((t >= 0.0) and (t <= seg->length))
      p3 = seg->getPositionOnLine(t);
    else if ((t2 >= 0.0) and (t2 <= seg->length))
      p3 = seg->getPositionOnLine(t2);
    else
      wS = -1; // failed
    if (wS == 0)
    { // valid if on arc
      at = atan2(p3.y - cy, p3.x);
      if ((at > minA) and (at < maxA))
      { // save result
        wM = 0;
        ps = p3;
        dist = 0.0;
      }
      else
        // failed
        wS = -1;
    }
  }
  if (wM == -1)
  { // no solution found so far.
    // distance from both ends to segment
    da1s = fabs(seg->getDistanceXYSigned(pa1, &w1));
    da2s = fabs(seg->getDistanceXYSigned(pa2, &w2));
    // is first end of segment within arc limits
    s1a = atan2(p1.y - cy, p1.x);
    if ((s1a >= minA) and (s1a <= maxA))
    { // is it closer then one of the arc ends?
      da1s1 = hypot(p1.y - cy, p1.x) - radius;
      if ((fabs(da1s1) < da1s) and (fabs(da1s1) < da2s))
      { // yes, this is the best candidate so far
        wM = 0;
        at = s1a;
        wS = 1;
      }
    }
    else
      da1s1 = da1s; // set to large value
    // try other end of segment, inside arc limits?
    s2a = atan2(p2.y - cy, p2.x);
    if ((s2a >= minA) and (s2a <= maxA))
    { // and is it closer then arc ends and other segment end?
      da1s2 = hypot(p2.y - cy, p2.x) - radius;
      if ((fabs(da1s2) < da1s) and
           (fabs(da1s2) < da2s) and
           (fabs(da1s2) < fabs(da1s1)))
      { // this is the closest point
        wM = 0;
        at = s2a;
        wS = 2;
      }
    }
    if (wM == -1)
    { // one of the arc ends are closer
      if (da1s < da2s)
      {
        wM = 1;
        p3 = pa1;
        wS = w1;
      }
      else
      {
        p3 = pa2;
        wM = 2;
        wS = w2;
      }
    }
    if (wS == 0)
    { // get hitpoint on segment
      t = seg->getPositionOnLine(p3);
      ps = seg->getPositionOnLine(t);
    }
    else if (wS == 1)
      ps = p1;
    else if (wS == 2)
      ps = p2;
    else
      printf("No way to get here (1)\n");
    if (wM == 0)
      dist = hypot(ps.y - cy, ps.x) - radius;
    else if (wM == 1)
    {
      dist = da1s;
      if ((ps.y * signofd(angle)) > 0.0)
        dist = -dist;
    }
    else if (wM == 2)
    {
      dist = da2s;
      s2a = atan2(ps.y - pa2.y, ps.x - pa2.x);
      if (((s2a > angle) and (angle > 0.0)) or
          ((s2a < angle) and (angle < 0.0)))
        dist = -dist;
    }
  }
  if (wM == 0)
  { // get hit position on arc
    // convert angle to manoeuvre angle
    poseA.x = cos(at) * radius;
    poseA.y = cy + sin(at) * radius;
    if (angle > 0.0)
      poseA.h = at + M_PI / 2.0;
    else
      poseA.h = at - M_PI / 2.0;
  }
  else if (wM == 1)
    poseA.set(pa1.x, pa1.y, 0.0);
  else if (wM == 2)
    poseA.set(pa2.x, pa2.y, angle);
  else
    printf("No way to get here (2)\n");
  // return results
  if (angle < 0.0)
    dist = -dist;
  if (not posIsRight)
    dist = -dist;
  if (whereOnSeg != NULL)
    *whereOnSeg = wS;
  if (whereOnMan != NULL)
    *whereOnMan = wM;
  if (posOnSeg != NULL)
    *posOnSeg = ps;
  if (poseOnMan != NULL)
    *poseOnMan = poseA;
  return dist;
}

///////////////////////////////////////////////////////

bool  UManArc::getSMRCLcmd2(char * buf, int bufCnt,
                            UPoseV * startPose,
                            bool * first, double firstDistance,
                            int * lineCnt, int lineCntMax,
                            double * distSum, double distSumMax,
                            UTime * t, FILE * logprim)
{
  int i;
  UPoseV pv, pv2;
  double v, d, dist;
  int n, m, np;
  double segmentSize = 2.0 * firstDistance; // [m]
  UManArc ma;
  char * p1;
  double h;
  bool result;
  //double dmax;
  double breakAt = distSumMax * 0.5;
  double ang, ang2, endAng, sa, ca;
  const double useTurnRradius = 6.0;
  //
  if (fabs(vel) < 0.15)
    v = 0.15;
  else
    v = vel;
  //
  //dmax = distSumMax - *distSum;
  if (not first or ((*distSum + getDistance()) > firstDistance))
  { // get command for arc (not skipped)
    p1 = buf;
    n = 0;
    if (radius < useTurnRradius)
    { // use turnr command type
      d = getDistance();
      np = 4;
      if (*distSum < breakAt and d + *distSum >= breakAt)
      {
        dist = breakAt - *distSum;
        // part of turn only, maintaining sign
        ang = angle * dist / d;
        endAng = startPose->h + ang;
        sa = sin(endAng);
        ca = cos(endAng);
        snprintf(p1, bufCnt - n, "turnr %.3f %.3f \"rad\" @v%.2f : "
          "(abs(%.4f - sin($odoth)) + abs(%.4f - cos($odoth)) < 0.1)\n\t\n",
                 radius, ang, v, sa, ca);
        (*distSum) += dist;
        (*lineCnt)++;
        n += strlen(p1);
        p1 = &buf[n];
        if (logprim != NULL)
        {
          ma.setTurnRadius(radius);
          ma.setTurnAngle(ang);
          pv = ma.getEndPoseV(*startPose);
          fprintf(logprim, "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
                  t->getSec(), t->getMicrosec(),
                            np,
                            startPose->x, startPose->y, startPose->h, startPose->vel,
                            pv.x, pv.y, pv.h, v);
          np += 100;
        }
      }
      else
      {
        ang = 0.0;
        dist = 0.0;
        pv = *startPose;
      }
      // remaining distance
      dist = fmin(distSumMax - *distSum, d - dist);
        // remaining part of turn, maintaining sign
      ang2 = angle * dist / d;
      // end angle to improve stop-condition
      endAng = startPose->h + ang + ang2;
      sa = sin(endAng);
      ca = cos(endAng);
      snprintf(p1, bufCnt - n, "turnr %.3f %.3f \"rad\" @v%.2f : "
          "(abs(%.4f - sin($odoth)) + abs(%.4f - cos($odoth)) < 0.1)",
          radius, ang2, v, sa, ca);
      n = strlen(p1);
      if (logprim != NULL)
      {
        ma.setTurnRadius(radius);
        ma.setTurnAngle(ang2);
        pv2 = ma.getEndPoseV(pv);
        fprintf(logprim, "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
                t->getSec(), t->getMicrosec(),
                          np,
                          pv.x, pv.y, pv.h, pv.vel,
                          pv2.x, pv2.y, pv2.h, v);
      }
      pv = getEndPoseV(*startPose);
      (*distSum) += dist;
      *startPose = pv;
      (*lineCnt)++;
      *first = false;
    }
    else
    { // segment arc into tangent lines
      m = roundi(getDistance() / segmentSize);
      ma.setTurnRadius(radius);
      ma.setTurnAngle(angle / double(m));
      pv2 = *startPose;
      pv = ma.getEndPoseV(*startPose);
      // debug
      // printf("UManArc::getSMRCLcmd2  ma.angle=%g ma.radius=%g",
      //         ma.angle, ma.radius);
      // // debug end
      np = 5;
      for (i = 0; i < m; i++)
      {
        h = pv.h * 180.0 / M_PI;
        while (h < 0.0)
          h += 360.0;
        snprintf(p1, bufCnt - n,
                "driveon %.3f %.3f %.2f @v%.2f @a%.2f :($targetdist < 0.0) \n",
                pv.x, pv.y, h, v, fabs(acc));
        n += strlen(p1);
        p1 = &buf[n];
        (*lineCnt)++;
        if (logprim != NULL)
        {
          fprintf(logprim, "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
                          t->getSec(), t->getMicrosec(),
                          np,
                          pv2.x, pv2.y, pv2.h, pv2.vel,
                          pv.x, pv.y, pv.h, v);
          pv2 = pv;
        }
        d = ma.getDistance();
        if ((*distSum < breakAt) and
            (d + *distSum) >= breakAt)
        { // mark place for event trigger
          snprintf(p1, bufCnt - n, "\t\n");
          n += strlen(p1);
          p1 = &buf[n];
          np += 100;
        }
        (*distSum) += d;
        *startPose = pv;
        if (*lineCnt >= lineCntMax)
          break;
        if (*distSum >= distSumMax)
          break;
        // advance to next end position
        pv = ma.getEndPoseV(pv);
      }
      // remove last '\n'
      if (n > 0)
        buf[n-1] = '\0';
      *first = false;
    }
  }
  else
  { // too small manoeuvre, or not suited fas first command,
    // so just update startpose and distSum
    pv = getEndPoseV(*startPose);
    (*distSum) += getDistance();
    *startPose = pv;
    n = 0;
  }
  result = (n < bufCnt);
  return result;
}

////////////////////////////////////////////////////

bool UManArc::getSMRCLcmd(char * buf, int bufCnt, double maxDist)
{
  int i, n, m;
  double d, dd, da;
  char * p1;
  bool result = true;
  double v;
  double dist;
  //
  // turn is not allowed at zero speed
  if (fabs(vel) < 0.2)
    v = 0.25;
  else
    v = vel;
  //
  if (radius < 3.0)
  {
    dist = mind(maxDist, getDistance());
    snprintf(buf, bufCnt, "turnr %.3f %.3f @v%.2f @a%.3f :($drivendist > %.2f)",
             maxd(radius, 0.15), // use at least 15cm turn radius
             angle * 180.0 / M_PI, v,
             maxd(getMinAcc(), fabs(acc)), dist);
    n = strlen(buf);
  }
  else
  { // it is too unsafe to turn and finish on the implicit angle criteria
    // divide into 30 cm steps
    m = maxi(1, roundi(fabs(angle) / 0.05)); // about 3 deg steps
    d = radius * angle;
    dd = d / double(m);
    da = angle / double(m);
    n = 0;
    p1 = buf;
    dist = mind(maxDist, getDistance());
    for (i = 0; i < m; i++)
    { // one command takes a little less than 60 characters.
      // so there need to be at least 60 characters left in buffer
      result = ((bufCnt - n) > 60);
      if (not result)
      { // no more space for next command
        printf("*** UManArc::getSMRCLcmd: no space left (%d left) in buffer (size %d) for next drive command\n", bufCnt, n);
        break;
      }
      // space for next command line
      snprintf(p1, bufCnt - n, "turnr %.3f %.3f \"rad\" @v%.2f @a%.3f :($drivendist > %.2f)\n",
               radius, da, v, maxd(getMinAcc(), fabs(acc)), dd);
      n += strlen(p1);
      p1 = &buf[n];
      dist -= dd;
      if (dist < 0.0)
        break;
    }
    // remove laset '\n'
    if (n > 0)
      buf[n-1] = '\0';
  }
  return (n < bufCnt);
}

////////////////////////////////////////

UPosition UManArc::getTurnCentre(UPose startPose)
{
  UPosition result;
  //
  if (angle > 0)
  { // turning left
    result.x = startPose.x - radius * sin(startPose.h);
    result.y = startPose.y + radius * cos(startPose.h);
  }
  else
  { // turning right
    result.x = startPose.x + radius * sin(startPose.h);
    result.y = startPose.y - radius * cos(startPose.h);
  }
  return result;
}

///////////////////////////////////////////

bool UManArc::getFoodprintPolygon(UPolygon * polyIncl,
                                 UPolygon * polyExcl,
                                 double leftX, double leftY,
                                 double rightX, double rightY,
                                double clearence,
                                 UPose * startPose, double allowedErr)
{
  bool result = polyIncl != NULL;
  double di, x, y, x1 = 0.0, y1 = 0.0, x2, y2, d, d2;
  UPosition p1;
  double our; // outher radius
  int i, n;
  bool leftTurn = angle > 0.0;
  U2Dlined l1, l2;
  //
  polyIncl->clear();
  // outher turn radius for most extreme point
  our = hypot(-rightY + radius, rightX) + clearence + allowedErr;
  //  increment in radians
  di = acos(1.0 - allowedErr / our * 2.0) * 2.0;
  n = int(fabs(angle) / di) + 1;
  di = angle / double(n);
  // we need to take the polygon in the CCV order.
  if (leftTurn)
  {
    // start angle for outher turn circle, where robot x is opposing side
    d = atan2(rightX, radius - rightY); // rightY is negative
    d += di;
  }
  else
  { // positive start angle for outher circle
    d = atan2(leftX, radius + leftY);
    d -= angle;  // d will be positive with the full turn angle more than the start angle
    // di will be negative;
  }
  for (i = 0; i < n; i++)
  { // now the outher arc
    x = sin(d) * our;
    y = radius - cos(d) * our;
    if (not leftTurn)
      y = -y;
    p1 = startPose->getPoseToMap(x, y);
    polyIncl->add(p1);
    d += di;
  }
  if (leftTurn)
  { // then to the inner side of robot front after manoeuvre
    // first including front clearence
    x2 =           sin(angle) * (radius - leftY) + sin(angle + M_PI / 2.0) * (leftX + clearence);
    y2 = radius - (cos(angle) * (radius - leftY) + cos(angle + M_PI / 2.0) * (leftX + clearence));
    p1 = startPose->getPoseToMap(x2, y2);
    if (angle > M_PI)
      // this point is possibly inside envelope - and thus not visible
      d2 = polyIncl->getDistance(p1.x, p1.y);
    else
      d2 = 1.0;
    if (d2 > 0.0)
      polyIncl->add(p1);
    // then including side clearence
    x1 =           sin(angle) * (radius - leftY - clearence) + sin(angle + M_PI / 2.0) * leftX;
    y1 = radius - (cos(angle) * (radius - leftY - clearence) + cos(angle + M_PI / 2.0) * leftX);
    p1 = startPose->getPoseToMap(x1, y1);
    if (angle > M_PI)
      d2 = polyIncl->getDistance(p1.x, p1.y);
    else
      d2 = 1.0;
    if (d2 > 0.0)
    {
      polyIncl->add(p1);
      // we may need back-left corner in start position
      if (x1 > 0.0)
        polyIncl->add(startPose->getPoseToMap(0.0, leftY + clearence));
    }
    // then back right in start position
    p1 = startPose->getPoseToMap(0.0, rightY - clearence);
    if (angle > M_PI * 1.5)
      d2 = polyIncl->getDistance(p1.x, p1.y);
    else
      d2 = 1.0;
    if (d2 > 0.0)
      // if almost 360 deg turn
      polyIncl->add(p1);
    // and front right in start position
    polyIncl->add(startPose->getPoseToMap(rightX, rightY - clearence));
  }
  else
  { // and outher front in start position
    //polyIncl->add(startPose->getPoseToMap(leftX + clearence, leftY));
    polyIncl->add(startPose->getPoseToMap(leftX, leftY + clearence));
    // and the the outher back end of robot in start position
    p1 = startPose->getPoseToMap(0.0, leftY + clearence);
    if (fabs(angle) > M_PI)
      d2 = polyIncl->getDistance(p1.x, p1.y);
    else
      d2 = 1.0;
    if (d2 > 0.0)
    {
      polyIncl->add(p1);
      // then to the inner side of robot front  after manoeuvre
      x1 =           sin(-angle) * (radius + rightY - clearence) + sin(-angle + M_PI / 2.0) * rightX;
      y1 = -radius + (cos(-angle) * (radius + rightY - clearence) + cos(-angle + M_PI / 2.0) * rightX);
      x2 =           sin(-angle) * (radius + rightY) + sin(-angle + M_PI / 2.0) * (rightX + clearence);
      y2 = -radius + (cos(-angle) * (radius + rightY) + cos(-angle + M_PI / 2.0) * (rightX + clearence));
      // but first the inner back end of robot at start position - if needed
      if (x1 > 0.0 and x2 > 0.0)
        polyIncl->add(startPose->getPoseToMap(0.0, rightY - clearence));
      p1 = startPose->getPoseToMap(x1, y1);
      if (fabs(angle) > M_PI)
        // this point is possibly inside envelope
        d2 = polyIncl->getDistance(p1.x, p1.y);
      else
        d2 = 1.0;
      if (d2 > 0.0)
        polyIncl->add(p1);
      p1 = startPose->getPoseToMap(x2, y2);
      if (fabs(angle) > M_PI)
        d2 = polyIncl->getDistance(p1.x, p1.y);
      else
        d2 = 1.0;
      if (d2 > 0.0)
        polyIncl->add(p1);
    }
  }
  //
  if (polyExcl != NULL)
  {
    polyExcl->clear();
    // x,y is relative position of front left corner after turn
    // this is always needed - used as start point
    p1 = startPose->getPoseToMap(x1, y1);
    polyExcl->add(p1);
    if (leftTurn)
    { // then if turn angle is more than 90 deg, then
      // we need a point where the line from x,y to back-right crosses
      // the left side of robot extended backwards
      if (y1 < leftY + clearence)
        // point (inner front after turn) is direct behind robot, and fair to use directly
        p1 = startPose->getPoseToMap(x1, y1);
      else if (angle > M_PI / 2.0)
      { // set up lines
        l1.set2P(x1, y1, 0.0, rightY - clearence);
        l2.set2P(0.0, leftY + clearence, leftX, leftY + clearence);
        l1.getCrossing(l2, &x, &y);
        // get crossing in real coordinates
        p1 = startPose->getPoseToMap(x, y);
      }
      else
        p1 = startPose->getPoseToMap(0.0, leftY + clearence);
      polyExcl->add(p1);
      // now the inner radius
      our = radius - leftY + allowedErr - clearence;
    }
    else
    { //
      polyExcl->add(p1);
      our = radius + rightY + allowedErr - clearence;
      // we may need this line back to right corner, if
      // the turn angle is > 90deg
      l1.set2P(x1, y1, 0.0, leftY + clearence);
    }
    // and a new interval
    di = acos(1.0 - allowedErr / our  * 2.0) * 2.0;
    // and number of points rounded to fit angle
    n = int(fabs(angle) / di) + 1;
    di = angle / double(n);
    // start after first interval
    if (leftTurn)
      // start after first interval
      d = di;
    else
      // start at full angle (positive)
      d = -angle;
    for (i = 0; i < n; i++)
    { // now the arc in CCV order
      x = sin(d) * our;
      y = radius - cos(d) * our;
      if (not leftTurn)
        y = -y;
      p1 = startPose->getPoseToMap(x, y);
      polyExcl->add(p1);
      d += di;
    }
    if (not leftTurn)
    { // we need the last point at inner side of robot back
      if (y1 > rightY - clearence)
        // point (inner front after turn) is directly behind robot - fairly safe to use
        p1 = startPose->getPoseToMap(x1, y1);
      else if (fabs(angle) > M_PI / 2.0)
      {
        l2.set2P(0.0, rightY - clearence, rightX, rightY - clearence);
        l1.getCrossing(l2, &x, &y);
        p1 = startPose->getPoseToMap(x, y);
      }
      else
        p1 = startPose->getPoseToMap(0.0, rightY - clearence);
      polyExcl->add(p1);
    }
  }
  //
  return result;
}

