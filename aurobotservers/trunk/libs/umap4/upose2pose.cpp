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
#include "upose2pose.h"
#include <ugen4/u2dline.h>

//////////////////////////////////////////////////

UPose2pose::UPose2pose()
 : UPoseV()
{
}

//////////////////////////////////////////////////

UPose2pose::UPose2pose(double ix, double iy, double ih, double iv)
  : UPoseV(ix, iy, ih, iv)
{
}

//////////////////////////////////////////////////

UPose2pose::~UPose2pose()
{
}

//////////////////////////////////////////////////

// bool UPose2pose::get2arcRight(UPose toPose,
//            double * r1, double * h1, double * h2)
// {
//   UPose p2 = toPose;
//   UMatrix4 mF(2,1); // function value (in U)
//   UMatrix4 mJ(2,2); // jacobian
//   UMatrix4 mJi(2,2); // inverse jacobian
//   UMatrix4 mU(2,1); // old values
//   UMatrix4 mUd(2,1); // new estimate delta
//   double j11, j12, j22, j21; // derivative in u0
//   double f1, f2; // function values at u0
//   double uP, uP2; // unknown 1. Phi2
//   double uR, uR2; // unknown 2. Turn radius (common)
//   double uP1;
//   int i;
//   const double ELIM = 0.0001;
//   bool result = false;
//   //
//   // print("P1");
//   //p2.print("p2");
//   //
//   uR = 3.0; // estimated radius
//   uP = 1.0;  // estimated angle for 2nd turn
//   // iterate
//   for (i = 0; i < 15; i++)
//   { // set function value at uR, uP
//     f1 = x - p2.x + uR * (sin(h) + sin(p2.h) -
//         2.0 * sin(p2.h - uP));
//     f2 = y - p2.y + uR * (2.0 * cos(p2.h - uP) -
//         cos(h) - cos(p2.h));
//     result = ((absd(f1) < ELIM) and (absd(f2) < ELIM) and
//              uP >= 0.0);
//     if (result)
//       break;
//     // set derivative of two variables at uR, uP
//     j11 = 2.0 * uR * cos(p2.h - uP);
//     j12 = sin(h) + sin(p2.h) - 2.0 * sin(p2.h - uP);
//     j21 = 2.0 * uR * sin(p2.h - uP);
//     j22 = 2.0 * cos(p2.h - uP) - cos(h) - cos(p2.h);
//     mF.setRC(0, 0, f1);
//     mF.setRC(1, 0, f2);
//     mJ.setRow(0, j11, j12);
//     mJ.setRow(1, j21, j22);
//     mJi = mJ.inversed();
//     mUd = mJi * mF;
//     uP2 = uP - mUd.get(0,0);
//     uR2 = uR - mUd.get(1,0);
//     // debug
//     printf("Phi: %7g rad ", uP2);
//     printf("Rad: %7g m (f1=%7g, f2=%7g)\n", uR2, f1, f2);
//     // debug end
//     uP = uP2;
//     uR = uR2;
//   }
//   while (uP > 2.0 * M_PI)
//     uP -= 2.0 * M_PI;
//   uP1 = h - p2.h + uP;
//   // debug
//   printf("Right R: R=%5.2f Phi1=%5.4f Phi2=%5.4f (f1=%7g, f2=%7g)\n",
//          uR, uP1, uP, f1, f2);
//   printf("Returned after %d loops\n", i);
//   // debug end
//   if (r1 != NULL)
//     *r1 = uR;
//   if (h1 != NULL)
//     *h1 = uP1;
//   if (h2 != NULL)
//     *h2 = uP;
//   //
//   return result;
// }

//////////////////////////////////////////////////

bool UPose2pose::get22arcLeft(double * r1, double * h1, double * h2, FILE * logFp)
{
  UPose2pose pMirror(x, -y, -h, vel);
  bool result;
  //
  result = pMirror.get2arcQ4(r1, h1, h2, logFp);
  return result;
}

//////////////////////////////////////////////////

bool UPose2pose::get22arcRight(double * r1, double * h1, double * h2, FILE * logFp)
{
  bool result;
  result = get2arcQ4(r1, h1, h2, logFp);
  return result;
}

//////////////////////////////////////////////////

bool UPose2pose::get2arcQ4(double * r1, double * h1, double * h2, FILE * logFp)
{ // find 2-arc solution starting in 4th quadrant
  // - start in a right turn forward, followed by a left turn forward)
  UMatrix4 mF(2,1); // function value (in U)
  UMatrix4 mJ(2,2); // jacobian
  UMatrix4 mJi(2,2); // inverse jacobian
  UMatrix4 mU(2,1); // old values
  UMatrix4 mUd(2,1); // new estimate delta
  double j11, j12, j22, j21; // derivative in u0
  double f1, f2; // function values at u0
  double uP, uP2; // unknown 1. Phi2
  double uR, uR2; // unknown 2. Turn radius (common)
  double uP1;
  double iR, iP;
  int i;
  int k, kMax = 10;
  const double ELIM = 0.0001;
  bool result = false;
  //
  // print("P1");
  //p2.print("p2");
  //
  iP = absd(limitToPi(atan2(y, x) - h)) + 0.01;
  iR = 1.0 + sqrt(sqr(x) + sqr(y)) / 4.0 / (iP + 0.03);
  //
  for (k = 0; k < kMax; k++)
  {
    if (k > 0)
    {
      iR = 2.0 * double(k);
      iP = 2.0 / double(k);
    }
    //iR = 2.0; //0.5 + double(j); // estimated radius
    uR = iR;
    //iP = 0.5; //0.1 + 0.3 * double(k);  // estimated angle for 2nd turn
    uP = iP;
    // iterate
    for (i = 0; i < 19; i++)
    { // set function value at uR, uP
      f1 = -x + uR * (sin(h) - 2.0 * sin(h - uP));
      f2 = -y + uR * (2.0 * cos(h - uP) - 1.0 - cos(h));
      result = ((absd(f1) < ELIM) and (absd(f2) < ELIM) and
          (uP >= 0.0) and uR >=0.0);
      if (result)
        break;
      // set derivative of two variables at uR, uP
      j11 = 2.0 * uR * cos(h - uP);
      j12 = sin(h) - 2.0 * sin(h - uP);
      j21 = 2.0 * uR * sin(h - uP);
      j22 = 2.0 * cos(h - uP) - 1.0 - cos(h);
      mF.setRC(0, 0, f1);
      mF.setRC(1, 0, f2);
      mJ.setRow(0, j11, j12);
      mJ.setRow(1, j21, j22);
      mJi = mJ.inversed();
      mUd = mJi * mF;
      uP2 = uP - mUd.get(0,0);
      uR2 = uR - mUd.get(1,0);
      // debug
/*      printf("Phi: %7g rad ", uP2);
      printf("Rad: %7g m (f1=%7g, f2=%7g)\n", uR2, f1, f2);*/
      // debug end
      uP = uP2;
      while (uP > 2.0 * M_PI)
        uP -= 2.0 * M_PI;
      while (uP < 0.0)
        uP += 2.0 * M_PI;
      uR = uR2;
    }
    while (uP > 2.0 * M_PI)
      uP -= 2.0 * M_PI;
    uP1 = uP - h;
    // debug
/*    printf("Right R: R=%5.2f Phi1=%5.4f Phi2=%5.4f (f1=%7g, f2=%7g)\n",
          uR, uP1, uP, f1, f2);
    printf("Returned after %d loops\n", i);*/
    // debug end
    if (result)
    {
      if (r1 != NULL)
        *r1 = uR;
      if (h1 != NULL)
        *h1 = uP1;
      if (h2 != NULL)
        *h2 = uP;
    }
    if (logFp != NULL)
    { // make additional log
      fprintf(logFp, "---%5.1fr,%5.1fa %2di: %5.2fr, %5.2fa1, %5.2fa2\n",
              iR, iP, i, uR, uP1, uP);
    }
    if (result)
      break;
  }
  return result;
}

//////////////////////////////////////////////////

// bool UPose2pose::get2here(int * manType,
//                           double * dist,
//                           double * radius,
//                           double * arc1,
//                           double * arc2,
//                           bool * left1st, FILE * logFp)
// {
//   UPosition p3, p4; // destination crosses x-axis (using dest heading)
//   double p3dd = 0.0; // distance from destination to x-axis using destination heading
//   double p4dd; // distance of arc secant
//   double a, b = 0.0, r = 0.0, d = 0.0;
//   MAN_SOL_TYP mt = MST_NONE;
//   MAN_SOL_TYP mt2 = MST_NONE;
//   bool result = true;
//   bool left = false;
//
//   //
//   if ((absd(y) < 0.05) and (x > 1.5))
//     // 5cm is OK for going straight
//     mt = MST_LINE;
//   else
//   {
//     if (absd(h) < 1e-3)
//       // very small angle - use 2-arc
//       mt = MST_2ARC;
//     else
//     { // depend on crossing of heading line with x-axis
//       p3dd = y / sin(h);
//       p3.y = 0.0;
//       p3.x = x - cos(h) * p3dd;
//       if (((p3.x * y * h) < 0.0) or (absd(y) < 0.3))
//         // better using 2 arcs
//         mt = MST_2ARC;
//       if (p3.x > p3dd)
//         // longer to crossing point than from
//         // crossing point to destination
//         mt2 = MST_LINE_ARC;
//       else
//         // start with an arc
//         mt2 = MST_ARC_LINE;
//     }
//     if (mt == MST_NONE)
//       mt = mt2;
//   }
//   // try 2arc first, as it fails, where a 1-arc may do the job
//   //
//   if (mt == MST_2ARC)
//   {
//     left = (y > 0.0);
//     if (absd(y) < 0.3)
//       // close to x-axis, then
//       // start in the other direction
//       left = (h < 0.0);
//     // get a 2-arc solution
//     if (left)
//       result = get22arcLeft(radius, arc1, arc2, logFp);
//     else
//       result = get22arcRight(radius, arc1, arc2, logFp);
//     d = 0.0;
//     if (not result and (absd(y) > 0.2))
//     { // fall back on 2nd priority method
//       mt = mt2;
//       result = true;
//     }
//     if (not result)
//     { // else just go forward
//       mt = MST_LINE;
//       result = true;
//     }
//   }
//   if (mt == MST_ARC_LINE)
//   { // find arc radius and angle
//     // get endpoint of arc, on line to dest
//     // p3.x is half turn angle on x-axis
//     p4.y = p3.x * sin(h);
//     p4.x = p3.x + p3.x * cos(h);
//     // get half distance to p4 (sekant)
//     p4dd = sqrt(sqr(p4.x) + sqr(p4.y)) / 2.0;
//     // get angle from arc center to destination heading
//     a = asin(p4dd/p3.x);
//     // get half turn arc
//     b = M_PI/2.0 - a;
//     // get arc radius
//     r = p4dd / sin(b);
//     // line distance after arc i.e. p4 to destination
//     d = sqrt(sqr(x - p4.x) + sqr(y - p4.y));
//   }
//   else if (mt == MST_LINE_ARC)
//   { // find arc radius and angle
//     p4.y = 0;
//     p4.x = p3.x - p3dd;
//     // get half distance from p4 to destination (sekant)
//     p4dd = sqrt(sqr(x - p4.x) + sqr(y - p4.y)) / 2.0;
//     // get angle from arc center to destination heading
//     a = asin(p4dd/p3dd);
//     // get half turn arc
//     b = M_PI/2.0 - a;
//     // get arc radius
//     r = p4dd / sin(b);
//     // line distance
//     d = p4.x;
//   }
//   else if (mt == MST_LINE)
//   { // just go the forward the x distance
//     d = x;
//     r = 0.0;
//     b = 0.0;
//   }
//   else if (mt == MST_ARC_LINE_ARC)
//   { // arc line arc solution
//   }
//   // return result
//   if (dist != NULL)
//     *dist = d;
//   if ((mt == MST_ARC_LINE) or (mt == MST_LINE_ARC))
//   { // return arc value
//     if (radius != NULL)
//       *radius = r;
//     if (arc1 != NULL)
//       *arc1 = 2.0 * b;
//      left = (y > 0.0);
//   }
//   if (left1st != NULL)
//     *left1st = left;
//   //
//   if (manType != NULL)
//     *manType = mt;
//   return result;
// }

////////////////////////////////////////////////////

bool UPose2pose::get2RightLineLeft(double initVel, double maxAcc, double maxTurnAcc,
                                       double * radius1, double * arc1,
                                       double * dist,
                                       double * radius2, double * arc2,
                                       FILE * logFp)
{ // we are in 0,0 and want to this pose
  // max speed of first angle is start speed,
  // max speed of exit arc is end speed.
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  double r1, r2; // arc radius
  double dcc; // distance center to enter
  double dtt; // distance from tangent point to tangent point
  bool result = false;
  double h2;
  const double halfPi = M_PI / 2.0;
  double a, b, dce;
  const double MIN_SPEED = 0.2; /* m/sec */
  //double MAX_BREAK_ACC = 2.0 * maxAcc;
  // a = angle of line from c1 to c2
  //
  r1 = sqr(initVel) / maxTurnAcc;
  r2 = sqr(vel) / maxTurnAcc;
  h2 = h + halfPi;
  c2.x = x + cos(h2) * r2;
  c2.y = y + sin(h2) * r2;
  c1.x = 0.0;
  c1.y = - r1;
  dcc = hypot(c2.x - c1.x, c2.y - c1.y);
  result = (dcc >= (r1 + r2));
  if (not result)
  { // test if a slower exit speed may solve the problem
    // get distance from c1 to exit point
    dce = hypot(x, y - c1.y);
    // find minim radius of r2
    r2 = sqr(MIN_SPEED) / maxTurnAcc;
    result = (dce - r1) > r2;
    if (result)
    { // a solution is possible if there is time to break.
      // find angle to tangent point - direct towards exit
      // is worst case
      a = atan2(y - c1.y, x);
      // traveled distance along first arc - at maximum
      //d = r1 * (halfPi - a);
      // get largest second arc (in worst case)
      r2 = (dce - r1) / 2.0;
      //v2 = sqrt(r2 * maxTurnAcc);
      // get needed acceleration (negative is decreased speed)
/*    This is not the point to give up
      a = (sqr(v2) - sqr(initVel))/(2.0 * d);
      // true if no need to break too much
      result = (a > -MAX_BREAK_ACC);*/
    }
    if (result)
    { // there is a new c2
      c2.x = x + cos(h2) * r2;
      c2.y = y + sin(h2) * r2;
      dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    }
  }
  if (not result)
  { // give a hint in r1 to what is needed
    // try a solution with r2 half of r1
    r1 = hypot(x, y) / 3.0;
    r2 = r1 / 2.0;
    c1.y = -r1;
    c2.x = x + cos(h2) * r2;
    c2.y = y + sin(h2) * r2;
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    result = true;
    // no point in giving up here
//    result = (r1 > sqr(MIN_SPEED)/maxTurnAcc);
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 + r2) / dcc);
    // angle from c1 to tangent point
    c1.h = a - b - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h + M_PI;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    if (logFp != NULL)
    {
      fprintf(logFp, "--- %.4fa, %.4fb, (%.3fx,%.3fy)c1, (%.3fx,%.3fy)t1, %.3fdtt, (%.3fx,%.3fy)t2, (%fx,%fy)c2\n",
              a, b, c1.x, c1.y, t1.x, t1.y, dtt, t2.x, t2.y, c2.x, c2.y);
    }
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = halfPi - c1.h;
      while (*arc1 < 0.0)
        *arc1 += 2.0 * M_PI;
      while (*arc1 > 2.0 * M_PI)
        *arc1 -= 2.0 * M_PI;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = h - halfPi - c2.h;
      while (*arc2 < 0.0)
        *arc2 += 2.0 * M_PI;
      while (*arc2 > 2.0 * M_PI)
        *arc2 -= 2.0 * M_PI;
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakRightLineLeft(double initVel,
                                           double maxAcc, double maxTurnAcc, double minTurnRad,
                                           double * initialBreakDist, double * toVel,
                                           double * radius1, double * arc1,
                                           double * dist,
                                           double * radius2, double * arc2,
                                           double * finalBreak)
{ // we are in 0,0 and want to this pose (x, y, h)
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  double r1, r2, r; // arc radius
  double dcc; // distance center to enter
  double dtt; // distance from tangent point to tangent point
  bool result = false;
  //double h2;
  const double halfPi = M_PI / 2.0;
  double a, b, dr;
  const double MIN_SPEED = 0.2; /* m/sec */
  double x2, y2, rMaxV, rMax, rMin, v, d1, d2;
  double cosh2, sinh2, sinh0, cosh0;
  const double RADIUS_SURPLUS_MARGIN = 0.05; // 5cm away from optimal is OK
  int i;
  //
  v = maxd(initVel, vel);
  // needed turn radius is based on maximum speed
  rMaxV = sqr(v) / maxTurnAcc;
  // but not less  than the capable turn radius
  rMax = maxd(minTurnRad * 3.0, rMaxV);
  //h2 = h + halfPi;
  cosh0 = cos(h);
  sinh0 = sin(h);
  cosh2 = -sinh0;
  sinh2 = cosh0;
  // minimum is based on minimum speed and minimum turn radius
  rMin = maxd(minTurnRad, sqr(MIN_SPEED) / maxTurnAcc);
  r1 = rMax;
  r2 = rMin;
  // rMax -= 1e-5;
  // rMin += 1e-5;
  r = r1;
  c1.x = 0;
  i = 0;
  while (not result and i < 12)
  { // test if a slower exit speed may solve the problem
    // but not higher than target velocity or start velocity
    v = fmin(sqrt(r * maxTurnAcc), fmax(initVel, vel));
    // needed break (or acc) distance
    if (rMaxV > rMax * 0.8 and initVel > vel)
      // safer to break on a straight line then
      d1 = sqr(v - initVel)/(2.0 * maxAcc);
    else
      // no need to have straight break distance
      d1 = 0.0;
    if (v > initVel and dist == NULL)
    { // initial acceleration distance is not desired
      d1 = 0.0;
    }
    x2 = x - d1;
    if (v > vel)
    { // allow a final break distance (x2,y2) is start of final break
      d2 = sqr(v - vel)/(2.0 * maxAcc);
      x2 = x2 - d2 * cosh0;
      y2 = y - d2 * sinh0;
    }
    else
    { // no final break
      y2 = y;
      d2 = 0;
    }
    // center of first arc is on y-axis
    c1.y = - r;
    // center of second arc is in direction h2 from x2, y2
    c2.x = x2 + cosh2 * r;
    c2.y = y2 + sinh2 * r;
    // center to center distance
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    // surplus radius margin
    dr = dcc - 2.0 * r;
    if ((dr > RADIUS_SURPLUS_MARGIN) and (r < rMax))
      // is possible, but can be better increase r2
      r2 = r;
    else if (dr > 1e-5)
    { // close and on the safe side - stop
      r1 = r;
      r2 = r;
      result = true;
      break;
    }
    else if (r > rMin)
      // not good decrease r1
      r1 = r;
    else
      // too small - no solution
      break;
    // try between the new limits
    r = (r1 + r2) / 2.0;
    i++;
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 + r2) / dcc);
    // angle from c1 to tangent point
    c1.h = a - b - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h + M_PI;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
      *arc1 = limitTo2Pi(halfPi - c1.h);
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
      *arc2 = limitTo2Pi(h - halfPi - c2.h);
    if (initialBreakDist != NULL)
      *initialBreakDist = d1;
    if (toVel != NULL)
      *toVel = v;
    if (finalBreak != NULL)
      *finalBreak = d2;
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakRightLineRight(double initVel,
           double maxAcc, double maxTurnAcc,
           double minTurnRad,
           double * initialBreakDist,
           double * toVel, // returned result velocity
           double * radius1, double * arc1,
           double * dist,
           double * radius2, double * arc2,
           double * finalBreak)
{ // we are in 0,0 and want to this pose (x, y, h) at 'vel' m/s
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  double r1, r2, r; // arc radius
  //double dcc; // distance center to enter
  double dtt; // distance from tangent point to tangent point
  bool result = false;
  //double h2;
  const double halfPi = M_PI / 2.0;
  double a, b, dr;
  const double MIN_SPEED = 0.2; /* m/sec */
  double x2, y2, rMax, rMaxV, rMin, v, d1, d2;
  double cosh2, sinh2, sinh0, cosh0;
  const double RADIUS_SURPLUS_MARGIN = 0.05; // 5cm away from optimal is OK
  double dc1x2, dc2x1;
  int i;
  //
  v = maxd(initVel, vel);
  rMaxV = sqr(v) / maxTurnAcc;
  // preferred turn radius at current speed
  rMax = maxd(minTurnRad * 3.0, rMaxV);
  //h2 = h - halfPi;
  cosh0 = cos(h);
  sinh0 = sin(h);
  cosh2 = sinh0;
  sinh2 = -cosh0;
  // min speed allowed turn radius - sharpest turn possible
  rMin = maxd(minTurnRad, sqr(MIN_SPEED) / maxTurnAcc);
  r1 = rMax;
  r2 = rMin;
  rMax -= 1e-5;
  rMin += 1e-5;
  r = r1;
  c1.x = 0;
  i = 0;
  while (not result and i < 12)
  { // test if a slower exit speed may solve the problem
    // but not higher than target velocity or start velocity
    v = fmin(sqrt(r * maxTurnAcc), fmax(initVel, vel));
    // needed initial break distance?
    if (rMaxV > rMax * 0.8 and initVel > v)
      // safer to break on a straight line then
      d1 = sqr(v - initVel)/(2.0 * maxAcc);
    else
      // no need to have straight break-acc distance
      d1 = 0.0;
    x2 = x - d1;
    // need final break?
    if (v > vel)
    { // allow a final break distance
      d2 = sqr(v - vel)/(2.0 * maxAcc);
      x2 = x2 - d2 * cosh0;
      y2 = y - d2 * sinh0;
    }
    else
    { // no final break
      y2 = y;
      d2 = 0;
    }
    // center of first arc is on y-axis
    c1.y = - r;
    // center of second arc is in direction h2 from x2, y2
    c2.x = x2 + cosh2 * r;
    c2.y = y2 + sinh2 * r;
    // center to center distance
    dc1x2 = hypot(c1.x - x2, c1.y - y2);
    dc2x1 = hypot(c2.x, c2.y);
    // surplus radius margin
    dr = mind(dc1x2 - r, dc2x1 - r);
    // get tangent angle
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // Get arc-2 angle, if this is slightly negative (should turn almost 360 deg), then
    // there is a chance to get a good solution if radius is smaller
    b = limitToPi(a - h);
    if ((dr > RADIUS_SURPLUS_MARGIN) and (r < rMax) and (b > 0.1))
    { // is possible, but can be better increase r2
      r2 = r;
    }
    else if (dr > 1e-5 or (b <= 0.1 and b > 0.01))
    { // close and on the safe side - use this r
      r1 = r;
      r2 = r;
      result = true;
      break;
    }
    else if (r > rMin)
      // not good decrease r1
      r1 = r;
    else
      // no solution
      break;
    // try between the new limits
    r = (r1 + r2) / 2.0;
    i++;
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // center center distance
    //dcc = hypot(c1.x - c2.x, c1.y - c2.y);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    //b = asin((r1 - r2) / dcc);
    // angle from c1 to tangent point
    //c1.h = a - b - halfPi;
    c1.h = a - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = limitTo2Pi(halfPi - c1.h);
      // minor angle (0.0005 deg) is zero
      if (*arc1 > (2.0 * M_PI - 1e-5))
        *arc1 = 0.0;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = limitTo2Pi(c2.h - (h + halfPi));
      // minor angle (0.0005 deg) is zero
      if (*arc2 > (2.0 * M_PI - 1e-5))
        *arc2 = 0.0;
    }
    if (initialBreakDist != NULL)
      *initialBreakDist = d1;
    if (toVel != NULL)
      *toVel = v;
    if (finalBreak != NULL)
      *finalBreak = d2;
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UPose2pose::get2RightLineRight(double initVel, double maxAcc, double maxTurnAcc,
                                   double * radius1, double * arc1,
                                   double * dist,
                                   double * radius2, double * arc2,
                                   FILE * logFp)
{ // we are in 0,0 and want to this pose
  // max speed of first angle is start speed,
  // max speed of exit arc is end speed.
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  double r1, r2; // arc radius
  double dc1e; // distance from first center to exit point
  double dc2s; // distance from center 2 to start point
  double dtt; // distance from tangent point to tangent point
  bool result = false;
  double h2;
  const double halfPi = M_PI / 2.0;
  double a, b, d;
  //const double MIN_SPEED = 0.2; // m/sec
  double dcc; // ceter-center distance
  // a = angle of line from c1 to c2
  //
  r1 = sqr(initVel) / maxTurnAcc;
  r2 = sqr(vel) / maxTurnAcc;
  h2 = h - halfPi;
  c2.x = x + cos(h2) * r2;
  c2.y = y + sin(h2) * r2;
  c1.x = 0.0;
  c1.y = - r1;
  dc1e = hypot(c1.x - x, c1.y - y);
  dc2s = hypot(c2.x, c2.y);
  if (dc2s <= r2)
  { // a slower exit speed will be needed, as origo is inside arc
    // The line from exit to orogo has a mid-normal, that
    // crosses the exit turn center, then line in direction h2
    // crosses center too, find cross.
    // distance to exit is
    d = hypot(x, y) / 2.0;
    // angle between line at exit is
    a = h2 - atan2(-y, -x);
    // from exit to cross (hypotonuse) is turn radius
    r2 = d / cos(a) - 0.02;
    // get new center
    c2.x = x + cos(h2) * r2;
    c2.y = y + sin(h2) * r2;
    // and center - center distance
    //dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    //
    // result = r2 >= sqr(MIN_SPEED)/maxTurnAcc;
  }
  if (dc1e < r1)
  { // r1 is too big, find a usable value
    // solve triangle from 0.0, to new r1, to midpoint from 0.0 to exit
    // angle to exit is the same as angle at r1 in triangle
    a = atan2(y, x);
    // distance to half-exit point
    d = hypot(y, x) / 2.0;
    // sin(a) = d/r1.x
    r1 = fabs(d/sin(a)) - 0.02;
    // new center value
    c1.y = - r1;
  }
  result = (r2 >= 0.0) and (r1 >= 0.0);
  if (result)
  { // circles cross or are separated
    // get angle og c1 to c2 line
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get centre centre distance
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 - r2) / dcc);
    // angle from c1 to tangent point
    c1.h = limitToPi(a - b - halfPi);
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    // log calculation - if logfile exist
    if (logFp != NULL)
    {
      fprintf(logFp, "--- %.4fa, %.4fb, (%.3fx,%.3fy)c1, (%.3fx,%.3fy)t1, %.3fdtt, (%.3fx,%.3fy)t2, (%fx,%fy)c2\n",
              a, b, c1.x, c1.y, t1.x, t1.y, dtt, t2.x, t2.y, c2.x, c2.y);
    }
/*    fprintf(stdout, "--- (%.2fx,%.2fy)c1, (%.2fx,%.2fy)t1, %.2fdtt,\n"
        "--- (%.3fx,%.3fy)c2, (%.2fx,%.2fy)t2 (RR)\n",
        c1.x, c1.y, t1.x, t1.y, dtt, c2.x, c2.y, t2.x, t2.y);*/
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = halfPi - c1.h;
      while (*arc1 < 0.0)
        *arc1 += 2.0 * M_PI;
      while (*arc1 > 2.0 * M_PI)
        *arc1 -= 2.0 * M_PI;
      // minor angle (0.0005 deg) is zero
      if (*arc1 > (2.0 * M_PI - 1e-5))
        *arc1 = 0.0;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = c2.h - (h + halfPi);
      while (*arc2 < 0.0)
        *arc2 += 2.0 * M_PI;
      while (*arc2 > 2.0 * M_PI)
        *arc2 -= 2.0 * M_PI;
      // minor angle (0.0005 deg) is zero
      if (*arc2 > (2.0 * M_PI - 1e-5))
        *arc2 = 0.0;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2LeftLineLeft(double initVel, double maxAcc, double maxTurnAcc,
                                    double * radius1, double * arc1,
                                    double * dist,
                                    double * radius2, double * arc2,
                                    FILE * logFp)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2RightLineRight(initVel, maxAcc, maxTurnAcc,
                            radius1, arc1, dist, radius2, arc2, logFp);
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakLeftLineLeft(double initVel,
                                  double maxAcc, double maxTurnAcc, double minTurnRad,
                                  double * breakDist, double * toVel,
                                  double * radius1, double * arc1,
                                  double * dist,
                                  double * radius2, double * arc2, double * finalBreak)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2ViaBreakRightLineRight(initVel, maxAcc, maxTurnAcc, minTurnRad,
                               breakDist, toVel,
                               radius1, arc1,
                               dist,
                               radius2, arc2,
                               finalBreak);
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2LeftLineRight(double initVel, double maxAcc, double maxTurnAcc,
                                  double * radius1, double * arc1,
                                  double * dist,
                                  double * radius2, double * arc2,
                                  FILE * logFp)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2RightLineLeft(initVel, maxAcc, maxTurnAcc,
                            radius1, arc1, dist, radius2, arc2, logFp);
}

//////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakLeftLineRight(double initVel,
                                  double maxAcc, double maxTurnAcc, double minTurnRad,
                                  double * breakDist, double * toVel,
                                  double * radius1, double * arc1,
                                  double * dist,
                                  double * radius2, double * arc2, double * finalBreak)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2ViaBreakRightLineLeft(initVel, maxAcc, maxTurnAcc, minTurnRad,
                              breakDist, toVel,
                              radius1, arc1, dist, radius2, arc2, finalBreak);
}

//////////////////////////////////////////////////

bool UPose2pose::get2hereALA(int * manType,
                          double initVel, double maxAcc, double maxTurnAcc,
                          double * radius1,
                          double * arc1,
                          double * dist,
                          double * radius2,
                          double * arc2, FILE * logFp)
{
  bool result = true;
  //double v1 = initVel; //, v2;
  const int MSL = 4;
  bool isOK;
  int i;
  int best = -1;
  double minArc = M_PI * 3.0;
  double r1, r2, a1, a2, d, asum;
  //const double MIN_VEL = 0.2;
  UPosition p3, p4; // destination crosses x-axis (using dest heading)
  //
  // limit minimum velocity
  //v1 = maxd(MIN_VEL, initVel);
  // try all 4 combinations, and select the shortest in time.
  for (i = 0; i < MSL; i++)
  {
    switch (i)
    {
      case 0:
        isOK = get2LeftLineLeft(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 1:
        isOK = get2LeftLineRight(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 2:
        isOK = get2RightLineLeft(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 3:
        isOK = get2RightLineRight(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      default:
        isOK = false;
        break;
    }
    if (isOK)
    {
      // find best score (based on angles only)
      asum = (a1 + a2);
      // angles above 180 deg count double
      if (a1 > M_PI)
        asum += a1;
      if (a2 > M_PI)
        asum += a2;
      if ((a1 + a2) < minArc)
      {
        best = i;
        minArc = a1 + a2;
        if (radius1 != NULL)
          *radius1 = r1;
        if (radius2 != NULL)
          *radius2 = r2;
        if (arc1 != NULL)
          *arc1 = a1;
        if (arc2 != NULL)
          *arc2 = a2;
        if (dist != NULL)
          *dist = d;
      }
      if (logFp != NULL)
      {
        fprintf(logFp, "using %d, %.2fm, (a1=%.3f, d=%.2fm, r2=%.2fm, a2=%.3f)\n",
              i, a1 * r1 + d + a2 * r2,
              a1, d, r2, a2);
      }
    }
  }

  result = best >= 0;
  if (manType != NULL)
    *manType = best;
  //
  return result;
}

//////////////////////////////////////////////////

bool UPose2pose::get2hereLALA(int * manType,
            double initVel, double maxAcc,
            double maxTurnAcc, double minTurnRad,
            double * breakDist, double * breakToVel,
            double * radius1,
            double * arc1,
            double * dist,
            double * radius2,
            double * arc2,
            double * finalBreak)
{
  bool result = true;
  //double v1 = initVel; //, v2;
  const int MSL = 4;
  bool isOK;
  int i;
  int best = -1;
  double minArc = 20.0;
  double r1, r2, a1, a2, d, asum, b, v, bFinal;
  //const double MIN_VEL = 0.2;
  UPosition p3, p4; // destination crosses x-axis (using dest heading)
  //
  // limit minimum velocity
  //v1 = maxd(MIN_VEL, initVel);
  // try all 4 combinations, and select the shortest in time.
  for (i = 0; i < MSL; i++)
  { // desired end velocity is in poseV (vel)
    v = initVel; // calculated end velocity
    b = 0.0;
    switch (i)
    {
      case 0:
        isOK = get2ViaBreakLeftLineLeft(initVel, maxAcc,
                   maxTurnAcc, minTurnRad,
                   &b, &v, &r1, &a1, &d,
                   &r2, &a2, &bFinal);
        break;
      case 1:
        isOK = get2ViaBreakLeftLineRight(initVel, maxAcc,
                   maxTurnAcc, minTurnRad,
                   &b, &v, &r1, &a1, &d,
                   &r2, &a2, &bFinal);
        break;
      case 2:
        isOK = get2ViaBreakRightLineLeft(initVel, maxAcc,
                      maxTurnAcc, minTurnRad,
                      &b, &v, &r1, &a1, &d,
                      &r2, &a2, &bFinal);
        break;
      case 3:
        isOK = get2ViaBreakRightLineRight(initVel, maxAcc,
                      maxTurnAcc, minTurnRad,
                      &b, &v, &r1, &a1, &d,
                      &r2, &a2, &bFinal);
        break;
      default:
        isOK = false;
        break;
    }
    if (isOK)
    {
      // find best score (based on angles only)
      asum = (a1 + a2);
      // angles above 180 deg count double
      if (a1 > M_PI)
        asum += a1;
      if (a2 > M_PI)
        asum += a2;
      if (asum < minArc)
      {
        best = i;
        minArc = asum;
        if (radius1 != NULL)
          *radius1 = r1;
        if (radius2 != NULL)
          *radius2 = r2;
        if (arc1 != NULL)
          *arc1 = a1;
        if (arc2 != NULL)
          *arc2 = a2;
        if (dist != NULL)
          *dist = d;
        if (breakDist != NULL)
          *breakDist = b;
        if (breakToVel != NULL)
          *breakToVel = v;
        if (finalBreak != NULL)
          *finalBreak = bFinal;
      }
  /*        if (logFp != NULL)
      {
        fprintf(logFp, "using %d, %.2fm, (a1=%.3f, d=%.2fm, r2=%.2fm, a2=%.3f)\n",
                i, a1 * r1 + d + a2 * r2,
                a1, d, r2, a2);
      }*/
    }
  }
  result = best >= 0;
  if (manType != NULL)
    *manType = best;
  //
  return result;
}


////////////////////////////////////////////////////////////

UPose UPose2pose::get2line(double turnRad, double * turn1, double * direct, double * turn2)
{ // turn radius is defined from driveon parameters angle gain and distance gain
  // angle gain gA gives turn angle left new angle is to the left.
  // distance gain gD gives turn angle left, if to the right og line,
  // but no more than a turn angle from a 90 deg turn.
  // double turnRad = M_PI/2.0 * gA / gD;
  U2Dseg line; // exit line description
  U2Dpos c1, c2;
  bool startRight;
  UPose result;
  double d1, d2, t1, t2; //, tu1, tu2l, tu1r, tu2r, dil, dir;
  //
  if (fabs(h) < 1e-7)
  {
    h = 0.0;
    startRight = false;
  }
  else
  {
    line.setFromPose(x, y, h, 1.0);
    // start left turn centre
    c1.x = 0.0;
    c1.y = turnRad;
    // start right turn centre
    c2.x = 0.0;
    c2.y = - turnRad;
    // get distance to line from both
    d1 = fabs(line.distanceSigned(c1.x, c1.y));
    d2 = fabs(line.distanceSigned(c2.x, c2.y));
    //printf("Distance to %.2f,%.2f,%.4f left,right (d1=%.3f, d2=%.3f,  radius %.2fm)\n",
    //       x, y, h, d1, d2, turnRad);
    if (d1 > turnRad or d2 > turnRad)
    { // at large angles, the destination count
  /*    if (fabs(h) >= M_PI / 2.0)
        startRight = y < 0.0;
      else*/
        // at small angles the closest center count
        startRight = d2 < d1;
    }
    else
    { // the closest point in the line is more forward
      // for the start turn
      t1 = line.getPositionOnLine(c1);
      t2 = line.getPositionOnLine(c2);
      //printf("Position left,right (t1=%.3g, t2=%.3g) radius %.2fm\n",
      //       t1, t2, turnRad);
      startRight = t2 > t1;
    }
  }
  //printf("so start right (%s)\n", bool2str(startRight));
  //
  for (int i = 0; i < 2; i++)
  {
    if (startRight)
    { // mirror coordinate system
      y = -y;
      h = -h;
    }
    result = get2lineStartLeft(turnRad, turn1, direct, turn2);
    if (startRight)
    { // reverse back the result
      y = -y;
      h = -h;
      result.y = -result.y;
      result.h = -result.h;
      *turn1 = -*turn1;
      *turn2 = -*turn2;
    }
    if (fabs(*turn1) < 6.0 and fabs(*turn2) < 6.0)
      break;
    printf("*** UPose2pose::get2line mistake!  %.5fx,%.5fy,%.8fh by %.4frad,%.2fm,%.4frad\n", x,y,h, *turn1, *direct, *turn2);
    startRight = not startRight;
  }
/*  printf("p2p to %.2f,%.2f,%.4f by %.4frad,%.2fm,%.4frad\n",
         x,y,h, *turn1, *direct, *turn2);*/
  return result;
}



// UPose UPose2pose::get2line(double gA, double gD, double * turn1, double * direct, double * turn2)
// {
//   UPose result, r2;
//   int i;
//   U2Dlined line; // target line
//   double d1, d2 = 1e6, dm1, dm2;
//   double t1, t2, di;
//   double turnRad = M_PI/2.0 * gA / gD;
//   //
//   line.setPH(x, y, h);
//   for (i = 0; i < 2; i++)
//   {
//     if (i == 1)
//     { // mirror coordinate system
//       y = -y;
//       h = -h;
//     }
//     result = get2lineStartLeft(turnRad, turn1, direct, turn2);
//     if (i == 1)
//     { // reverse back the result
//       y = -y;
//       h = -h;
//       result.y = -result.y;
//       result.h = -result.h;
//       *turn1 = -*turn1;
//       *turn2 = -*turn2;
//     }
//     // test result
//     d1 = fabs(line.distanceSigned(result.x, result.y));
//     dm1 = (fabs(*turn1) + fabs(*turn2)) * turnRad + *direct;
//     // debug
//     printf("p2p #%d: to %.2f,%.2f,%.4f by %.4frad,%.2fm,%.4frad lng=%.2f\n", i, x,y,h, *turn1, *direct, *turn2, dm1);
//     // debug end
// /*    if (fabs(d1) < 0.01 and fabs(*turn1 + *turn2 - h) < 0.001)
//       break;*/
//     if (dm2 < dm1 and
//         d2 < 0.01 and
//         fabs(t1 + t2 - h) < 0.001 and
//         (*direct < 0.01 and t1*t2 <= 0.0))
//     { // first result is better - use
//       result = r2;
//       *direct = di;
//       *turn1 = t1;
//       *turn2 = t2;
//       break;
//     }
//     if (i == 1)
//       // second result is better
//       break;
//     d2 = d1;
//     dm2 = dm1;
//     di = *direct;
//     t1 = *turn1;
//     t2 = *turn2;
//     r2 = result;
//   }
//   return result;
// }

/////////////////////////////////////////////////////////

// UPose UPose2pose::get2line(double gA, double gD, double * turn1, double * direct, double * turn2)
// { // turn radius is defined from driveon parameters angle gain and distance gain
//   // angle gain gA gives turn angle left new angle is to the left.
//   // distance gain gD gives turn angle left, if to the right og line,
//   // but no more than a turn angle from a 90 deg turn.
//   double turnRad = M_PI/2.0 * gA / gD;
//   double d, nt, nd;
//   U2Dseg seg; // exit line description
//   U2Dpos nPos;
//   bool startRight, square1and3;
//   UPose result;
//   double t1, t2; //, tu1, tu2l, tu1r, tu2r, dil, dir;
//   int n;
//   //
//   seg.setFromPose(x, y, h);
//   // get right angle distance
//   nd = fabs(seg.C());
//   // get coordinates of point nearest to origin (current pose)
//   nt = seg.getPositionOnLine(0.0, 0.0);
//   nPos = seg.getPositionOnLine(nt);
//   d = cos(h)*turnRad;
//   // heading forward-left or backwards-right
//   square1and3 = seg.vx() * seg.vy() > 0.0;
//   //
//   // divide into start-left and start right
//   if (nd > d + turnRad)
//   { // straight distance before final turn of 90 deg or
//     // circles touch, but behaviour as for straight line
//     // start right if line crosses in front of robot
//     // and reversed if line crosses behind robot
//     startRight = square1and3 xor (nPos.x < 0.0);
// //    printf("cross far away (at %.2fx,%.2fy,%.1f) - ", nPos.x, nPos.y, h * 180.0 / M_PI);
//   }
//   else
//   { //crossing near current position, solution changes, if
//     // too close to correct in time, that is if heading is negative and
//     // the path crosses the turn right circle, then start right
//     // if no crossing then correct a bit left first, and get there on the right turn
//     if (h < 0.0)
//     { // heading is negative, so right turn circle determines the solution
//       n = seg.getCircleCrossings(0.0, -turnRad, turnRad, &t1, &t2);
//       startRight = (n == 2);
//     }
//     else
//     { // heading is positive, so left turn circle determines solution
//       // if crossing, then start left
//       n = seg.getCircleCrossings(0.0, turnRad, turnRad, &t1, &t2);
//       startRight = not (n == 2);
//     }
// //    printf("cross close (at %.2fx,%.2fy,%.1f) - ", nPos.x, nPos.y, h * 180.0 / M_PI);
//   }
// /*  if (startRight)
//     printf(" start-right\n");
//   else
//     printf(" start-left\n");*/
//   //
//   if (startRight)
//   { // mirror coordinate system
//     y = -y;
//     h = -h;
//   }
//   result = get2lineStartLeft(turnRad, turn1, direct, turn2);
//   if (startRight)
//   { // reverse back the result
//     y = -y;
//     h = -h;
//     result.y = -result.y;
//     result.h = -result.h;
//     *turn1 = -*turn1;
//     *turn2 = -*turn2;
//   }
//   return result;
// }

//////////////////////////////////////////////////

UPose UPose2pose::get2lineStartLeft(double turnRad,
                                    double * turn1,
                                    double * direct,
                                    double * turn2)
{
  UPose result; // pose at end of turn
  double d2l; // distance from turn centre to destination line
  U2Dseg dSeg; // destination line
  double hca; // half return angle
  double hrd; // half return distance, when crossing destination line
  double hrdd; // half return distance in destination direction
  double t;
  U2Dpos nPos; // new position when line is reached
  double hp = h; // positive destination heading
  // set line parameters from destination pose
  if (hp < 0.0)
    hp += M_PI;
  /// @todo error if h < ~ -M_PI/2 
  dSeg.setFromPose(x, y, hp); 
  // distance from circle centre (0,turnRad) perpendicular to line
  d2l = dSeg.distanceSigned(0.0, turnRad);
  // get also position on destination line, where
  // manoeuvre ends - if far from line
  t = dSeg.getPositionOnLine(0.0, turnRad);
  // if distance is > turn radius, then surplus distance for straight part
  if (d2l >= turnRad)
  { // first turn gets to straight part
    if (hp > M_PI / 2.0)
    {
      *turn1 = hp - M_PI / 2.0;
      // distance is the remainimg part less the last 90deg turn
      *direct = d2l - turnRad;
      // second turn is 90deg
      if (h >= 0.0)
        *turn2 = M_PI / 2.0;
      else
      { // going the other way
        *turn2 = -M_PI / 2.0;
        // finish further away
        t -= 2.0 * turnRad;
      }
    }
    else
    { // less than 90 deg turn remainimg, so
      // turn to half remainin angle
      *turn1 = hp / 2.0;
      *turn2 = *turn1;
      // straight part of distance to c1's point on target line
      // from end of first turn
      // h is tarhet heading, r is turn radius,
      // d2l is distance to target line from first turn centre
      // b := from centre c1 at half-angle to cross with target line
      // b := d2l / cos(h/2);
      // alpha is angle between line b and target line
      // alpha := Pi/2 - h - h/2;
      // y is extension of direct tangent line befor crossing target line
      // y := r * tan(h/4);
      // b-r is part of b line outside first turning circle
      // x is tangent side opposite angle alpha (crossing b at right angle)
      // x := (b - r) * tan(alpha);
      // direct := x - y;
      *direct = (d2l / cos(h/2.0) - turnRad) * tan(M_PI/2.0 - h/2.0) - turnRad * tan(h/4.0);
    }
  }
  else if (d2l <= -turnRad)
  { // destination line is (far) behind robot
    if (h > - M_PI / 2.0)
    { // first turn gets to straight part facing destination line
      *turn1 = fabs(h) - M_PI / 2.0 + M_PI;
      // second turn is 90deg
      *turn2 = -M_PI / 2.0;
      // end position advances by 2 x turn rad
      t += 2.0 * turnRad;
    }
    else
    { // heading is almost back, so turn about 90 deg
      // 90 deg clockwise, or 1.5 Pi positive, as h angle is negative
      *turn1 = h - M_PI / 2.0 + 2.0 * M_PI;
      // second turn is 90deg
      *turn2 = M_PI / 2.0;
      // end position is OK
    }
    // distance is the remainimg part
    *direct = fabs(d2l) - turnRad;
  }
  else
  { // too close for a straight part
    // and heading is positive
    *direct = 0.0;
    // correct sign is needed for distance to line
    if (h < 0.0)
    {
      dSeg.setFromPose(x, y, h);
      d2l = -d2l;
      t = -t;
    }
    // initial turn crosses destination line, so
    // so turn is to destination heading plus a bit.
    // the bit must bring the robot half the way back to the line
    hrd = (turnRad - d2l) / 2.0;
    // angle to turn passed the top point
    hca = acos((turnRad - hrd) / turnRad);
    // first turn (must be positive)
    *turn1 = limitTo2Pi(h + hca);
    // turn 2 is in the other direction
    *turn2 = -hca;
    // distance in destination direction of last turn
    hrdd = turnRad * sin(hca);
    // end pose is closest point plus the last 2 small turns
    t += hrdd * 2.0;
  }
  nPos = dSeg.getPositionOnLine(t);
  result.x = nPos.x;
  result.y = nPos.y;
  // destination heading remains
  result.h = h;
  //
  return result;
}


// UPose UPose2pose::get2lineStartLeftOld(double turnRad, double * turn1, double * direct, double * turn2)
// {
//   double d, c1x, c1y, c1r;
//   U2Dseg seg, segp; // exit line description
//   U2Dpos nPos;
//   U2Dpos c2;
//   double t1, t2, tc2;
//   double nt, nd; // nearest point parameters
//   int n;
//   UPose result;
//   //
//   seg.setFromPose(x, y, h);
//   // set start circle
//   c1x = 0.0;
//   c1y = turnRad;
//   // end circle may be on rouble radius perimeter
//   c1r = 2.0 * turnRad;
//   // point on end-pose line closest to the the origin
//   nt = seg.getPositionOnLine(0.0, 0.0);
//   // get right angle distance
//   nd = fabs(seg.C());
//   // get coordinates of point nearest to origin (current pose)
//   //nPos = seg.getPositionOnLine(nt);
//   // move to parallel line to the right, and turnRad away
//   // result pose heading do not change
//   result.h = h;
//   // distance perpendicular to target line used by first turn (assuming stratight line > 0)
//   d = fabs(cos(h)*turnRad);
//   if (nd > d + turnRad)
//   { // start and end circles are not touching, so
//     // turn to perpendicular to line
//     *turn1 = M_PI / 2.0 + h;
//     if (*turn1 > M_PI)
//       *turn1 -= M_PI;
//     // then to the exit circle
//     *direct = nd - d - turnRad;
//     // get parallel distance used on first turn
//     d = (1.0 - cos(*turn1)) * turnRad;
//     // which must be 90 deg.
//     if (seg.vx() > 0.0)
//     {
//       *turn2 = -M_PI / 2.0;
//       // origin position + d + one more radius for final turn
//       nPos = seg.getPositionOnLine(nt + d + turnRad);
//     }
//     else
//     { // going back, so reverse the final turn
//       *turn2 = M_PI / 2.0;
//       // origin position + d + one more radius for final turn
//       nPos = seg.getPositionOnLine(nt - d + turnRad);
//     }
//     result.x = nPos.x;
//     result.y = nPos.y;
// //    printf("not touching\n");
//   }
//   else // if (nd > d)
//   { // circles are touching, so no direct
//     *direct = 0.0;
//     // make a parallel line to the right of pose line
//     segp = seg;
//     segp.shiftRight(turnRad);
//     // get potential crossing positions with the side shifted line
//     n = segp.getCircleCrossings(c1x, c1y, c1r, &t1, &t2);
//     if (n == 0)
//     {
//       printf("UPose2pose::get2line: **** Logic error, no crossing when expected ***\n");
//       result = *this;
//     }
//     else
//     { // only the most forward crossing are usefull.
//       if (t1 > t2)
//         tc2 = t1;
//       else
//         tc2 = t2;
//       // get center position for exit turn
//       c2 = segp.getPositionOnLine(tc2);
//       // first turn from line to center of exit circle
//       *turn1 = atan2(c2.x, turnRad - c2.y);
//       if (*turn1 < 0.0)
//         *turn1 += M_PI * 2.0;
//       // second turn is the rest
//       *turn2 = h - *turn1;
//       if (*turn2 > 0.0)
//         *turn2 -= M_PI * 2.0;
//       if (*turn2 < -M_PI*2.0)
//         *turn2 += M_PI * 2.0;
//       // exit point is where exit circle touches line
//       nPos = seg.getPositionOnLine(tc2);
//       result.x = nPos.x;
//       result.y = nPos.y;
//     }
//   }
//   // debug
//   //  printf("first %gdeg left then straight for %gm then %g deg (positiv is left)\n",
//   //         *turn1 * 180.0 / M_PI, *direct, *turn2 * 180.0 / M_PI);
//   //  printf("result is %gx,%gy,%gh\n", result.x, result.y, result.h);
//   // debug end
//   return result;
// }
