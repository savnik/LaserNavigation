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
#include <stdio.h>

#include <ugen4/ucommon.h>
#include <ugen4/uline.h>
#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>

#include "ulaserdata.h"




UFakeMap::UFakeMap()
{
  reset();
  distError = 0.0;
  headError = 0.0;
  distOffset = 0.0;
  headOffset = 0.0;
  rangeError = 0.0;
  keepODOpose = true;
  keepUTMpose = false;
  fakeTargetPoseCnt = 0;
  fakeWallOnlyCnt = 0;
  fakeMapCnt = 0;
  initFakeMap();
}

void UFakeMap::reset()
{
  state = 0;
  currentTruePose.clear();
  currentTruePose.t.now();
  currentOdoPose.clear();
}

bool UFakeMap::copyToPoly()
{
  bool isOK = false;
  if (varPool != NULL)
  {
    isOK = varPool->callVS("poly.del", "fakeMap*");
    if (isOK)
    { // a polygon plugin is available
      ULineSegment * seg1 = &fakeMap[0]; 
      ULineSegment * seg2 = &fakeMap[0];
      const int MSL = 12;
      char s[MSL];
      int n = 0;
      UPolygon poly;
      poly.setSize(10);
      for (int i = 1; i < fakeWallOnlyCnt; i++)  
      {
        UPosition p1 = seg1->getOtherEnd();
        if (seg2->pos.distSq(&p1) > 0.001)
        {
          if (poly.getPointsCnt() > 0)
          { // send old poly to poly-plugin
            poly.setAsPolyline();
            poly.setColor("b2dd"); // b2dd: blue solid line 2 pixel wide
            snprintf(s, MSL, "mapWall%03d", n);
            varPool->callVSCD("poly.setPolygon", s, &poly,  2);
            n++;
            poly.clear();
          }
        }
        if (poly.getPointsCnt() == 0)
          poly.add(seg2->pos.x, seg2->pos.y);
        poly.add(seg2->getOtherEnd());
        seg1 = seg2;
        seg2++;
      }
      // and last line
      if (poly.getPointsCnt() > 0)
      { // send old poly to poly-plugin
        poly.setAsPolyline();
        poly.setColor("b2dd"); // b2dd: blue solid line 2 pixel wide
        snprintf(s, MSL, "mapWall%03d", n);
        varPool->callVSCD("poly.setPolygon", s, &poly,  2);
        n++;
        poly.clear();
      }
      for (int i = 0; i < fakeTargetPoseCnt; i++)
      {
        UPose *p1 = &fakeTargetPose[i - 1];
        poly.add(p1->x, p1->y);
      }
      poly.setAsPolyline();
      poly.setColor("g1dd"); // b2dd: blue solid line 2 pixel wide
      varPool->callVSCD("poly.setPolygon", "fakePath", &poly,  2);
    }
  }
  return isOK;
}


///////////////////////////////////////////////////////////////////////

void UFakeMap::initFakeMap()
{
  // make fake line environment
  int n = 0;
    // left side
  fakeMap[n++].setFromPoints(UPosition(0.0, 2.0, 0.0), UPosition(3.0, 2.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(3.0, 2.0, 0.0), UPosition(3.0, 2.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(3.25,3.7, 0.0), UPosition(3.75,3.7, 0.0));
  fakeMap[n++].setFromPoints(UPosition(4.0, 2.0, 0.0), UPosition(4.0, 2.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(4.0, 2.4, 0.0), UPosition(3.95, 1.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(4.0, 2.0, 0.0), UPosition(7.0, 2.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(0.0, 5.0, 0.0), UPosition(8.5, 5.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(6.0, 5.0, 0.0), UPosition(8.5, 3.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(3.5, 3.7, 0.0), UPosition(3.5, 5.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(4.0, 2.5, 0.0), UPosition(7.0, 2.0, 0.0));
  // right side
  fakeMap[n++].setFromPoints(UPosition(0.0, -1.0, 0.0), UPosition(3.0, -1.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(0.0, -1.2, 0.0), UPosition(3.0, -1.2, 0.0));
  fakeMap[n++].setFromPoints(UPosition(0.0, -1.0, 0.0), UPosition(0.0, -1.2, 0.0));
  fakeMap[n++].setFromPoints(UPosition(2.0, -1.0, 0.0), UPosition(2.0, -0.8, 0.0));
  fakeMap[n++].setFromPoints(UPosition(3.0, -1.0, 0.0), UPosition(3.5, -1.866,0.0));
  fakeMap[n++].setFromPoints(UPosition(4.0, -1.0, 0.0), UPosition(7.0, -1.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(8.0, -1.0, 0.0), UPosition(8.0, -2.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(4.5, -1.0, 0.0), UPosition(4.5, -5.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(0.0, -4.5, 0.0), UPosition(8.5, -4.5, 0.0));
  // back end
  fakeMap[n++].setFromPoints(UPosition(8.5, -4.5, 0.0), UPosition(8.5, 4.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(-2.0, -3.5, 0.0), UPosition(-2.0, -0.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(-2.0, 0.5, 0.0), UPosition(-2.0, 3.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(-2.0, -3.5, 0.0), UPosition(0.0, -4.5, 0.0));
  fakeMap[n++].setFromPoints(UPosition(-2.0, 3.5, 0.0), UPosition(0.0, 5.0, 0.0));
  fakeWallOnlyCnt = n;
  // obsts
  fakeMap[n++].setFromPoints(UPosition(1.5,  1.0, 0.0), UPosition(1.7, 1.2, 0.0));
  fakeMap[n++].setFromPoints(UPosition(1.8,  1.2, 0.0), UPosition(2.0, 1.0, 0.0));
  fakeMap[n++].setFromPoints(UPosition(5.25, 1.25, 0.0), UPosition(5.5, 1.0,0.0));
  fakeMap[n++].setFromPoints(UPosition(5.75, 1.25, 0.0), UPosition(6.0, 1.0,0.0));
  fakeMap[n++].setFromPoints(UPosition(5.5, 1.0, 0.0), UPosition(5.75, 1.25,0.0));
  fakeMap[n++].setFromPoints(UPosition(6.0, 1.0, 0.0), UPosition(6.25, 1.25,0.0));
  fakeMap[n++].setFromPoints(UPosition(5.25, -0.5, 0.0), UPosition(5.5, -0.25,0.0));
  fakeMap[n++].setFromPoints(UPosition(5.5, -0.25, 0.0), UPosition(5.75, -0.5,0.0));
  fakeMapCnt = n;

  // make fake path
  n = 0;
  fakeTargetPose[n++].set(6.7, 0.5, 0.0);
  fakeTargetPose[n++].set(7.8, 2.8, M_PI/2.0);
  fakeTargetPose[n++].set(7.0, 3.5, 120.0 * M_PI/180.0);
  fakeTargetPose[n++].set(3.6, 2.3, -M_PI/2.0);
  fakeTargetPose[n++].set(4.0, -1.75, -45.0 * M_PI/180.0);
  fakeTargetPose[n++].set(3.0, -3.5, M_PI);
  fakeTargetPose[n++].set(0.0, -3.0, M_PI);
  fakeTargetPose[n++].set(0.0, 0.0, 0.0);
  fakeTargetPoseCnt = n;
}

/////////////////////////////////////////////////////////

UPoseTVQ UFakeMap::fakeAdvancePose(double dt)
{
  UPoseTVQ pt;
  double d;
  UPose rob, devPose;
  //
//   scanTime.Now();
  pt = currentTruePose;
  pt.t += dt;
  pt.q = 1.0;
      // move 10 cm and add a random heading change
      //dh = double(random())/double(RAND_MAX) * 0.1 - 0.05;
      //pt.add(0.02, dh);
//   devPose.set(devicePose.getPos(), devicePose.getRot() );
//  rob = fakeTargetPose[state] - devPose;
  d = fakeAdvanceControl(&pt, fakeTargetPose[state], dt);
  if (d < 0.8)
  {
    state = (state + 1) % fakeTargetPoseCnt;
    printf("Faking to state %d:\n", state);
    // targetPose[state].print(" - ");
  }
  if (true)
  {
    double dist = pt.getDistance(currentTruePose);
    double dh = limitToPi(pt.h - currentTruePose.h);
    double distErr = 0.0;
    double headErr = 0;
    if (fabs(dist) > 0.000001)
    {
      distErr = getPink(distError, 3)*dist + distOffset*dist; 
      headErr = getPink(headError, 3)*dist + headOffset*dist;
    }
    currentOdoPose.add(dist + distErr, dh + headErr);
    currentOdoPose.t = pt.t;
  }
  if (keepUTMpose and resUTM != NULL)
  {
    resUTM->addIfNeeded(currentTruePose, -1); 
  }
  if (keepODOpose and resOdo != NULL)
  {
    resOdo->addIfNeeded(currentOdoPose, -1); 
  }
  currentTruePose = pt;
  // get current laser scanner pose - NB! rotation around x-axis is not implemented
  //pt.add(devicePose.x, devicePose.y, devicePose.Kappa);
  UPose dp(devicePose.x, devicePose.y, devicePose.Kappa);
  currentLaserPose = pt + dp;
  // 
  return currentTruePose;
}

////////////////////////////////////////////////

double UFakeMap::fakeAdvanceControl(UPoseTVQ * currentPose, UPose targetPose, double dt)
{
  const double kP = 0.8;
  const double kA = 1.0;
  const double kB = -0.35;
  const double maxV = 0.5;
  const double maxA = 0.3;
  double v, dv;
  double w;
  double d; // distance to target
  double a; // angle to target
  double b; // angle from direction to target to desired end direction
  //
  d = hypot(targetPose.x - currentPose->x, targetPose.y - currentPose->y);
  a = atan2(targetPose.y - currentPose->y, targetPose.x - currentPose->x);
  b = limitToPi(targetPose.h - a);
  a = limitToPi(a - currentPose->h);
  // velocity
  v = kP * d;
  v = fmin(maxV, v);
  dv = v - currentPose->vel;
  if (dv > maxA * dt)
    dv = maxA * dt;
  currentPose->vel += dv;
  // rotation speed
  w = kA * a + kB * b;
  // advance
  currentPose->add(currentPose->vel * dt, w * dt);
  currentPose->t += dt;
  //
  return d;
}

/////////////////////////////////////////////////////////

double UFakeMap::getFake2range(double angle, double maxR, double minR)
{
  double range = maxR;
  int i;
  ULineSegment las, *ms;
  UPosition p1, p2, p3;
  double a, r = 0.0, dh;
  //
  // set robot position
  p1.set( currentLaserPose.x, currentLaserPose.y, 0.0);
  // set heading vector
  a = currentLaserPose.h + angle;
  p2.set(cos(a), sin(a), 0.0);
  p2.scale(maxR);
  p2.add(p1);
  // set laser line segment
  las.setFromPoints(p1, p2);
  ms = fakeMap;
  for (i = 0; i < fakeMapCnt; i++)
  {
    if (las.getSegmentCrossingXY(ms, &p2))
    {
      r = hypot((p2.x - p1.x)/cos(devicePose.Phi), p2.y - p1.y);
      if (r < range)
        range = r;
    }
    ms++;
  }
  // compensate if tilted laser can see the ground
  dh = sin(devicePose.Phi) * range * cos(angle);
  if (dh > devicePose.getZ())
    range *= devicePose.getZ() / dh;
  if (rangeError > 0.0 and range < maxR)
  {
    double dr = getPink(rangeError, 3);
    range += dr;
  }
  if (range < minR)
    range = 0.0;
  if (range > maxR)
  {
    if (minR > 0.001)
      range = 0.001;
    else 
      range = maxR;
  }
  return range;
}


double UFakeMap::getPink(double width, int times)
{
  double result = 0;
  for (int i=0; i < times; i++)
  {
    result += (random() / (double)RAND_MAX - 0.5) * width;
  }
  return result;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

ULaserData::ULaserData()
{
  valid = false;
  rangeCnt = 0;
  angleResolution = 1.0;
  angleStart = 0.0;
  unit = 1; // mm
  serial = 0;
  maxValidRange = 80.0;
  device = 0;
  fake = false;
}

/////////////////////////////////////////////

ULaserData::~ULaserData()
{
}

//////////////////////////////////////////////

//////////////////////////////////////////////

void ULaserData::print(char * prestring)
{
  const int MSL = 30;
  char s[MSL];
  //
  scanTime.getTimeAsString(s, true);
  printf("%s valid (%s), count=%d, start=%f, res=%f, at=%s\n",
         prestring, bool2str(valid),
         rangeCnt, angleStart, angleResolution, s);
}

/////////////////////////////////////////////

void ULaserData::copy(ULaserData * source)
{
  angleResolution = source->getAngleResolutionDeg();
  angleStart = source->getAngleStart();
  rangeCnt = source->getRangeCnt();
  memcpy(flags, source->getFlags(0), sizeof(int) * rangeCnt);
  memcpy(range, source->getRange(0), sizeof(int) * rangeCnt);
  scanTime = source->getScanTime();
  unit = source->getUnit();
  valid = source->isValid();
  serial = source->getSerial();
  maxValidRange = source->getMaxValidRange();
  fakePose = source->getFakePose();
  //fakeState = source->getFakeState();
  device = source->getDeviceNum();
  fake = source->isFake();
}

/////////////////////////////////////////////

void ULaserData::setSimData(double minAng,
                  double maxAng,
                  double resolution,
                  double minRange,
                  double maxRange,
                  UPose sourcePose,
                  UPosRot sensorPose,
                  int fake)
{
  setFakeDataRad(minAng * M_PI / 180.0,
                        maxAng * M_PI / 180.0,
                        resolution * M_PI / 180.0,
                        minRange,
                        maxRange,
                        sourcePose,
                        sensorPose,
                        fake);
}

////////////////////////////////////////////////////////

void ULaserData::setFakeDataRad(double minAng,
                                double maxAng,
                            double resolution,
                            double minRange,
                            double maxRange,
                            UPose sourcePose,
                            UPosRot sensorPose,
                            int fakeMode)
{
  int i;
  int n = roundi((maxAng - minAng) / resolution) + 1;
  int * r, *f;
  long int rnd;
  double rng, span, a;
//  double cosTilt;
  double sinTilt;
//  const double LASER_X_AT_SMR = 0.28; // position of laser
//  const double LASER_Y_AT_SMR = 0.14; // position of laser to the right (eurobot)
//  const double LASER_X_AT_MMR = 0.43; // position of laser
  UPose pose;
  double dh;
  //
  angleStart = minAng * 180. / M_PI;
  angleResolution = resolution * 180. / M_PI;
  rangeCnt = mini(maxi(1, n), MAX_RANGE_VALUES);
  unit = 1;
  r = range;
  f = flags;
  span = (maxRange - minRange) / 2.0;
  rng = span;
  fakePose = sourcePose;
  // adjust for laser scanner position
/*  if (fake == 2)
    pose.set(LASER_X_AT_SMR, 0.0, 0.0);
  else if (fake == 3)
    pose.set(LASER_X_AT_MMR, 0.0, 0.0);
  else if (fake == 4)
    pose.set(LASER_X_AT_SMR, LASER_Y_AT_SMR, 0.0);*/
//   pose.x = sensorPose.getX();
//   pose.y = sensorPose.getY();
//   pose.h = sensorPose.getKappa();
//   pose = sourcePose + pose;
//   cosTilt = cos(sensorPose.getPhi());
//   if (absf(cosTilt) < 1e-5)
//     // just to avoid NAN
//     cosTilt = 1e-5;
  sinTilt = sin(sensorPose.getPhi());
  //
  for (i = 0; i < rangeCnt ; i++)
  {
    switch (fakeMode)
    {
      case 0:
      case 1:
        a = minAng + resolution * i;
        rnd = random();
        rng += double(rnd)/double(RAND_MAX) * span - span/2.0;
        dh = sinTilt * rng * cos(a);
        //z = sensorPose.getZ() - sinTilt * rng * cos(a);
        if (dh > sensorPose.getZ())
          rng *= sensorPose.getZ() / dh;
        rng = mind(maxRange, maxd(minRange, rng));
        *r = roundi(rng * 1000.0);
        *f = 0;
        break;
      case 2:
      case 3:
      case 4:
        a = minAng + resolution * i;
        rng = fakeMap.getFake2range(a, maxRange, minRange);
//         dh = sinTilt * rng * cos(a);
//         //z = sensorPose.getZ() - sinTilt * rng * cos(a);
//         if (dh > sensorPose.getZ())
//           rng *= sensorPose.getZ() / dh;
//         rng = mind(maxRange, maxd(minRange, rng));
        // fix to fake URG to give 0.0 for max range
        if ((maxRange < 6.0) and (rng > 4.09))
          rng = 0.001;
        *r = roundi(rng * 1000.0);
        *f = 0;
        break;
      default:
        // just a constant range corresponding to fake value in cm
        *r = fakeMode * 10;
        *f = 0;
        break;
    }
    r++;
    f++;
  }
  serial++;
  fake = true;
  valid = true;
}

/////////////////////////////////////////////

// double ULaserData::getFake2range(UPose sourcePose, double angle, double maxR, double cosTilt)
// {
//   double range = maxR;
//   int i;
//   ULineSegment las, *ms;
//   UPosition p1, p2, p3;
//   double a, r = 0.0;
//   //
//   // set robot position
//   p1.set( sourcePose.x, sourcePose.y, 0.0);
//   // set heading vector
//   a = sourcePose.h + angle;
//   p2.set(cos(a), sin(a), 0.0);
//   p2.scale(maxR);
//   p2.add(p1);
//   // set line segment
//   las.setFromPoints(p1, p2);
//   ms = fakeMap.fakeMap;
//   for (i = 0; i < fakeMap.fakeMapCnt; i++)
//   {
//     if (las.getSegmentCrossingXY(ms, &p2))
//     {
//       r = hypot((p2.x - p1.x)/cosTilt, p2.y - p1.y);
//       if (r < range)
//         range = r;
//     }
//     ms++;
//   }
//   return range;
// }

/////////////////////////////////////////////

void ULaserData::setValue(int idx,
                          unsigned char lsb, unsigned char msb)
{
  range[idx] = (msb & 0x1f) * 256 + lsb;
  flags[idx] = msb >> 5;
}

/////////////////////////////////////////////

double ULaserData::getRangeMeter(int index, bool * rangeValid)
{
  int v;
  double result;
  //
  if ((index >= 0) and (index < rangeCnt))
  { // get integer value
    v = range[index];
    switch(getUnit())
    { // adjust to SI units
      case 0: result = double(v) * 0.01; break;
      case 1: result = double(v) * 0.001; break;
      case 2: result = double(v) * 0.1; break;
      default:result = 0.0; break;
    }
  }
  else
    result = 0.0;
  if (rangeValid != NULL)
    *rangeValid = (result > 0.02) and (result < (maxValidRange * 0.98));
  //
  return result;
}

///////////////////////////////////////////////////

U2Dpos ULaserData::get2d(int index)
{
  int v;
  double r, a;
  U2Dpos pos;
  //
  if ((index >= 0) and (index < rangeCnt))
  { // get integer value
    v = range[index];
    switch(getUnit())
    { // adjust to SI units
      case 0: r = double(v) * 0.01; break;
      case 1: r = double(v) * 0.001; break;
      case 2: r = double(v) * 0.1; break;
      default:r = 0.0; break;
    }
  }
  else
    r = 0.0;
  if ((r <= 0.02) or (r > (maxValidRange * 0.98)))
    r = 0.0;
  if (r < 0.02)
    pos.clear();
  else
  {
    a = getAngleRad(index);
    pos.x = r * cos(a);
    pos.y = r * sin(a);
  }
  //
  return pos;
}

///////////////////////////////////////////////////

bool ULaserData::saveToLogFile(FILE * logfile)
{
  bool result = (logfile != NULL);
  double rng;
  int n;
  //
  if (result)
  { // format sec.usec serial res startAng count range .... range
    fprintf(logfile,"%lu.%06lu %lu %g %g %d",
            scanTime.getSec(), scanTime.getMicrosec(),
            serial,
            angleResolution, angleStart, rangeCnt);
    for (n = 0; n < rangeCnt; n++)
    {
      rng = getRangeMetre(n);
      fprintf(logfile, " %.3f", rng);
    }
    fprintf(logfile, "\n");
  }
  //
  return result;
}

////////////////////////////////////////////////

void ULaserData::setMirror(bool value)
{
  if (value)
  { // set to mirror - start at positive angle and count down
    angleStart = fabs(angleStart);
    angleResolution = -1.0 * fabs(angleResolution);
  }
  else
  { // start on negative angle and count up
    angleStart = -1.0 * fabs(angleStart);
    angleResolution = fabs(angleResolution);
  }
}

////////////////////////////////////////////////

// UPoseTVQ ULaserData::fakeAdvancePose(int * state, UPoseTVQ startPose, double dt)
// {
//   UPoseTVQ pt;
//   double d;
//   //
//   scanTime.Now();
//   pt = startPose;
//   pt.t = scanTime;
//   pt.q = 1.0;
//       // move 10 cm and add a random heading change
//       //dh = double(random())/double(RAND_MAX) * 0.1 - 0.05;
//       //pt.add(0.02, dh);
//   d = fakeAdvanceControl(&pt, fakeMap.fakeTargetPose[*state], dt);
//   if (d < 0.8)
//   {
//     *state = (*state + 1) % fakeMap.fakeTargetPoseCnt;
//     printf("Faking to state %d:\n", *state);
//     // targetPose[state].print(" - ");
//   }
//   return pt;
// }
// 
// ////////////////////////////////////////////////
// 
// double ULaserData::fakeAdvanceControl(UPoseTVQ * currentPose, UPose targetPose, double dt)
// {
//   const double kP = 0.8;
//   const double kA = 1.0;
//   const double kB = -0.35;
//   const double maxV = 0.5;
//   const double maxA = 0.3;
//   double v, dv;
//   double w;
//   double d; // distance to target
//   double a; // angle to target
//   double b; // angle from direction to target to desired end direction
//   //
//   d = hypot(targetPose.x - currentPose->x, targetPose.y - currentPose->y);
//   a = atan2(targetPose.y - currentPose->y, targetPose.x - currentPose->x);
//   b = limitToPi(targetPose.h - a);
//   a = limitToPi(a - currentPose->h);
//   // velocity
//   v = kP * d;
//   v = fmin(maxV, v);
//   dv = v - currentPose->vel;
//   if (dv > maxA * dt)
//     dv = maxA * dt;
//   currentPose->vel += dv;
//   // rotation speed
//   w = kA * a + kB * b;
//   // advance
//   currentPose->add(currentPose->vel * dt, w * dt);
//   currentPose->t += dt;
//   //
//   return d;
// }

////////////////////////////////////////////////

double ULaserData::getMinAngleDeg()
{
  double m1 = getAngleDeg(0);
  double m2 = getAngleDeg(rangeCnt - 1);
  return mind(m1, m2);
}

////////////////////////////////////////////////

double ULaserData::getMaxAngleDeg()
{
  double m1 = getAngleDeg(0);
  double m2 = getAngleDeg(rangeCnt - 1);
  return maxd(m1, m2);
}

////////////////////////////////////////////////
