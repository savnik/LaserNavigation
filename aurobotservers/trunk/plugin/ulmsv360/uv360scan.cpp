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
#include "uv360scan.h"

UV360Scan::UV360Scan()
{
  angleMax = 90.0 * M_PI / 180.0;
  angleMin = -90.0 * M_PI / 180.0;
  angleVirtualMin = -M_PI;
  angleVirtualRes = M_PI / 180.0; // / 2.0;
  scanCnt = 360; // 720;
  next = 0;
  sourceSerial = 0;
  advanceScan();
  laserPose.set(0.28, 0.0 ,0.0);
  maxValidRange = 0.0;
  currentPose.clear();
}

/////////////////////////////////////////////////

UV360Scan::~UV360Scan()
{
}

/////////////////////////////////////////////////

bool UV360Scan::update(ULaserData * data, UPoseTime dataPose, U2Dpos notValid[])
{
  bool result = true;
  int i1, n1; // index to fresh data
  //int i2; // index to virtual scan sectors
  int i, j;
  double a1, ar, aa;
  double r;
  UV360Sect * sec, * secS, * secD;
  UV360Meas * me;
  UPose poseRel, tmpP1, tmpP2;
  UPosition pos;
  double a, ch, sh, lx, ly;
  U2Dpos fwdLeft, backRight, meas;
  // debug stat
  int n, m, k;
  UTime t;
  maxValidRange = data->getMaxValidRange();
  fwdLeft = notValid[0];
  backRight = notValid[1];
  // debug end
  //
  // fill in newest data
  a1 = data->getAngleStartRad();
  ar = data->getAngleResolutionRad();
  // get interval of new data
  if (a1 > angleMin)
  {
    i1 = 0; // index start for fresh data
    //i2 = getIndex(a1);
  }
  else
  {
    i1 = roundi((angleMin - a1)/ar);
    //i2 = getIndex(angleMin);
  }
  n1 = data->getRangeCnt();
  if (angleMax < (a1 + ar * n1))
    n1 = roundi(((a1 + ar * n1) - angleMin)/ar);
  //n2 = i2 + roundi(double(n1 - i1)*ar/angleVirtualRes);
  // clear data in the sector with fresh data
  // starting with first live angle
  sec = &scan[next];
  for (i = 0; i < scanCnt; i++)
  {
    sec->clear();
    sec++;
  }
  // set new data
  for (i = i1; i < n1; i++)
  { // these data are to be fed into sector
    r = data->getRangeMetre(i, NULL);
    if ((r > 0.02) and (r < maxValidRange))
    {
      aa = data->getAngleRad(i);
      sec = getSector(next, aa);
      meas.x = r * cos(aa);
      meas.y = r * sin(aa);
      if (meas.x > fwdLeft.x or meas.x <= backRight.x or
          meas.y > fwdLeft.y or meas.y <= backRight.y)
        // outside robot footprint
        sec->addMeas(r, aa, dataPose.t);
    }
  }
  // move newest scan to next
  // get pose of laserscanner in last scan
  tmpP1 = currentPose + laserPose;
  // get pose of laserscanner in in new scan
  tmpP2 = dataPose + laserPose;
  // get new laser scanner pose relative to last scan
  poseRel = tmpP1.getMapToPosePose(&tmpP2);;
  // coordinate conversion pre-calculation
  ch = cos(poseRel.h);
  sh = sin(poseRel.h);
  secS = &scan[newest]; // source sector
  n = 0;
  m = 0;
  k = 0;
  t.Now();
  for (i = 0; i < scanCnt; i++)
  {
    me = secS->getMeas(0);
    for (j = 0; j < secS->getMeasCnt(); j++)
    { // convert all old measurements to new robot coordinates
      //pos = poseRel.getMapToPose(me->getPos());
      pos = me->getPos();
      lx = pos.x - poseRel.x;
      ly = pos.y - poseRel.y;
      pos.x =  ch * lx + sh * ly;
      pos.y = -sh * lx + ch * ly;
      // get new angle to measurement
      a = atan2(pos.y, pos.x);
      n++; // count measurements for debug
      if ((a < angleMin) or (a > angleMax))
      { // this is inside the virtual sector - move the data
        r = hypot(pos.y, pos.x);
        // get destination sector
        secD = getSector(next, a);
        // add new measurement to segment
        if (not secD->addMeas(pos, r, a, me->getT()))
          k++; // no space to add this measurement
        m++;
      }
      me++;
    }
    // advance to next source sector
    secS++;
  }
  // debug
  //printf("UV360Scan::update: updated %d of %d points - %d hits limit (took %g ms)\n",
  //       m, n, k, t.getTimePassed() * 1000.0);
  // debug end
  // advance to new scan
  advanceScan();
  currentPose = dataPose;
  sourceSerial = data->getSerial();
  //
  return result;
}

//////////////////////////////////////////////////

UV360Sect * UV360Scan::getSector(int ref, double angle)
{
  UV360Sect * result;
  int n;
  //
  n = roundi((angle - angleVirtualMin)/angleVirtualRes);
  // debug
/*    if ((n < 0) or (n >= scanCnt))
      printf("UV360Scan::getSector: out of range\n");*/
  // debug end
  if (n < 0)
    n = 0;
  else if (n >= scanCnt)
    n = scanCnt - 1;
  result = &scan[ref + n];
  return result;
}

//////////////////////////////////////////////////

void UV360Scan::advanceScan()
{
  int n;
  //
  newest = next;
  n = newest + scanCnt;
  if  (n >= scanMaxCnt)
    n = 0;
  next = n;
}

//////////////////////////////////////////////////

int UV360Scan::getIndex(double angle)
{
  int result;
  //
  result = roundi((angle - angleVirtualMin)/angleVirtualRes);
  result = result % scanCnt;
  //
  return result;
}

//////////////////////////////////////////////////

bool UV360Scan::getScan(ULaserData * laserData)
{
  ULaserData * ld = laserData;
  int i;
  int * rp, *fp;
  UV360Sect * se;
  double r;
  const double maxrMM = 8.15 * 4.0;
  const double maxrCM = 8.15 * 20.0;
  const double maxrDM = 8.15 * 200.0;
  //
  ld->setAngleResAndStart(angleVirtualMin * 180.0 / M_PI, angleVirtualRes * 180.0 / M_PI);
  ld->setRangeCnt(scanCnt);
  ld->setScanTime(currentPose.t);
  ld->setMaxValidRange(maxValidRange);
  if (maxValidRange > 20)
    ld->setUnit(0); // cm
  else
    ld->setUnit(1); // mm
  se = &scan[newest];
  rp = ld->getRange(0);
  fp = ld->getFlags(0);
  for (i = 0; i < scanCnt; i++)
  {
    r = se->getRange();
    if (r < 0.02)
      r = 1.0e3;
    switch (ld->getUnit())
    {
      case 0: // cm
        *rp++ = roundi(mind(maxrCM, r) * 100.0);
        break;
      case 1: // mm
        *rp++ = roundi(mind(maxrMM, r) * 1000.0);
        break;
      default:  // dm
        *rp++ = roundi(mind(maxrDM, r) * 10.0);
        break;
    }
    *fp++ = 0;
    se++;
  }
  ld->setSerial(sourceSerial);
  ld->setValid(true);
  return true;
}

//////////////////////////////////////

int UV360Scan::getMeasurementCnt()
{
  int result = 0;
  UV360Sect * sc;
  int i;
  //
  sc = getNewestScan();
  for (i = 0; i < scanCnt; i++)
  {
    result += sc->getMeasCnt();
    sc++;
  }
  return result;
}

