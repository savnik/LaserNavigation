/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
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
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ulaserscan.h"
#include "uresobstacle.h"

ULaserScan::ULaserScan()
{
  clear();
//  scannerHeight = 0.86;
}

/////////////////////////////////////////////////////////

void ULaserScan::clear()
{
  valid = false;
  serial = 0;
  dataCnt = 0; //ORIGINAL_RAYS;
  //pisCnt = 0;
  //dataRes = 1.0;
}

/////////////////////////////////////////////////////////

// void ULaserScan::clearLinks()
// {
//   int i;
//   ULaserPi * pp;
//   //
//   pp = pis;
//   for (i = 0; i < pisCnt; i++)
//   {
//     pp->clearLinks();
//     pp++;
//   }
// }

/////////////////////////////////////////////////////////


// void ULaserScan::setPoseAndTime(UTime laserTime,
//                                 double odoX,
//                                 double odoY,
//                                 double odoTheta)
// {
//   time = laserTime;
//   odoPose.set(odoX, odoY, odoTheta);
// }

//////////////////////////////////////////////////////////////

// void ULaserScan::setScanPos(int index,
//                             double x, double y, double z, double rng)
// {
//   if ((index >= 0) and (index < ORIGINAL_RAYS))
//   {
//     data[index].setPos(x, y, z, rng);
//   }
// }

/////////////////////////////////////////////////////////////

// void ULaserScan::setScan(double * rng, int rngCnt, double laserTilt)
// {
//   ULaserPoint * lp;
//   int cnt = mini(rngCnt, ORIGINAL_RAYS);
//   double ang;
//   double * range;
//   int i;
//   //
//   lp = data;
//   range = rng;
//   for (i = 0; i < cnt; i++)
//   {  // change zero range (dazzeled??) to max-range
//     // as this otherwise will be a (unreasonable bad) obstacle
//     if (*range < 0.05)
//       *range = 8.5;
//     // convert to x,y-position
//     ang = (double(i) - 90.0) * M_PI / 180.0;
//     lp->setPos(ang, laserTilt, *range);
//     range++;
//     lp++;
//   }
// }

/////////////////////////////////////////////////////////////

void ULaserScan::setQ(int index, UPassQual value)
{
  if ((index >= 0) and (index < dataCnt))
  {
    data[index].setQ(value);
  }
}

/////////////////////////////////////////////////////////////

void ULaserScan::setQ(int from, int through,  UPassQual value)
{
  int i;
  ULaserPoint * pt;
  int m = mini(through + 1, dataCnt);
  //
  if ((from >= 0) and (from < dataCnt))
  {
    pt = &data[from];
    for (i = from; i < m; i++)
    {
      pt->setQ(value);
      pt++;
    }
  }
}

/////////////////////////////////////////////////////////////

// double ULaserScan::getVarHere(int index)
// {
//   int i;
//   ULaserPoint * p;
//   double result;
//   // first variance to the left of point
//   p = &data[index];
//   result = p->varL;
//   // find variance to the right
//   i = index - (p->varToL - index);
//   if (i >= 0)
//   {
//     p = &data[i];
//     if (p->varToL < index)
//     {
//       while (p->varToL < index)
//         p++;
//     }
//     else
//     {
//       while ((p->varToL > index) and (i > 0))
//         p--;
//     }
//   }
//   else
//     i = 0;
//   if (p->varL > 1e-7)
//   {
//     result += p->varL;
//     result /= 2.0;
//   }
//   return result;
// }

/////////////////////////////////////////////////////////////

double ULaserScan::getVarHere(int index)
{
  double vL, vR, result;
  //
  vL = getVarLeft(index);
  vR = getVarRight(index);
  if ((vL + vR) < 1.0)
    result = (vL + vR) / 2.0;
  else if (vL < 1.0)
    // left is invalid
    result = vR;
  else
    // right is invalid (and maybe also right)
    result = vL;
  //
  return result;
}

/////////////////////////////////////////////////////////////

double ULaserScan::getVarLeft(int index)
{
  double result = 1.0;
  ULaserPoint * p;
  //
  if (( index >= 0) and (index < dataCnt))
  {
    p = &data[index];
    result = p->varL;
  }
  return result;
}

/////////////////////////////////////////////////////////////

double ULaserScan::getVarRight(int index)
{
  int i;
  ULaserPoint * p;
  double result = 1.0; // default, if no data to the right;
  // first variance to the left of point
  if (( index >= 0) and (index < dataCnt))
  {
    p = &data[index];
    // find variance to the right
    // find most likely index 
    i = maxi(0, index - (p->varToL - index));
    // get this point
    p = &data[i];
    if (p->varToL < index)
    { // too far right
      while (p->varToL < index)
        p++;
      // the one back
      p--;
    }
    else
    { // too far left
      while ((p->varToL > index) and (i > 0))
        p--;
    }
    if (p->varL > 1e-7)
      result = p->varL;
  }
  return result;
}

/////////////////////////////////////////////////////////////

int ULaserScan::getRight(int index)
{
  int i = -1;
  ULaserPoint * p;
  // first variance to the left of point
  if (( index >= 0) and (index < dataCnt))
  {
    p = &data[index];
    // find variance to the right
    // find most likely index 
    i = maxi(0, index - (p->varToL - index));
    // get this point
    p = &data[i];
    if (p->varToL < index)
    { // too far right
      while (p->varToL < index)
      {
        p++;
        i++;
      }
      // the one back
      p--;
      i--;
    }
    else
    { // too far left
      while ((p->varToL > index) and (i > 0))
      {
        p--;
        i--;
      }
    }
  }
  return i;
}

/////////////////////////////////////////////////////////////

UPassQual ULaserScan::getQ(int index)
{
  UPassQual result = PQ_UNKNOWN;
  if ((index >= 0) and (index < dataCnt))
  {
    result = data[index].getQ();
  }
  return result;
}

//////////////////////////////////////////////////////////////

double ULaserScan::getRange(double angle)
{
  double result = 0.0;
  int idx;
  //
  if (dataCnt >= 180)
    // assumed 180 deg scan
    idx = roundi(angle * 180.0 / M_PI + 90.0);
  else
    // assumed 100 deg scan (not tested)
    idx = roundi(angle * 180.0 / M_PI + 50.0);
  if ((idx >= 0) and (idx < dataCnt))
    result = data[idx].range;
  return result;
}

//////////////////////////////////////////////////////////////

void ULaserScan::setValid(bool value)
{
  valid = value;
}


/////////////////////////////////////////////////////////////

void ULaserScan::setVariance(double width, int minCnt)
{ // using floating line-fit
  double sX = 0.0;
  double sY = 0.0;
  double sXY = 0.0;
  double sY2 = 0.0;
  double sX2 = 0.0;
  ULaserPoint *lpl, *lpr;
  int lil, lir; // laser point left and right index
  double w;
  int n, nr, valCnt, pointCnt, maxRangeCnt;
  int ndl = 1000, ndr = 1000;
  U2Dline lineX, line;
  UPosition posl, posr;
  double posld = 0.0, poslr = 0.0;
  double posrd = 0.0, posrr = 0.0;
  const int MPA = MAX_RANGE_DATA_CNT;
  float fx[MPA], fy[MPA];
  //float x1,y1, x2, y2;
  float varX, var; //, varI;
  double tilt, curv;
  int lastValid = 0;
  int leftValid = 0;
  //const double CURVATURE_WEIGHT = 0.005; // curvature weight relative to variance
  //
  lpr = data;
  lir = 0;
  lil = lir;
  lpl = lpr;
  // load start sum
  w = 0.0;
  n = 0;
  var = 1.0;
  pointCnt = 0;
  tilt = 0.0;
  curv = 0.0;
  // count the number of ilegal range values
  maxRangeCnt = 0;
  valCnt = 0;
  while (lir < (dataCnt - 1))
  { // continue until no more data
    // calculate new width
    if ((lpl->isValid()) and (lpr->isValid()))
    { // calculate new width
      w = lpr->pos.dist(lpl->pos);
      // get number of valid measurement points
      valCnt = pointCnt - maxRangeCnt;
      // calculate variance if enough points
      if (valCnt > (minCnt - 1))
      { // calculate variance - also if 1 too less
        line.setYline(sX, sY, sY2, sXY, valCnt);
        lineX.setXline(sX, sY, sY2, sXY, valCnt);
        nr = n - valCnt;
        var = line.variance(&fx[nr], &fy[nr], valCnt, NULL);
        varX = lineX.variance(&fx[nr], &fy[nr], valCnt, NULL);
        if (varX < var)
        { // fit on X line is best, so use this instead
          var = varX;
          line = lineX;
        }
        tilt = atan2(line.B(), line.A());
        // debug
        //line.print("line1");
        // line.set(&fx[nr], &fy[nr], valCnt, &var);
        //line2.print("line2");
        // debug end
        // debug
        //line.getOnLine(fx[nr], fy[nr], &x1, &y1);
        //line.getOnLine(fx[n-1], fy[n-1], &x2, &y2);
        // printf("V:: (cnt=%2d, max=%2d, r=%3d (%4.2f,%4.2f) l=%3d (%4.2f,%4.2f)"
        //       " rend(%4.2f, %4.2f), lend(%4.2f, %4.2f) v=%.5f, w=%.2f\n",
        //   pointCnt, maxRangeCnt,
        //   lir, fx[nr], fy[nr],
        //   lil, fx[n-1], fy[n-1],
        //   x1, y1, x2, y2, var, w);*/
        // debug end
        //
        // calculate curvature estimate
        // get a position approximately half way in the
        // left to right interval
        //lph = &lpr[pointCnt/2];
        //curv = 2.0 * lph->pos.x - lpr->pos.x - lpl->pos.x;
      }
      else
        var = 1.0; // big value
    }
    //
    if (((w < width) or (valCnt <= minCnt)) and
          (lil < dataCnt) and
          (lpr->isValid()))
    { // move left side until enough data
      //lpl->varL = 1.0; // initialize to high value
      //lpl->varToL = 180;
      if (lil > 0)
        lpl++;
      if (not lpl->isValid())
      { // count number of non-valid measurements in interval and since last valid
        maxRangeCnt++;
        ndl++;
      }
      else
      { // valid point - add values
        posld = (lpl->range - poslr)/ndl;
        poslr = lpl->range;
        ndl = 1;
        posl = lpl->pos;
        sX += posl.x;
        sY += posl.y;
        fx[n] = posl.x;
        fy[n] = posl.y;
        sXY += posl.x * posl.y;
        sY2 += sqr(posl.y);
        sX2 += sqr(posl.x);
        leftValid = lil;
        n++;
      }
      lil++;
      pointCnt++;
    }
    else
    { // move right side until widh is small enough and
      // right side has valid range
      if (not lpr->isValid())
      { // count number of non-valid measurements in interval and since last valid
        maxRangeCnt--;
        ndr++;
      }
      else
      { // subtrackt old values
        posrd = (lpr->range - posrr) / ndr;
        posrr = lpr->range;
        ndr = 1;
        posr = lpr->pos;
        sX -= posr.x;
        sY -= posr.y;
        sXY -= posr.x * posr.y;
        sY2 -= sqr(posr.y);
        sX2 -= sqr(posr.x);
        lastValid = lir;
      }
      // save result
      lpr->varLine = line;
      lpr->varL = var; // for area to left (until varToL)
      //lpr->varI = varI;
      lpr->varToL = leftValid; // leftmost valid point
      // save distance from fittet line to end points
      lpr->distLeft = line.distanceSigned(posl.x, posl.y);
      lpr->distRight = line.distanceSigned(posr.x, posr.y);
      // save inter-point distance (valid points only in range)
      lpr->distLeftPkt = posld;
      lpr->distRightPkt = posrd;
      // and tilt value for the area to the left
      lpr->tilt = tilt;
      // and curvature for the area to the left
      //lpr->curve = curv;
      // move to next
      lpr++;
      lir++;
      pointCnt--;
    }
  }
  // set high variance on left edge to the right
  // to give stop value of interval to the leftmost
  lpr = &data[lastValid];
  for (lir = lastValid; lir < dataCnt; lir++)
  { // mark as not passable
    lpr->varL = 1.0;
    //lpr->varI = 1.0;
    lpr->varToL = lastValid;
    lpr->distLeft = 1.0;
    lpr->distRight = 1.0;
    lpr->tilt = 0.0;
    //lpr->curve = 0.0;
    lpr++;
  }
  lpr = data;
  // calculate curvature
  //varI = maxIntegratedVariance;
  var = 1.0;
  for (lir = 0; lir < dataCnt - 1; lir++)
  {
    lil = (lpr->varToL + lir)/2;
    lpl = &data[lil];
    lpr++;
  }
}

/////////////////////////////////////////////////////////

ULineSegment ULaserScan::getLineSegmentFit(int left, int right,
                                           double * variance,
                                           UPosition * center)
{
  ULineSegment result;
  const int MAL = MAX_RANGE_DATA_CNT;
  float fx[MAL];
  float fy[MAL];
  bool ignore[MAL];
  int i, n;
  ULaserPoint * lp;
  float * pfx = fx;
  float * pfy = fy;
  bool * pIgn = ignore;
  UPosition p1, p2;
  int cnt;
  int ignCnt;
  float var, x, y;
  U2Dline line;
  UPosition pc;
  //
  lp = &data[right];
  cnt = left - right + 1;
  ignCnt = 0;
  for (i = 0; i < cnt; i++)
  {
    *pIgn = (lp->range >= LASER_MAX_RANGE);
    if (not *pIgn)
    {
      *pfx++ = lp->pos.x;
      *pfy++ = lp->pos.y;
      pc += lp->pos;
    }
    else
      ignCnt++;
    pIgn++;
    lp++;
  }
  n = cnt - ignCnt;
  if (n > 0)
  { // make linefit - at least one point
    line.set(fx, fy, cnt - ignCnt, &var);
    // get endpoints on line - candidates for ends
    line.getOnLine(fx[0], fy[0], &x, &y);
    p1.set(x, y , getPos(right).z);
    line.getOnLine(fx[n-1], fy[n-1], &x, &y);
    p2.set(x, y , getPos(left).z);
    // make result candidate (from right pointing left)
    result.setFromPoints(p1, p2);
  }
  else
  { // make a short line a single point (probably at max ramge)
    p1 = getPos(left);
    p2 = p1.scaled(2.0); // pointing away from robot
    // make result candidate (from right pointing left)
    result.setFromPoints(p1, p2);
    result.length = 0.05; // default length for one point
    var = 0.0;
    pc = p1;
  }
  // report other results
  if (variance != NULL)
    *variance = var;
  if (center != NULL)
  { // get center position
    if (n > 0)
      pc.scale(1.0/double(n));
    *center = pc;
  }
  //
  return result;
}

////////////////////////////////////////////////////////

int ULaserScan::countValidPoints(int fromThis, int upToThis)
{
  int result = 0;
  ULaserPoint * lp;
  int i;
  //
  lp = &data[fromThis];
  for (i = fromThis; i < upToThis; i++)
  {
    if (lp->isValid())
      result++;
    lp++;
  }
  return result;
}

////////////////////////////////////////////////////////

bool ULaserScan::findNearObstacles(UObstaclePool * obsts,
                                   UPoseTime odoPose,
                                   double maxRngL, double maxRngR, double minRange,
                                   bool outdoorContext,  bool horizontalScan, bool ignoreIfFixed,
                                  double searchExt)
{
  bool result = true;
  ULaserPoint *p1, *p2;
  bool gotStart = false;
  bool gotEnd = false;
  int first = 0, last = 0;
  const double MIN_OBST_SEP_OUTDOOR = 0.6; // range distance for obstacle declaration
  const double MIN_OBST_SEP_INDOOR = 0.2; // range distance for obstacle declaration
  double usedObstSep = MIN_OBST_SEP_INDOOR;
  int i, n = 0;
  double maxRng = maxRngR + searchExt;
  //
  p1 = data;
  p2 = p1;
  if (outdoorContext)
    usedObstSep = MIN_OBST_SEP_OUTDOOR;
  for (i = 0; i < dataCnt; i++)
  { // divide into left and right
    if (i == dataCnt / 2)
      maxRng = maxRngL + searchExt;
    if (not gotStart)
    {
      gotStart = (p1->getQ() >= PQ_ROUGH) and
          (p1->range < maxRng) and p1->isValid() and
          (p1->range > minRange);
      first = i;
    }
    else
    { // exclude objects touching robot
      gotEnd = (p1->getQ() < PQ_ROUGH) or
          (p1->range > maxRng) or
          (fabs(p1->range - p2->range) > usedObstSep) or
          (i == dataCnt - 1) or not p1->isValid();
      last = i;
    }
    if (outdoorContext and not horizontalScan)
    { // test for ditch openings
      if ((p1->getQ() < PQ_ROUGH) and
           ((p2->range - p1->range) > usedObstSep) and
           (p2->range < maxRng))
      { // right side:  a deep ditch at right of passable interval
        // add edge of passable interval as obstacle
        obsts->addObst(this, odoPose, i-1, i,
                       outdoorContext, horizontalScan, ignoreIfFixed);
        n++;
      }
      else if ((p2->getQ() < PQ_ROUGH) and
                ((p1->range - p2->range) > usedObstSep) and
                (p1->range < maxRng))
      { // left side: a deep ditch at left side of passable
        // add left edge of passable interval as obstacle
        obsts->addObst(this, odoPose, i-1, i, outdoorContext, horizontalScan, ignoreIfFixed);
        n++;
      }
    }
    if (gotEnd)
    {
      obsts->addObst(this, odoPose, first, last - 1,
                     outdoorContext, horizontalScan, ignoreIfFixed);
      // last may be a new start
      gotStart = (p1->getQ() >= PQ_ROUGH) and
          (p1->range < maxRng) and p1->isValid() and
          (p1->range > minRange);
      first = i;
      gotEnd = false;
      n++;
    }
    //
    p2 = p1;
    p1++;
  }
  // add potential last obstacle
  if (gotStart)
  {
    obsts->addObst(this, odoPose, first, dataCnt - 1,
                   outdoorContext, horizontalScan, ignoreIfFixed);
    n++;
  } 
  // remove all obstacles with 1 (or less) hits 
  //   and is older than this scan
  if (obsts->getGroupsCnt() > 0)
    obsts->getGroup(0)->removeInvalid(time, 1);
  //
  if (n == 0)
  { // no obstacles found, so just update grouping
    obsts->getObstGrp(odoPose);
    // debug
    //printf("ULaserScan::findNearObstacles: found no obstacles in scan %lu\n", getSerial());
    // debug end
  }
  obsts->obstDataUpdated(odoPose.t);
  //
  return result;
}

///////////////////////////////////////////////////////

bool ULaserScan::setScan(ULaserData * source, UPose odoPose, UPosRot *laserPose)
{
  int j, i, di;;
  ULaserPoint * lpt;
  double r, a, h, tilt;
  bool rngValid;
    
  lock();
  sensorPose = *laserPose;
  h = sensorPose.getZ();
  tilt = sensorPose.getPhi();
  robotPose = odoPose;
  setMaxValidRange(source->getMaxValidRange());
  lpt = getData(0);
  // ensure that first value is to the right
  // it does not matter much, but all the comments are arraged like this 
  if (source->getAngleStart() < 0.0)
  {
    i = 0;
    di = 1;
  }
  else
  {
    i = source->getRangeCnt() - 1;
    di = -1;
  }
  for (j = 0; j < source->getRangeCnt(); j++)
  {
    r = source->getRangeMeter(i, &rngValid);
    a = source->getAngleRad(i);
    lpt->setPosRA(r, a, tilt, h, rngValid);
    lpt++;
    i += di;
  }
  dataCnt = source->getRangeCnt();
  serial = source->getSerial();
  time = source->getScanTime();
  valid = true;
  unlock();
  return valid;
}
