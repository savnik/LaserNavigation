/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#include <math.h>

#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>

#include "urespassable.h"
#include "uobstaclepool.h"

void UResPassable::UResPassableInit()
{ // these first two lines is needed
  // to save the ID and version number
/*  setResID(getResID());
  resVersion = getResVersion();*/
  // other local initializations
  addPassableIntervalParameters();
  // this sensorPose may be changed in the future to
  // be part of the laser scanner pool
  sensorPose = new UPosRot();
  // set default MMR laser scanner position
  sensorPose->set(0.44, 0.0, 0.41, 0.0, 9 * M_PI / 180.0, 0.0);
  // create scan buffer
  scan = new ULaserScan();
  iHeight = 0;
  iMaxVar = 0;
  iEndDist = 0;
  iInside = 0;
  iMeasureDist = 0;
//  fixeds = new UObstacleGroup();
}

///////////////////////////////////////////

UResPassable::~UResPassable()
{
  delete sensorPose;
//  delete fixeds;
}

///////////////////////////////////////////

void UResPassable::addPassableIntervalParameters()
{
  if (getVarPool() == NULL)
    createVarSpace(20, 0, 0, "Laser scanner based road and obstacle detection", false);
  //
  addVar("version", getResVersion()/100.0, "d", "Resource version");
  varRoughSDLimit = addVar("roughSDLimit", 0.1, "d", "SD limit ~ 0.1 (hard limit of SD)");
  varRoughWidth = addVar("roughWidth", 0.45, "d", "Roughness filter distance - about robot wheel base");
  varRoughMinCnt = addVar("roughMinCnt", 3.0, "d", "Minimum number of measurements in a roughness interval");
  varConvexFac = addVar("convexFac", 0.0075, "d", "Factor to disfavor concave road");
  varConvexOffset = addVar("convexOffset", 0.005, "d", "Offset [m] to disfavor concave road");
  varFitZTiltLimit = addVar("fitZTiltLimit", 10.0 * M_PI / 180.0, "d", "In y-z coordinates");
  varFitXTiltLimit = addVar("fitXTiltLimit", 75.0 * M_PI / 180.0, "d", "In y-x coordinates");
  varFitEndDev = addVar("fitEndDev", 4.5, "d", "Distance in SD to be outside current traversable segment");
  varUseSmoothSettings = addVar("useSmoothSettings", 1.0, "d", "Boolean to increase edge sensitivity");
  varUseIntervalCombiner = addVar("useIntervalCombiner", 1.0, "d", "Combine or not combine intervals");
  varMinCombinedWidth = addVar("minCombinedWidth", 0.7, "d", "Minimum width for a traversable segment (combined)");
  //varUseOutdoorObst = addVar("useOutdoorObst", 1.0, "d", "Use roules that apply to outdoor obstacles (as opposed to indoor)");
  varMinObstDist = addVar("minObstDist", 0.1, "d", "(rw) Obstacles close to scanner is assumed to "
      "be the robot sun-screen or other non-relevant features.");
  varIgnoreIfFixed = addVar("ignoreIfFixed", 0.0, "d", "(r/w) Ignore obstacles that overlap fixed (mapped) obstacles.");
}

///////////////////////////////////////////////////////

void UResPassable::getSettingsFromVarPool()
{
  //UVarPool * vp;
  //double v;
  //
  //vp = getVarPool();
  //if (vp != NULL)
  {
    // vp->getLocalValue("convexOffset", &lineFitConvexOffset);
    lineFitConvexOffset = varConvexOffset->getValued();
    //if (vp->getLocalValue("roughSDLimit", &v))
    //  lineFitVarLimit = sqr(v);
    lineFitVarLimit = sqr(varRoughSDLimit->getValued());
    //vp->getLocalValue("roughWidth", &lineFitWidth);
    lineFitWidth = varRoughWidth->getValued();
    // if (vp->getLocalValue("roughMinCnt", &v))
    //   lineFitMinCnt = roundi(v);
    lineFitMinCnt = varRoughMinCnt->getInt();
    //vp->getLocalValue("convexFac", &lineFitConvexFac);
    lineFitConvexFac = varConvexFac->getValued();
    // vp->getLocalValue("fitZTiltLimit", &lineFitZTiltLimit);
    lineFitZTiltLimit = varFitZTiltLimit->getValued();
    // vp->getLocalValue("fitXTiltLimit", &lineFitXTiltLimit);
    lineFitXTiltLimit = varFitXTiltLimit->getValued();
    // vp->getLocalValue("fitEndDev", &lineFitEndpointDev);
    lineFitEndpointDev = varFitEndDev->getValued();
    // if (vp->getLocalValue("useSmoothSettings", &v))
    //  lineSmoothSettings = (v > 0.5);
    lineSmoothSettings = varUseSmoothSettings->getBool();
    // if (vp->getLocalValue("useIntervalCombiner", &v))
    //  lineUseCombine = (v > 0.5);
    lineUseCombine = varUseIntervalCombiner->getBool();
    // if (vp->getLocalValue("useOutdoorObst", &v))
    //   outdoorObsts = (v > 0.5);
    // outdoorObsts = varUseOutdoorObst->getBool();
    // vp->getLocalValue("minCombinedWidth", &minCombinedSegmentWidth);
    minCombinedSegmentWidth = varMinCombinedWidth->getValued();
  }
}

///////////////////////////////////////////

const char * UResPassable::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  char * p1;
  int i, n;
  //
  snprintf(buff, buffCnt, "%s found %d passable intervals\n", preString, pisCnt);
  n = strlen(buff);
  p1 = &buff[n];
  for (i = 0; i < pisCnt; i++)
  {
    pis[i].print("    ", p1, buffCnt - n);
    n += strlen(p1);
    p1 = &buff[n];
  }
  return buff;
}

///////////////////////////////////////////

bool UResPassable::setScan(ULaserData * source, UPose odoPose, UPosRot * laserPose)
{
  bool result;
  double tilt, hgt;
  //
  lock();
  *sensorPose = *laserPose;
  tilt = sensorPose->getPhi();
  hgt = sensorPose->getZ();
  result = scan->setScan(source, odoPose, sensorPose);
  unlock();
  return result;
}

/////////////////////////////////////////////

bool UResPassable::addPassableInterval(const int right,
                                     const int left,
                                     double varMin,
                                     double varMin2)
{ // NB! first OK point is right last is left (both inclusive)
  bool result = false;
  ULaserPi * pp;
  //
  /**
  @todo ULaserScan::addPassableInterval overwriting last interval if list is full; should be better than that? */
  if (pisCnt < MAX_PASSABLE_INTERVALS_PR_SCAN)
  {
    pisCnt++;
    result = true;
  }
  else
    printf("ULaserScan::addPassableInterval: too many intervals (%d)\n", pisCnt);
  //
  // not space, overwrite last interval
  pp = &pis[pisCnt - 1];
  pp->setInterval(right, left, scan, varMin, varMin2);
  // mark area as rough
  scan->setQ(right, left, PQ_ROUGH);
  //
  // debug
/*  printf("addPassableInterval #%02d (scan %lu) right=%d left=%d (width=%.2f m) SD=%.4f SD2=%.4f\n",
         pisCnt, scan->getSerial(), right, left, pp->getWidth(),
        sqrt(pp->getVarMin()), sqrt(pp->getVarMin2()));*/
  // debugEnd
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////

int UResPassable::makePassableIntervals2(const int rightLim, const int leftLim)
{
  int result = 0;
  int i, j;
  ULaserPoint *pr;
  ULaserPoint *pl; // pointer to left test point
  ULaserPoint *pll; // pointer to last left test point
  ULaserPoint *par;
  int piLeft, piRight; // left and right index
  int paRight, paLeft; // actual interval limits
  bool passable;
  double w, d;
  double varMin = 1.0;
  double varMin2 = 1.0;
  double varTest = 0.0;
  const double LASER_MEARUREMENT_VARIANCE = sqr(0.01); // in meter^2
  double varMinLim = LASER_MEARUREMENT_VARIANCE;
  double varStop = 1.0;
  UPosition p;
  // max allowed separation of measurements
  const double MAX_MEASUREMENT_INTERVAL_DIST = 0.8;
//  const double VAR_MIN2_LIMIT = 2; // minimum measurements for valid varince type 2 (no min limit)
  double d2p; // distance to previous measurement
  bool limHeight, limMaxVar, limEndDist, limInside, limMeasureDist;
  bool doContinue;
  //
  //
  // set default as not passable
  scan->setQ(rightLim, leftLim - 1, PQ_NOT);
  //
  pr = scan->getData(rightLim);
  if (lineSmoothSettings)
    varMinLim = LASER_MEARUREMENT_VARIANCE;
  else
    varMinLim = LASER_MEARUREMENT_VARIANCE * 2.0;
  pl = pr;
  i = rightLim;
  passable = false;
  piRight = 0;
  piLeft = -1; // just to avoid warnings
  while (i < leftLim)
  { // test for terrain structure
    if (not passable)
    { // look for start of passable using max limit
      passable = (pr->varL < lineFitVarLimit) and     // within variance limit ...
          (absd(pr->tilt) < lineFitXTiltLimit) and // and tilt OK
          (pr->pos.z < MAX_ALLOWED_HEIGHT) and // and not too high ...
          (pr->pos.z > MIN_ALLOWED_HEIGHT);    // or too low
      if (passable)
      { // right edge of a potential area is found
        piRight = i;
        varMinLim = LASER_MEARUREMENT_VARIANCE;
        varMin = maxd(pr->varL, varMinLim);
        varMin2 = varMin; // 1.0;
        // initialize looking for left side of interval
        piLeft = i;
        pl = scan->getData(i);
      }
    }
    //
    if (passable)
    {
      pll = pl;
      pl = scan->getData(i);
      // look for left side of interval using adaptive limit
      varTest = mind(varMin, lineFitVarLimit)  * lineFitEndpointDev;
      // save index for (adaptive) left limit
      paLeft = scan->getData(piLeft)->varToL;
      // get compensated distance for last measurement to the left of interval
      d = pr->distLeft - lineFitConvexOffset;
      // get distance to previous measurement
      if (pll->isValid() and pl->isValid())
        d2p = hypot(pl->pos.x - pll->pos.x, pl->pos.y - pll->pos.y);
      else
        d2p = 0.0;
      // debug
      //if (pr->pos.z >= MAX_ALLOWED_HEIGHT)
      //  pr->pos.z = pr->pos.z + 0.01;
      // debug end
      limHeight = (pr->pos.z < MAX_ALLOWED_HEIGHT) and (pr->pos.z > MIN_ALLOWED_HEIGHT);
      limMaxVar = pr->varL < lineFitVarLimit;
      limEndDist = sqr(d) < varTest;
      limInside  = paLeft <= leftLim;
      limMeasureDist = d2p < MAX_MEASUREMENT_INTERVAL_DIST;
      // debug
      doContinue = (limMeasureDist and limInside and limEndDist and limMaxVar and limHeight);
      if (not doContinue)
      {
        if (not limMeasureDist)
          iMeasureDist++;
        if (not limInside)
          iInside++;
        if (limEndDist)
          iEndDist++;
        if (limMaxVar)
          iMaxVar++;
        if (limHeight)
          iHeight++;
      }
      // debug end
      // test for end of passable interval
      if ((pr->varL < lineFitVarLimit) and  // max line-fit variance
           (sqr(d) < varTest) and      // fit of left-most point adapted
           (paLeft <= leftLim) and     // inside test interval
           (absd(pr->tilt) < lineFitXTiltLimit) and // line tilt OK
           (pr->pos.z < MAX_ALLOWED_HEIGHT) and // item height not too high ...
           (pr->pos.z > MIN_ALLOWED_HEIGHT) and
           (d2p < MAX_MEASUREMENT_INTERVAL_DIST)) // or too low
      { // still passable
        if (pr->isValid())
          piLeft = i; // left-most place to search for a right edge
        // adapt limit of line-fit
        if (pr->varL < varMin)
          varMin = maxd(pr->varL, varMinLim);
        if ((pr->varL < varMin2)) // and ((pr->varToL - i) > VAR_MIN2_LIMIT))
          varMin2 = pr->varL;
      }
      else
      { // (adaptive) left end of interval is found
        // look from here to the right for adaptive right limit
        paRight = piLeft;
        // get position of left side of interval
        p = scan->getData(paLeft)->pos;
        par = scan->getData(paRight); // first guess of adapted right edge
        w = 0.0; // reset width
        for (j = paRight; j >= piRight; j--)
        { // look for right edge with this variance.
          // calculate passable width
          w = p.dist(par->pos);
          // get distance relative to line compensated
          // to favor convex shaped road
          d = par->distRight - lineFitConvexOffset;
          varStop = par->varL;
          //
          limMaxVar = pr->varL < lineFitVarLimit;
          limEndDist = sqr(d) < varTest;
          doContinue = (limMeasureDist and limInside and limEndDist and limMaxVar and limHeight);
          if (not doContinue)
          {
            if (limEndDist)
              iEndDist++;
            if (limMaxVar)
              iMaxVar++;
          }
          //
          if ((par->varL <= lineFitVarLimit) and  // max line-fit variance limit
               (sqr(d) < varTest))// and           // last right-most point (adaptive)
            paRight = j;
          else
            // right edge is found
            break;
          // go further right
          par--;
        }
        // left is left side of last good interval
        // missing test for big (in-line) jumps in untested interval
        // from liLeft to paLeft
        pl = scan->getData(piLeft);
        for (j = piLeft + 1; j <= paLeft; j++)
        {
          pll = pl;
          pl = scan->getData(j);
          if (pll->isValid() and pl->isValid())
          { // test only if measurements are valid
            d2p = hypot(pl->pos.x - pll->pos.x, pl->pos.y - pll->pos.y);
            if (d2p >= MAX_MEASUREMENT_INTERVAL_DIST)
            { // stop before the jump
              paLeft = j - 1;
              break;
            }
          }
        }
        addPassableInterval(paRight, mini(paLeft, leftLim), varMin, varMin2);
        //
        result++;
        // look for new start
        i = paLeft;
        pr = scan->getData(i);
        passable = false;
        // an interval is found (or failed), but there may be another
        // to the right of the tested interval
        if ((paRight - piRight) > MIN_MEASUREMENTS_PER_PI)
        { // test this interval
          result += makePassableIntervals2(piRight, paRight - 1);
        }
        // continue with next interval
      }
    }
    i++;
    //    prOld = pr;
    pr++;
  }
  return result;
}


//////////////////////////////////////////////////////////

int sortPis(const void * pp1, const void * pp2)
{
  int result;
  ULaserPi * pip1 = (ULaserPi *) pp1;
  ULaserPi * pip2 = (ULaserPi *) pp2;
  //
  if (pip1->getRight() > pip2->getRight())
    result = 1;
  else if (pip1->getRight() < pip2->getRight())
    result = -1;
  else
    result = 0;
  return result;
}

/////////////////////////////////////////////////

bool UResPassable::combineNearIntervals(double obstSize, double lowVarLimit)
{
  ULaserPi *pp1, *pp2;
  int i, k;
  const int PI_NEAR_LIMIT = MIN_MEASUREMENTS_PER_PI;
  bool result = false;
  bool combined;
  double avgX, distLimit;
  double xl, xr, xl2, xr2;
  double v1, v2;
  double vm1, vm2;
  double a1, a2, ad;
  double sg1l, sg2l;
//  double sgLen;
//  ULaserPi * ppis[MAX_PASSABLE_INTERVALS_PR_SCAN];
//  const double LOW_VAR_LIMIT = 0.00005;
  //  /** max difference in variance for two intervals */
//  const double varVarianceFactor = 1.0;
//  const double varVarLengthFactor = 0.5;
//  double vf;
  // sort intervals from right to left
/*  for (i = 0; i < pisCnt; i++)
  ppis[i] = &pis[i];*/
  qsort(pis, pisCnt, sizeof(ULaserPi), sortPis);
  // combine near intervals
  pp1 = pis;
  v1 = pp1->getFitVariance();
  vm1 = pp1->getVarMin();
  a1 = atan2(pp1->getSegment()->vec.y, pp1->getSegment()->vec.x);
  sg1l = pp1->getSegment()->length;
  pp2 = pp1 + 1;
  i = 0;
  while (pp2 < &pis[pisCnt])
  { // test right of pp1 and left of pp2
    combined = false;
    v2 = pp2->getFitVariance();
    vm2 = pp2->getVarMin();
    a2 = atan2(pp2->getSegment()->vec.y, pp2->getSegment()->vec.x);
    sg2l = pp2->getSegment()->length;
    if (abs(pp2->getRight() - pp1->getLeft()) < PI_NEAR_LIMIT)
    { // combine possible to the left of pp1
      // get average x-value
      xl = scan->getPos(pp1->getLeft()).x;
      xr = scan->getPos(pp2->getRight()).x;
      avgX = (xl + xr)/2.0;
      distLimit = obstSize * avgX / sensorPose->getZ(); //scannerHeight;
      // dist limit is for small obstacles
      // level steps must be smaller
      combined = fabs(xl - xr) < distLimit;
      if (combined)
      { // test also the ends of the fitted line
        xl2 = pp1->getSegment()->getOtherEnd().x;
        xr2 = pp2->getSegment()->pos.x;;
        combined = fabs(xl2 - xr2) < distLimit;
      }
      // test min half-robot-width variance difference
      if (combined)
      { // test if step in variance is less than a factor 2
        // if (fmax(vm1, vm2) > fmax(fmin(vm1, vm2), lowVarLimit/9.0) * 2.0)
        if (fmax(vm1, vm2) > fmax(fmin(vm1, vm2), lowVarLimit) * 1.8)
          combined = false;
      }
      if (combined)
      { // test angle and angle difference of segments
        ad = limitToPi(a1 - a2);
        if ((ad > (60 * M_PI / 180.0)) or
             (ad < (-20 * M_PI / 180.0)) or
             (fabs(a1) < 30 * M_PI / 180.0) or
             (fabs(a1) > 150 * M_PI / 180.0) or
             (fabs(a2) < 30 * M_PI / 180.0) or
             (fabs(a2) > 150 * M_PI / 180.0))
          combined = false;
      }
      if (combined)
        // test measurements inbetween
        for (k = pp1->getLeft() + 1; k < pp2->getRight(); k++)
      {
        combined  = (avgX - scan->getPos(k).x) < distLimit;
        if (not combined)
          break;
      }
      if (combined)
        pp1->setInterval(pp1->getRight(), pp2->getLeft(),
                         scan,
                         mind(pp1->getVarMin(), pp2->getVarMin()),
                         mind(pp1->getVarMin2(), pp2->getVarMin2()));
    }
    // use data from left-most interval as new base (combined or not)
    v1 = v2;
    vm1 = vm2;
    a1 = a2;
    sg1l = sg2l;
    if (combined)
    { // combined, so move the remaining intervals down
      for (k = i + 2; k < pisCnt; k++)
        pis[k - 1] = pis[k];
      pisCnt--;
    }
    else
    { // not combined - so move on to next
      pp1++;
      pp2++;
      i++;
    }
  }
  return result;
}

//////////////////////////////////////////////////

bool UResPassable::findTopOfRoad(ULaserPi * pip)
{
  bool result;
  int j;
  ULineSegment * seg;
  double d = 0.0; //, dl, dr;
  const int MAX_TOP_CNT = 10;
  double topHgt[MAX_TOP_CNT];
  int top[MAX_TOP_CNT];
  int topCnt;
  ULaserPoint * pt; //, *pl, *pr;
  const double TOP_LIMIT = sqr(0.5); // times line-fit variance
  const double MIN_TOP_LIMIT = 0.04; // distance (in x) from fit-line
  bool topValid;
  bool isOK;
  const double MAX_TOP_RANGE = 4.5; // laser range [m]
  const double TOP_DIST_FROM_END = 0.4; // meter
  double minTopHgt;
  double td;
  double topVarMin;
  UPosition topPos;

  // set edges and top
  result = (pip != NULL);
  if (result)
  {
    seg = pip->getSegment();
    result = (absd(pip->getTilt()) < lineFitZTiltLimit);
  }
  if (result)
  { // find center of road
    // minimum value for valid height
    minTopHgt = maxd(pip->getFitVariance() * TOP_LIMIT, MIN_TOP_LIMIT);
    // get first values to test
    pt = scan->getData(pip->getRight());
//    pl = pt + 4;
//    pr = pt - 4;
    // initialize top candidate array
    topCnt = 0;
    topHgt[topCnt] = -1.0;
    top[topCnt] = -1;
    // find center position candidates
    for (j = pip->getRight(); j <= pip->getLeft(); j++)
    { // set classification to passable
      pt->setQ(PQ_EASY);
      isOK = ((j > (pip->getRight() + 4)) and
          (j < (pip->getLeft() - 4)));
      if (isOK)
      { // valid data set
        d = seg->getXYsignedDistance(pt->pos);
//        dl = seg->getXYsignedDistance(pl->pos);
//        dr = seg->getXYsignedDistance(pr->pos);
        if ((d > topHgt[topCnt]) and //(d > dl) and (d > dr) and
             (pt->range < MAX_TOP_RANGE) and
             (d > minTopHgt))
        { // negative is closer (and higher)
          topHgt[topCnt] = d;
          top[topCnt] = j;
        }
        if ((d < 0.0) and (top[topCnt] >= 0))
        { // top has ended, and ready to find next
          if (topCnt < MAX_TOP_CNT - 1)
          {
            topCnt++;
            topHgt[topCnt] = -1.0;
            top[topCnt] = -1;
          }
        }
      }
      pt++;
//      pl++;
//      pr++;
    }
    // find best top - the one with the lowest variance
    // -- that is lowest variance in the area to the left of the top
    // -- (should possibly be the minimum or average of left and right side variance)
    if (top[topCnt] >= 0)
      topCnt++;
    topValid = (topCnt > 0);
    if (topValid)
    {
      topVarMin = 10.0;
      for (j = 0; j < topCnt; j++)
      {
        pt = scan->getData(top[j]);
        if (pt->varL < topVarMin)
        {
          topVarMin = pt->varL;
          topPos = pt->pos;
        }
      }
    }
    if (topValid)
    {
      td = seg->getPositionOnLine(&topPos);
      topValid = td > TOP_DIST_FROM_END;
    }
    if (topValid)
      topValid = td < (seg->length - TOP_DIST_FROM_END);
    if (not topValid)
      // use center position
      td = seg->length / 2.0;
    pip->setCenter(td, topValid);
    // and analyze edges
  }
  return result;
}

//////////////////////////////////////////////////////

ULaserPi * UResPassable::findPi(int withThisIdx)
{
  ULaserPi * result;
  int i;
  //
  result = pis;
  for (i = 0; i < pisCnt; i++)
  {
    if ((result->getLeft() >= withThisIdx) and
         (result->getRight() <= withThisIdx))
      break;
    result++;
  }
  if (i >= pisCnt)
    result = NULL;
  //
  return result;
}

//////////////////////////////////////////////////////


int UResPassable::makePassableIntervalsFit(
//                        bool outdoorObsts,
                        double * maxPassLeft, double * maxPassRight,
                        FILE * logo,
                        FILE * fdI1,
                        FILE * fdI2
                        )
{
  ULaserPi * pp;
  int i;
  ULineSegment * seg;
  int ppOKCnt = 0;
  const double MAX_OBST_HEIGHT = 0.05; // meter
  const double VAR_LIMIT_ROUGH = sqr(0.05);
  const double VAR_LIMIT_SMOOTH = sqr(0.01);
  double r;
  //
  // produce raw intervals
  makePassableIntervals2(0, scan->getDataCnt());
  //
  // debug
  if (fdI1 != NULL)
  {
    for (i = 0; i < pisCnt; i++)
      fprintf(fdI1, "%lu %d %d %d\n", scan->getSerial(), i, pis[i].getRight(), pis[i].getLeft());
  }
  /*
  r = iHeight + iMaxVar + iEndDist + iMeasureDist + iInside;
  printf("makePassableIntervalsFit n=%d  height=%.1f%% "
      "maxVar=%.1f%% endDist=%.1f%% measDist=%.1f%% inside=%.1f%%\n",
      int(r), 100.0*iHeight/r, 100.0*iMaxVar/r,
      100.0*iEndDist/r, 100.0*iMeasureDist/r, 100.0*iInside/r);
  */
  // debug end
  //
  if (lineUseCombine)
  {
    if (lineSmoothSettings)
      // fine terrain, so avoid horse-shit and appels.
      combineNearIntervals(MAX_OBST_HEIGHT / 2.0, VAR_LIMIT_SMOOTH);
    else
      // rough terrain - allow larger obstacles (stones, leaves etc.)
      combineNearIntervals(MAX_OBST_HEIGHT, VAR_LIMIT_ROUGH);
  }
  //
  // set edges and top for all intervals found
  // and test for valid tilt
  pp = pis;
  *maxPassRight = 0.0;
  *maxPassRight = 0.0;
  for (i = 0; i < pisCnt; i++)
  {
    if ((pp->getLeft() - pp->getRight()) > MIN_MEASUREMENTS_PER_PI)
    {
      seg = pp->getSegment();
      // test for interval tilt
      if (((absd(pp->getTilt()) < lineFitZTiltLimit) and (seg->length > minCombinedSegmentWidth)) or not lineUseCombine)
      { // find top of road position
        findTopOfRoad(pp);
        // and analyze edges (a bit depricated -- soon)
        //pp->findEdgeObstacles(logo);
        // forn max range for passable interval
        r = scan->getData(pp->getLeft())->range;
        if (pp->getLeft() < (scan->getDataCnt() / 2))
        { // right side
          if (r > *maxPassRight)
            *maxPassRight = r;
        }
        else
        { // left side
          if (r > *maxPassLeft)
            *maxPassLeft = r;
        }
        r = scan->getData(pp->getRight())->range;
        if (pp->getRight() < (scan->getDataCnt() / 2))
        { // right side
          if (r > *maxPassRight)
            *maxPassRight = r;
        }
        else
        { // left side
          if (r > *maxPassLeft)
            *maxPassLeft = r;
        }
        // save all good segments
        if (i > ppOKCnt)
          pis[ppOKCnt] = *pp;
        ppOKCnt++;
      }
    }
    pp++;
  }
  pisCnt = ppOKCnt;
  //
  //
  // debug
  if (fdI2 != NULL)
  { // save intervals after combination
    for (i = 0; i < pisCnt; i++)
      fprintf(fdI2, "%lu %d %d %d\n", scan->getSerial(), i, pis[i].getRight(), pis[i].getLeft());
  }
  // debug end
  //
  // debug
  //printf("ULaserScan::makePassableIntervalsFit found %d OK from %d pi's\n",
  //  ppOKCnt, pisCnt);
  // debug end
  return pisCnt;
}


////////////////////////////////////////////////////////

bool UResPassable::doFullAnalysis(ULaserData * laserData, UPose pose,
                                  UPosRot laserPose,
                                  UObstaclePool * obsts, bool outdoorContext,
                                  double obstSearchExt,
                                 UResRoadLine * roads)
{
  double maxPassLeft;
  double maxPassRight;
  UPoseTime odoPose;
  double maxLineRange;
  //
  // debug
  //printf("doFullAnalysis scan %lu\n", laserData->getSerial());
  // debug end
  // get a fresh set of analysis oparameters
  getSettingsFromVarPool();
  // load sensor data into scan
  setScan(laserData, pose, &laserPose);
  // calculate variance along scan
  scan->setVariance(lineFitWidth, lineFitMinCnt);
  // divide into segments
  pisCnt = 0;
  makePassableIntervalsFit(&maxPassLeft, &maxPassRight, NULL, NULL, NULL);
  // pose for obstacle correlation
  odoPose.setPt(pose, laserData->getScanTime());
  // update road lines
  if (roads != NULL)
  {
    maxLineRange = laserData->getMaxValidRange() * 0.8;
    roads->update(laserData->getSerial(), pis, pisCnt, odoPose, &laserPose, maxLineRange);
  }
  // do not look for obstacles near max range (last 20% is deemed unreliable for obstacles
  if ((maxPassLeft + obstSearchExt) > (laserData->getMaxValidRange() * 0.8))
    maxPassLeft = laserData->getMaxValidRange() * 0.8 - obstSearchExt;
  if ((maxPassRight + obstSearchExt) > (laserData->getMaxValidRange() * 0.8))
    maxPassRight = laserData->getMaxValidRange() * 0.8 - obstSearchExt;
  // find and save obstacles in scan
  if (obsts != NULL)
    scan->findNearObstacles(obsts, odoPose, maxPassLeft, maxPassRight, varMinObstDist->getValued(),
                            outdoorContext, false, varIgnoreIfFixed->getBool(), obstSearchExt);
  else
    printf("UResPassable::doFullAnalysis Missing obstacle pool structure\n");
  return true;
}

////////////////////////////////////////////////////////

bool UResPassable::doObstAnalysis(ULaserData * laserData, UPose pose,
                                  UPosRot laserPose,
                                  UObstaclePool * obsts, bool outdoorContext)
{
  double maxPassLeft;
  double maxPassRight;
  UPoseTime odoPoseOrigin, mapPose, odoPose;
  int n;
  UVariable vpo1, vpo2;
//  UVariable * vpar[2] = {&vpo1, &vpo2};;
  UResPoseHist * mapPoseHist;
  //
  // get mapped obstacles in odometry coordinates
  mapPoseHist = (UResPoseHist*) getStaticResource("mapPose", false);
  if (mapPoseHist != NULL)
  { // use odo pose origin
    mapPose = mapPoseHist->getNewest();
    odoPoseOrigin = mapPoseHist->getOdoPoseOrigin();
    n = 1;
    vpo1.setPose(&mapPose);
    vpo2.setPose(&odoPoseOrigin);
//    fixeds->clear();
    // get near obstacles in odometry coordinate system
//    isOK = callGlobalV("mapobst.getobst", "cc", vpar, (UDataBase**)&fixeds, &n);
  }
  // get a fresh set of analysis oparameters
  getSettingsFromVarPool();
  // load sensor data into scan
  setScan(laserData, pose, &laserPose);
  // pose for obstacle correlation
  odoPose.setPt(pose, laserData->getScanTime());
/*  if (isOK)
    obsts->setFixedObstacles(fixeds);*/
  // do not look for obstacles near max range (last 20% is deemed unreliable for obstacles
  maxPassLeft = laserData->getMaxValidRange() * 0.9;
  maxPassRight = maxPassLeft;
  // find and save obstacles in scan
  scan->findNearObstacles(obsts, odoPose, maxPassLeft, maxPassRight, varMinObstDist->getValued(),
                            outdoorContext, true, varIgnoreIfFixed->getBool(), 0.0);
  return true;
}

