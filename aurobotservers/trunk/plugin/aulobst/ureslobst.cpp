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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>
#include <urob4/usmltag.h>
#include <ugen4/uposrot.h>

#include "../../plugin/ulmspassable/uresobstacle.h"

#include "ureslobst.h"
#include "eransac.h"


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
#include <ulms4/ulaserdata.h>
#include <ulms4/ulaserdevice.h>

void UResLobst::UResLobstInit()
{
  setLogName("lobst");
  //openLog();
  // create status variables
  createBaseVar();
  verbose = false;
  obstGrpCnt = 0;
  lineListCnt = 0;
  pntCnt = 0;
}

///////////////////////////////////////////

UResLobst::~UResLobst()
{
}

///////////////////////////////////////////


void UResLobst::createBaseVar()
{
  varDistThreshold = addVar("maxToLineDist", 0.05, "d", "(r/w) max distance to line for members");
  varMinLineSupport = addVar("minPointsInLine", 8.0, "d", "(r/w) Minimum number of measurements in a line");
  varLineIter = addVar("maxLineIterations", 50.0, "d", "(r/w) Max number of iterations to allign line to members");
  varSamples = addVar("maxSamples", 20.0, "d", "(r/w) Number of samples to determine best supported line");
  varSplit = addVar("splitDist", 0.8, "d", "(r/w) distance to split long line into two (or more");
  varMinSplitCnt = addVar("minSplitCnt", 2.0, "d", "(r/w) if a line is grouped by 'splitDist', then each group must have at least this number of supporting point (set to 0 to disable).");
  varEnableSplit = addVar("enableSplit", 1.0, "d", "(r/w Enable split, i.e. reduce line to group with most measurements");
  varLineEatDist = addVar("lineEatDist", 0.1, "d", "(r/w) after line generation, regard measurements this close to the line as false measurements");
  varIgnoreIfFixed = addVar("ignoreIfFixed", 0.0, "d", "(r/w) ignore produced obstacles that overlap mapped fixed obstacles");
  varDebugDumpScanOn = addVar("debugDumpScanOn", 0.0, "d", "(rw) at this scan-number will the debugDump flag be set true");
  varDebugDumpScanOff = addVar("debugDumpScanOff", 0.0, "d", "(rw) at this scan-number will the debugDump flag be set false");
  varScan = addVar("scan", 0.0, "d", "(r) latest scan number processed");
  varNoObst = addVar("noObst", "0.25 0.17 -0.2 -0.17", "d", "(rw) Area where obstacles are ignored (below robot) front-left (x,y) and back-right (x,y) in robot coordinates");
  varLineCnt = addVar("lineCnt", 0.0, "d", "(r) number of detected ransac lines in last scan");
  varObstCnt = addVar("obstCnt", 0.0, "d", "(r) number of detected obstacles in last scan");
  addMethod("getRansacLines", "",  "Get array of estimated lines of type U2Dseg (in robot coordinates), first element of type UPoseTime with robot (odo) pose and detect time");
}

/////////////////////////////////////////////

bool UResLobst::methodCallV(const char * name, const char * paramOrder,
                                          UVariable * params[],
                                          UDataBase ** returnStruct,
                                          int * returnStructCnt)
{ // inter plug-in method call using UVariables as parameter and UDataBase as result type
  // ordinary parameters can be used both ways
 bool result = true;
 if ((strcasecmp(name, "getRansacLines") == 0) and (strlen(paramOrder) == 0))
 {
   if (*returnStructCnt > 0 and returnStruct != NULL and lineListCnt > 0)
   {
     UDataBase * b = NULL;
     U2Dseg * s;
     UPoseTime pt;
     GFLine * it;
     int m = mini(*returnStructCnt, lineListCnt + 1);
     UResPoseHist * odoHist = (UResPoseHist *) getStaticResource("odoPose", false, false);
     lock();
     if (odoHist != NULL)
       pt = odoHist->getPoseAtTime(scantime);
     pt.t = scantime;
     if (returnStruct[0] == NULL)
       returnStruct[0] = new UPoseTime(pt.x, pt.y, pt.h, pt.t);
     else if (returnStruct[0]->isA("posetime"))
       *(UPoseTime *)returnStruct[0] = pt;
     else
       printf("UResLobst::getRansacLines:callGlobalV expected first element in array to be a UPoseTime\n");
     // then the rest of the elements
     for (int i = 1; i < m; i++)
     {
       b = (UDataBase *) returnStruct[i];
       if (b == NULL)
       { // add a new object
         s = new U2Dseg();
         returnStruct[i] = s;
       }
       else if (b->isA("2Dseg"))
         s = (U2Dseg *)b;
       else
       { // wrong type
         printf("UResLobst::getRansacLines:callGlobalV Return struct pointer array holds wrong type of data - must be U2Dseg (or NULL)\n");
         m = 0;
         break;
       }
       it = &lineList[i - 1];
       s->setFromPoints(it->startX, it->startY, it->endX, it->endY);
     }
     *returnStructCnt = m;
     unlock();
   }
   else
     *returnStructCnt = 0;
 }
 else
   result = false;
 return result;
}

/////////////////////////////////////////////

const char * UResLobst::getList(const char * preStr, char * buff, const int buffCnt)
{
  const int MSL = 500;
  char s[MSL];
  char * p1 = buff;
  int n = 0;
  //
  strncat(p1, preStr, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  //
  snprintf(s, MSL, "ls %s\n", "(nothing)");
  strncpy(p1, s, buffCnt - n);
  return buff;
}

/////////////////////////////////////////////////

bool UResLobst::makeObst(ULaserData * scan, ULaserDevice * las)
{
  bool result = false;
  int i, rangeCnt, j;
  bool valid;
  double a,r, x, y;
  double csp; // laser tilt correction to range
  UPosRot lasPose;
  GFLine * it;
  LEL_GFParams params;
  double d, eatDist;
  U2Dseg seg;
  U2Dpos fl, br;
  //
  lasPose = las->getDevicePose();
  csp = cos(lasPose.Phi); // laser tilt
  rangeCnt = scan->getRangeCnt();
  scantime = scan->getScanTime();
  serial = scan->getSerial();
  pntCnt = 0;
  obstGrpCnt = 0;
  // set scannumber based debug flag
  if (int(scan->getSerial()) >= varDebugDumpScanOn->getInt() and
      (int(scan->getSerial()) < varDebugDumpScanOff->getInt() or
       varDebugDumpScanOff->getInt() == 0))
    setGlobalVar("avoid.debugDump", 1.0, false);
  if (int(scan->getSerial()) >= varDebugDumpScanOff->getInt() and
      varDebugDumpScanOff->getInt() > 0)
    setGlobalVar("avoid.debugDump", 0.0, false);
  // note current scan number
  varScan->setValued(scan->getSerial(), 0);
  // front left of area where no points should be used
  fl.set(varNoObst->getValued(0), varNoObst->getValued(1));
  // back right of area where no points should be used
  br.set(varNoObst->getValued(2), varNoObst->getValued(3));
  for(i = 0; i < rangeCnt; i++)
  { // convert to point array in robot coordinates
    // NB! do not take laser tilt into account
    a = scan->getAngleRad(i) + lasPose.Kappa;
    r = scan->getRangeMeter(i, &valid);
    if (valid)
    {
      x = cos(a) * r * csp + lasPose.x;
      y = sin(a) * r + lasPose.y;
      if (x > fl.x or x < br.x or y > fl.y or y < br.y)
      {
        pntX[pntCnt] = x;
        pntY[pntCnt] = y;
        pntUse[pntCnt] = -1;
        pntCnt++;
      }
    }
  }
  // set ransac parameters
  params.distThreshold = varDistThreshold->getDouble();
  params.sampleIter = varSamples->getInt();
  params.minLineSupport = varMinLineSupport->getInt();
  params.minNoOfPoints = params.minLineSupport + 1;
  params.lineIter = varLineIter->getInt();
  params.splitDist = varSplit->getDouble();
  params.minSplitCnt = varMinSplitCnt->getInt();
  params.enableSplit = varEnableSplit->getBool();
  //
  // find lines using ransac
  lineListCnt = ransac(&params, pntX, pntY, pntUse, pntCnt, lineList);
  varLineCnt->setInt(lineListCnt);
  // printf("Ransac results - found %d lines:\n", lineListCnt);
  //
  // remove assumed false measurements close to line
  eatDist = varLineEatDist->getDouble();
  for(i = 0; i < lineListCnt; i++)
  {
    it = &lineList[i];
    if (eatDist > params.distThreshold)
    { // remove measurements close to line
      seg.setFromPoints(it->startX, it->startY, it->endX, it->endY);
      for (j = 0; j < pntCnt; j++)
      {
        if (pntUse[j] == -1)
        {
          d = seg.getDistanceSigned(pntX[j], pntY[j], NULL);
          if (fabs(d) < eatDist)
          { // assign this measurement to the line
            // but do not recalculate the line
            pntUse[j] = i;
            // printf("Eat a measurment at %.2fx,%.2fy as part of line %d\n", pntX[j], pntY[j], i);
          }
          // debug
//           else if (fabs(d) < 0.5)
//           {
//             printf("Not Eat a measur at %.2fx,%.2fy as part of line %d\n", pntX[j], pntY[j], i);
//           }
        }
      }
    }
    // debug
/*    printf("%d %.4fA, %.4fB, %.4fC, start: %.2fx,%.2fy end:%.2fx,%.2fy, pntCnt:%d\n",
           i,
           it->A,it->B,it->C,
           it->startX,it->startY,it->endX,it->endY,it->edgeCount);*/
    // debug end
  }
  //
  if (true)
  {
    obstGrpCnt = makeObstGroups(params.splitDist, lineListCnt);
    varObstCnt->setInt(obstGrpCnt);
  }
  //
  return result;
}

/////////////////////////////////////////////////

char * UResLobst::codeLines(char * buff, int buffCnt)
{
  int i;
  GFLine * it;
  U2Dseg line;
  char * p1 = buff;
  int n = 0;
  //
  for(i = 0; i < lineListCnt; i++)
  {
    it = &lineList[i];
    line.setFromPoints(it->startX, it->startY, it->endX, it->endY);
    line.codeXml(NULL, p1, buffCnt - n, NULL);
    n += strlen(p1);
    p1 = &buff[n];
  }
  return buff;
}

///////////////////////////////////////////////////

int UResLobst::makeObstGroups(double splitDist, int firstCnt)
{
  const int MOG = 200;
  U2Dpos ogPos[MOG];
  U2Dpos * p1;
  int ogCnt = 0;
  int ogHit[MOG];
  int i, j;
  double d;
  //
  for (i = 0; i < pntCnt; i++)
  { // test all measurement points
    if (pntUse[i] == -1)
    { // not used for ransac lines
      for (j = ogCnt - 1; j >= 0; j--)
      { // match with established groups.
        // try the last - and most likely - group first.
        p1 = &ogPos[j];
        d = hypot(pntY[i] - p1->y, pntX[i] - p1->x);
        if (d < splitDist)
        { // no split, so add
          ogHit[j]++;
          // save most recent position
          p1->x = pntX[i];
          p1->y = pntY[i];
          // mark as used for group j
          pntUse[i] = j + firstCnt;
          break;
        }
      }
      if (pntUse[i] == -1)
      { // no group for this point - add new group
        p1 = &ogPos[ogCnt];
        p1->x = pntX[i];
        p1->y = pntY[i];
        pntUse[i] = ogCnt + firstCnt;
        ogHit[ogCnt] = 1;
        if (ogCnt < MOG)
          ogCnt++;
      }
    }
  }
  // debug
/*  printf("Found %d groups of non-line obstacles\n", ogCnt);
  for (i = 0; i < ogCnt; i++)
  {
    p1 = &ogPos[i];
    printf(" - %d, %dhits end at %.2fx,%.2fy\n", i + firstCnt, ogHit[i], p1->x, p1->y);
  }*/
  // debug end
  return ogCnt;
}

///////////////////////////////////////////////////

bool UResLobst::sendAsObstacles()
{
  UResObstacle * resObst;
  bool isOK;
  int i, j, n;
  GFLine * it;
  UPolygon400 poly400;
  UPolygon * poly = &poly400;
  UPolygon40 poly40;
  UResPoseHist * odoHist;
  UPoseTime pose;
  UPosition p1, p2;
  bool updated = false;
  //
  resObst = (UResObstacle *) getStaticResource("obst", false, false);
  odoHist = (UResPoseHist *)    getStaticResource("odoPose", false, false);
  isOK = resObst != NULL and odoHist != NULL;
  if (not isOK)
    printf("No obstacle resource - load ulmspassable.so.0 (or pass if static)\n");
  if (isOK)
  {
    pose = odoHist->getPoseAtTime(scantime);
    pose.t = scantime;
    for (i = 0; i < lineListCnt; i++)
    { // first lines
      it = &lineList[i];
      poly->clear();
      p1 = pose.getPoseToMap(it->startX, it->startY);
      p2 = pose.getPoseToMap(it->endX, it->endY);
      poly->add(p1);
      poly->add(p2);
      poly->setAsPolygon();
      resObst->addObst(poly, pose, false, true, varIgnoreIfFixed->getBool());
    }
    for (i = 0; i < obstGrpCnt; i++)
    {
      poly = &poly400;
      poly->clear();
      n = i + lineListCnt;
      for (j = 0; j < pntCnt; j++)
      {
        if (pntUse[j] == n)
        { // convert to odometry coordinates
          p1 = pose.getPoseToMap(pntX[j], pntY[j]);
          poly->add(p1);
        }
      }
      if (poly->getPointsCnt() > 2)
      { // need to be in the right order and convex
        poly->extractConvexTo(&poly40);
        poly = &poly40;
      }
      if (poly->getPointsCnt() > 0)
      {
        // debug
/*        p1 = poly->getCogXY();
        printf("Obstacle generation %d cog=%.2fx,%.2fy - area is %.2fm2 - pntCnt=%d\n", i, p1.x, p1.y, poly->getXYarea(), poly->getPointsCnt());*/
        // debug end
        updated |= resObst->addObst(poly, pose, false, true, varIgnoreIfFixed->getBool());
        // debug
        // printf("Added group %d with %d points to resObst\n", i, poly->getPointsCnt());
        // debug end
      }
    }
    if (updated)
      resObst->obstDataUpdated(pose.t);
    isOK = obstGrpCnt > 0 or lineListCnt > 0;
  }
  return isOK;
}

