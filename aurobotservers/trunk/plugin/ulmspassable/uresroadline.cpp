/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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

#include "uresroadline.h"
#include "ulaserscan.h"

/**
Road class with up to three road lines, and at least a center line.
Primarily used to hold the current road. */
class URoad
{
public:
  /** set road quality */
  void setRoadQ(double preferredWidth)
  {
/*    double currentQ;
    double distQ;
    double alignQ;
    double widthQ;
    double validQ;
    double qQ;*/
    const double maxDist = 8.0;
    int n = 3;
    double dd;
    // current
    currentQ = 1.0;
    if (not currentCenter)
      currentQ *= 0.95;
    if (not currentLeft)
      currentQ *= 0.95;
    if (not currentRight)
      currentQ *= 0.95;
    // valid
    if (idxLeft < 0)
      n--;
    if (idxRight < 0)
      n--;
    validQ = pow(0.25, 3-n);
    // distance
    distQ = 1.0 / (1.0 + 0.1 * fabs(distC / maxDist));
    if (idxLeft >= 0)
      distQ *= 1.0 / (1.0 + 0.1 * fabs(distL - preferredWidth/2.0));
    if (idxRight >= 0)
      distQ *= 1.0 / (1.0 + 0.1 * fabs(distR + preferredWidth/2.0));
    // alignment
    if ((idxLeft >= 0) and (idxRight >= 0))
    {
      dd = rlLeft->getLine()->vec.dot(rlRight->getLine()->vec);
      alignQ = 1.0 - 0.5 * (1.0 - dd);
    }
    else
      alignQ = 0.9;
    if (idxLeft >= 0)
    {
      dd = rlCenter->getLine()->vec.dot(rlLeft->getLine()->vec);
      alignQ *= 1.0 - 0.25 * (1.0 - dd);
    }
    if (idxRight >= 0)
    {
      dd = rlCenter->getLine()->vec.dot(rlRight->getLine()->vec);
      alignQ *= 1.0 - 0.25 * (1.0 - dd);
    }
    // width
    widthQ = 0.9;
    if ((idxLeft >= 0) and (idxRight >= 0))
      width = distL - distR;
    else if (idxLeft >= 0)
      width = mind(maxDist, 2.0 * (distL - distC));
    else if (idxRight >= 0)
      width = mind(maxDist, 2.0 * (distC - distR));
    else
      width = maxDist;
    widthQ = 1.0 / (1.0 + 0.5 * fabs(width - preferredWidth));
    // quality of road lines
    qQ = rlCenter->getQual();
    n = 1;
    if (idxLeft >= 0)
    {
      qQ +=  rlLeft->getQual();
      n++;
    }
    if (idxRight >= 0)
    {
      qQ +=  rlRight->getQual();
      n++;
    }
    qQ /= double(n);
    // pisQ
    pisQ = 1.0;
    if ((idxLeft >= 0) and (idxRight >= 0))
    {
      if (not (rlLeft->getPisIdx() == rlRight->getPisIdx()))
        pisQ = 0.95;
    }
    else if (idxLeft >= 0)
    {
      if (not (rlLeft->getPisIdx() == rlCenter->getPisIdx()))
        pisQ = 0.95;
    }
    else if (idxRight >= 0)
    {
      if (not (rlRight->getPisIdx() == rlCenter->getPisIdx()))
        pisQ = 0.95;
    }
    // total
    qualQ = currentQ * validQ * distQ * alignQ * widthQ * pisQ * qQ;
  };


public:
  /** index to relevant road line - is -1 if not valid */
  int idxLeft, idxRight, idxCenter;
  /** distance from robot to line */
  double distL, distR, distC;
  /** if roadline current */
  bool currentLeft, currentRight, currentCenter;
  /** pointer to road line */
  URoadLine * rlLeft, *rlRight, *rlCenter;
  /** road width */
  double width;
  /** road qualified quality */
  double qualQ;
  /** is road valid */
//  bool valid;
  /** is road current */
//  bool current;
  /** quality sub-values */
  double currentQ;
  double distQ;
  double alignQ;
  double widthQ;
  double validQ;
  double pisQ;
  double qQ;
};

//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////

/**
Class that colds details of candidate line correlation */
class UEdgeLinePair
{
public:
  /**
  Set data for a candidate */
  bool setPair(int lin, ULineSegment * pline, double estVar, int updates,
               int pis, UPosition * pos, double maxSD)
  {
    bool result;
    //
    line = lin;
    pi = pis;
    lineUpds = updates;
    pisPos = *pos;
    d2 = pline->getDistanceSq(&pisPos);
    result = sqrt(d2 / estVar) < maxSD;
    return result;
  };
  /**
  Compare two candidates */
  const int compare(const UEdgeLinePair * other)
  {
    int result;
    double a, b;
    a = d2 + d2 / (1.0 + lineUpds);
    b = other->d2 + other->d2 / (1.0 + other->lineUpds);
    if (a < b)
      result = -1;
    else
      result = 1;
    return result;
  };

  int line;
  int pi;
  double d2;
  double lineUpds;
  UPosition pisPos;
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

/**
Suport class to save a list of road line candidates */
class UEdgeList
{
public:
  /**
  Constructor */
  UEdgeList()
  {
    cnt = 0;
  };
  /**
  Clear candidate list */
  void clear()
  {
    cnt = 0;
  };
  /**
  Get qualified quality compensated for distance and current quality */
  double getQualifiedQual(double q, double dist, double idx, int currentIdx)
  {
    const double favorCurrentQualFactor = 0.95;
    const double favorCloseMinDist = 0.5; // meter
    const double favorCloseQualPerMeter = 0.02;
    double q1;
    //
    q1 = q;
    if (idx != currentIdx)
      q1 *= favorCurrentQualFactor;
    if (fabs(dist) > favorCloseMinDist)
      q1 -= (fabs(dist) - favorCloseMinDist) * favorCloseQualPerMeter;
    if (q1 < 0.0)
      q1 = 0.0;
    return q1;
  }
  /**
  Add a new candidate and keep the list in priority order */
  void add (double d, int N, double q, int rIdx, int currentIdx)
  { // better Q has higher priority
    int i;
    double q1;
    //
    q1 = getQualifiedQual(q, d, rIdx, currentIdx);
    if (cnt == 0)
    {
      dist[0] = d;
      upds[0] = N;
      qual[0] = q;
      idx[0] = rIdx;
      qualQ[0] = q1;
    }
    else
    {
      for (i = cnt; i >= 0; i--)
      {
        if ((i > 0) and (q1 > qualQ[i - 1]))
        { // move one level up
          if (i < MCC)
          { // move old one down
            dist[i] = dist[i - 1];
            upds[i] = upds[i - 1];
            qual[i] = qual[i - 1];
            qualQ[i] = qualQ[i - 1];
            idx[i] = idx[i - 1];
          }
        }
        else
        {
          if (i < MCC)
          {
            dist[i] = d;
            upds[i] = N;
            qual[i] = q;
            qualQ[i] = q1;
            idx[i] = rIdx;
          }
          break;
        }
      }
    }
    if (cnt < MCC)
      cnt++;
  };
  /**
  Get best edge candidate */
  int getBest(URoadLine * current, int currentIdx, double minD, double maxD)
  {
    int result = -1;
    int m;
//    double q0, q1;
    //
/*    if (cnt == 1)
      // just one candidate, so use it
      result = idx[0];
    else if (cnt >= 1)
    { // prefer number two if current and quality not too bad.
      q0 = getQualifiedQual(0, currentIdx);
      q1 = getQualifiedQual(1, currentIdx);
      if (q0 > q1)
          m = 1;
    }*/
/*    if (m >= 0)
    { // a best is available
      if ((dist[m] < minD) or (dist[m] > maxD))
      { // the selected is not qualified*/
    m = 0;
    while (m < cnt)
    { // look for the best that is qualified
      if ((dist[m] < minD) or (dist[m] > maxD))
        m++;
      else
        break;
    }
//      }
    if (m < cnt)
      result = idx[m];
    else
        // none qualified
      result = -1;
//    }
    return result;
  }
  /** max number of candidates */
  static const int MCC = 5;
  /** current count of candidates */
  int cnt;
  /** distance from robot to line */
  double dist[MCC];
  /** number of updates the line has received */
  int upds[MCC];
  /** line quality [0..1] */
  double qual[MCC];
  /** line quality [0..1] qualified for distance and current road selection */
  double qualQ[MCC];
  /** line index */
  int idx[MCC];
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

void UResRoadLine::UResRoadLineInit()
{ // these first two lines is needed
  // to save the ID and version number
//  setResID(getResID());
//  resVersion = getResVersion();
  // other local initializations
  const int MSL = 500;
  char s[MSL];
  //
  addRoadlineParameters();
  bestLine = 0;
  roadsCnt = 0;
  roads[0].clear(0);
  roadLeftIdx = -1;
  roadRightIdx = -1;
  roadCenterIdx = -1;
  roadLog = NULL;
  snprintf(s, MSL, "%s/road.log", dataPath);
  roadLog = fopen(s, "w");
  roadSerial = 0;
}

///////////////////////////////////////////

UResRoadLine::~UResRoadLine()
{
  if (roadLog != NULL)
    fclose(roadLog);
}

///////////////////////////////////////////

void UResRoadLine::addRoadlineParameters()
{
  addVar("version", getResVersion()/100.0, "d", "Resource version");
  addVar("roadType", -1.0, "d", "Road type (0=unknown, 1 = asphalt, 2=gravel)");
  addVar("roadTypeVar", -1.0, "dq", "Road type quality");
  addVar("roadTypeVarQ", -1.0, "dq", "");  // road type quality
  addVar("correlationSD", 2.0, "d", "max number of SD for road update correlation");
  addVar("expectedRoadWidth", 3.0, "d", "Favored road width when estimating current road");
  varPoolIdx[0] = addVar("left", 0.0, "dq", "Distance to left roadside");  // left distance
  varPoolIdx[1] = NULL; //addVar("leftQ", 0.0, "dq", "");  // left distance quality
  varPoolIdx[2] = addVar("leftN", 0.0, "d", "Left updates count");
  varPoolIdx[3] = addVar("leftSD", 0.0, "d", "left standardDev");
  varPoolIdx[4] = addVar("center", 0.0, "dq", "Distance to center of road");
  varPoolIdx[5] = NULL; //addVar("centerQ", 0.0, "dq", "");
  varPoolIdx[6] = addVar("centerN", 0.0, "d", "Center updates count");
  varPoolIdx[7] = addVar("centerSD", 0.0, "d", "Center standard deviation");
  varPoolIdx[8] = addVar("right", 0.0, "dq", "distance to right road line");
  varPoolIdx[9] = NULL; //addVar("rightQ", 0.0, "dq", "");
  varPoolIdx[10] = addVar("rightN", 0.0, "d", "Right updates count");
  varPoolIdx[11] = addVar("rightSD", 0.0, "d", "right standard deviation");
  varPoolIdx[12] = addVar("distSensorAdvance", 1.0, "d", "Dist sensor (left, cent, right) from robot");
}

////////////////////////////////////////////////////////

// void UResRoadLine::getSettingsFromVarPool()
// {
//   UVarPool * vp;
//   //
//   vp = getVarPool();
//   if (vp != NULL)
//   {
//     vp->getLocalValue("gain", &gain);
//   }
// }

///////////////////////////////////////////

const char * UResRoadLine::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  double ql, qc, qr;
  //
  ql = 0.0;
  qc = 0.0;
  qr = 0.0;
  if (getLeftIdx() >= 0)
    ql = roads[getLeftIdx()].getQual();
  if (getCentIdx() >= 0)
    qc = roads[getCentIdx()].getQual();
  if (getRightIdx() >= 0)
    qr = roads[getRightIdx()].getQual();
  snprintf(buff, buffCnt, "%s currently %d roads best(L=%.2f C=%.2f R=%.2f)\n",
           preString, roadsCnt, ql, qc, qr);
  return buff;
}

///////////////////////////////////////////

int compareRLC(const void * a, const void * b)
{
  UEdgeLinePair * aa = (UEdgeLinePair*)a;
  UEdgeLinePair * bb = (UEdgeLinePair*)b;
  return aa->compare(bb);
}

//-------------------------------------------

void UResRoadLine::update(unsigned long scan, ULaserPi * pis, int pisCnt,
                          UPoseTime pose, UPosRot * sensorPose, double maxLineRange)
{
  const int MPI = MAX_PASSABLE_INTERVALS_PR_SCAN;
  const int MCC = MAX_ROAD_LINES * MPI;
  UEdgeLinePair best[MCC];
  int bestCnt = 0;
  UEdgeLinePair * bc;
  const double MAX_CORR_SD_S_DEFAULT = 2.5; // number of SD's to allow correlation
  double corrSD;
  ULaserPi * pp;
  int i, j, m, k;
  UPosition p1, pos;
  URoadLine * pr;
  bool used[MPI][3]; // is line segment edge used
  UMatrix4 mLtoR; // sensor to robot coordinates
  UMatrix4 mRtoM; // robot to map
  UMatrix4 mLtoM; // all the way
  UPosRot rob;
  bool moving;
  // not moving limits
  const double moveDist = 0.03;
  const double moveHeading = 1.0 * M_PI / 180.0;
  double dh;
  //
  // make coordinate conversion matric from sensor position to odo-map
  mLtoR = sensorPose->getRtoMMatrix();
  rob.setFromPose( pose.x, pose.y, pose.h);
  mRtoM = rob.getRtoMMatrix();
  mLtoM = mRtoM * mLtoR;
  // set moving flag
  dh = limitToPi(odoPose.h - pose.h);
  moving = (hypot(odoPose.x - pose.x, odoPose.y - pose.y) > moveDist) or (fabs(dh) > moveHeading);
  if (not getLocalValue("correlationSD", &corrSD))
    corrSD = MAX_CORR_SD_S_DEFAULT;
  // find correlation candidates
  bc = best;
  pp = pis;
  for (i = 0; i < pisCnt; i++)
  { // Find best use of all intervals
    for (m = 0; m < 3; m++)
    { // and all three points on a passable interval
      switch (m)
      { // get edge position
        case 0: p1 = pp->getLeftPos(); break;
        case 1: p1 = pp->getCenterPos(); break;
        case 2: p1 = pp->getRightPos(); break;
      }
      if (p1.dist() > maxLineRange)
        // not valid as line source
        continue;
      pos = mLtoM * p1;
      pr = roads;
      for (j =0; j < roadsCnt; j++)
      { // try on all estanblished lines
        if (pr->isValid() and pr->isA(m))
        { // line is valid and of the right type
          if (bc->setPair(j, pr->getLine(), pr->getEstVar(), pr->getUpdateCnt(), i, &pos, corrSD))
          {
            bc++;
            bestCnt++;
            if (bestCnt >= MCC)
              break;
          }
        }
        pr++;
      }
      if (bestCnt >= MCC)
        break;
    }
    pp++;
    if (bestCnt >= MCC)
      break;
  }
  // evaluate for each road line
  // sort so that best correlations are first
  qsort(best, bestCnt, sizeof(UEdgeLinePair), &compareRLC);
  //
  for (i = 0; i < pisCnt; i++)
    for (m=0; m < 3; m++)
      used[i][m] = false;
  // now update from best association
  bc = best;
  for (k = 0; k < bestCnt; k++)
  {
    i = bc->pi;
    j = bc->line;
    pr = &roads[j];
    if (pr->getScanSerial() != scan)
    {
      m = pr->getLineType();
      if (not used[i][m])
      { // update with this position
        //pp = &pis[i];
        pr->update(bc->pisPos, pose, scan, moving, i);
        used[i][m] = true;
      }
    }
    bc++;
  }
  // create new for unused edges
  for (i = 0; i < pisCnt; i++)
  {
    for (m = 0; m < 3; m++)
    {
      if (not used[i][m])
      { // find empty slot
        pr = roads;
        j = 0;
        while (pr->isValid() and (j < MAX_ROAD_LINES))
        {
          pr++;
          j++;
        }
        if (not pr->isValid())
        {
          pp = &pis[i];
          switch (m)
          { // get edge position
            case 0: p1 = pp->getLeftPos(); break;
            case 1: p1 = pp->getCenterPos(); break;
            case 2: p1 = pp->getRightPos(); break;
          }
          pos = mLtoM * p1;
          pr->clear(m);
          pr->setNew(pos, pose, scan, i, roadSerial++);
          if (j >= roadsCnt)
          { // increase line count
            roadsCnt++;
            // and clear next entry
            if (roadsCnt < MAX_ROAD_LINES)
              roads[roadsCnt].clear(0);
          }
        }
      }
    }
  }
  //
  //
  postUpdate(scan, &pose, moving);
}

///////////////////////////////////////////

void UResRoadLine::postUpdate(unsigned int scan, UPoseTime * pose, bool moving)
{
  URoadLine * road;
  int i;
  //
  road = roads;
  for (i = 0; i < roadsCnt; i++)
  {
    if (road->isValid())
      road->postUpdate(scan, pose, moving);
    road++;
  }
  // find current road
  //findCurrentRoad(pose);
  findCurrentRoad2(pose);
}

///////////////////////////////////////////

int compareRdFull(const void * a, const void * b)
{
  int result = 0;
  URoad * aa = (URoad*)a;
  URoad * bb = (URoad*)b;
  if (aa->qualQ < bb->qualQ)
    result = 1;
  else if (aa->qualQ > bb->qualQ)
    result = -1;
  return result;
}

//-----------------------------------------

void UResRoadLine::findCurrentRoad2(UPoseTime * pose)
{
  URoadLine *rll, *rlc, *rlr;
  URoad * rd;
  const int MRC = 200;
  URoad rdFull[MRC];
  int rdFullCnt;
  int c, l, r;
  const int MLC = MAX_ROAD_LINES;
  double d[MLC];
  UPosition rPos;
  const int minUpdCnt = 4;
  double expectedRoadWidth;

  //
  if (not getLocalValue("expectedRoadWidth", &expectedRoadWidth))
    expectedRoadWidth = 3.0;
  rdFullCnt = 0;
  rPos.set(pose->x, pose->y, 0.0);
  rlc = roads;
  for (c = 0; c < roadsCnt; c++)
  {
    d[c] = -rlc->getLine()->getXYsignedDistance(rPos);
    rlc++;
  }
  rd = rdFull;
  rlc = roads;
  for (c = 0; c < roadsCnt; c++)
  {
    if (rlc->isA(1) and (rlc->getUpdateCnt() >= minUpdCnt))
    { // is a center line
      rll = roads;
      for (l = 0; l < roadsCnt; l++)
      { // find a matching left line
        if ((rll->isA(0) and (d[l] > d[c]) and (rll->getUpdateCnt() >= minUpdCnt))
             or (l == c))
        { // is a candidate (or dummy for no left line (l==c))
          rlr = roads;
          for (r = 0; r < roadsCnt; r++)
          {
            if ((rlr->isA(2) and (d[r] < d[c]) and (rlr->getUpdateCnt() >= minUpdCnt))
                 or (r == c))
            { // is a candidate (or dummy for no right (r == c)
              if (rdFullCnt < MRC)
              {
                rd->idxCenter = c;
                rd->distC = d[c];
                rd->rlCenter = rlc;
                if (l == c)
                {
                  rd->idxLeft = -1;
                  rd->distL = 1e5;
                  rd->rlLeft = NULL;
                }
                else
                {
                  rd->idxLeft = l;
                  rd->distL = d[l];
                  rd->rlLeft = rll;
                }
                if (r == c)
                {
                  rd->idxRight = -1;
                  rd->distR = 1e5;
                  rd->rlRight = NULL;
                }
                else
                {
                  rd->idxRight = r;
                  rd->distR = d[r];
                  rd->rlRight = rlr;
                }
                rd->currentCenter = (c == roadCenterIdx);
                rd->currentLeft = (l == roadLeftIdx);
                rd->currentRight = (r == roadRightIdx);
                rdFullCnt++;
                rd++;
              }
              else
                printf("Road list FULL!! (more than %d! from %d/%d source lines)\n",
                       rdFullCnt, c, roadsCnt);
            }
            rlr++;
          }
        }
        rll++;
      }
    }
    rlc++;
  }
  //
  rd = rdFull;
  for (r = 0; r < rdFullCnt; r++)
  {
    rd->setRoadQ(expectedRoadWidth);
    rd++;
  }
  // order roads by quality
  qsort(rdFull, rdFullCnt, sizeof(URoad), &compareRdFull);
  // best road is now in front
  if (rdFullCnt > 0)
  {
    rd = rdFull;
    roadCenterIdx = rd->idxCenter;
    roadLeftIdx = rd->idxLeft;
    roadRightIdx = rd->idxRight;
  }
  else
  {
    roadCenterIdx = -1;
    roadLeftIdx = -1;
    roadRightIdx = -1;
  }
  //
  updateRoadVariables(pose);
}

////////////////////////////////////////////

int UResRoadLine::findCurrentRoad(UPoseTime * pose)
{
  URoadLine * road;
  int i;
  UEdgeList rlList, rcList, rrList;
  double d, q;
  double dl = 0.0, dc = 0.0, dr = 0.0;
  int cnt;
  ULineSegment * seg;
  int type;
  UPosition rPos, pos;
  UPosition lPos;
  // alloe a left edge to be this distence to the right of the robot
  // and vice versa
  const double allowWrongSideDist = 0.5;
  //
  road = roads;
  rPos.set(pose->x, pose->y, 0.0);
  for (i = 0; i < roadsCnt; i++)
  {
    if (road->isValid())
    {
      type = road->getLineType();
      cnt = road->getUpdateCnt();
      seg = road->getLine();
      q = road->getQual();
      // get signed distance to line
      // returns positive if the robot is to the right of the line
      // so change sign so that d is positive if line is to the left of robot
      d = -seg->getXYsignedDistance(rPos);
      switch(type)
      {
      case 0: // left edge
        if (d > -allowWrongSideDist)
          rlList.add(d, cnt, q, i, roadLeftIdx);
        break;
      case 1: // centre
        rcList.add(d, cnt, q, i, roadCenterIdx);
        break;
      case 2: // right edge
        if (d < allowWrongSideDist)
          rrList.add(d, cnt, q, i, roadRightIdx);
        break;
      default: break;
      }
    }
    road++;
  }
  // implement if reasonable
  // Left edge
  roadLeftIdx = rlList.getBest(&roads[roadLeftIdx], roadLeftIdx, -1e5, 1e5);
  // Centre line
  roadCenterIdx = rcList.getBest(&roads[roadCenterIdx], roadCenterIdx, -1e5, 1e5);
  // Right edge
  roadRightIdx = rrList.getBest(&roads[roadRightIdx], roadRightIdx, -1e5, 1e5);
  //
  // find best edge
  if (roadCenterIdx >= 0)
  {
    d = roads[roadCenterIdx].getQual();
    i = roadCenterIdx;
  }
  else
  {
    d = 0.0;
    i = -1;
  }
  if ((roadRightIdx >= 0) and (d < roads[roadRightIdx].getQual()))
  {
    d = roads[roadRightIdx].getQual();
    i = roadRightIdx;
  }
  if ((roadLeftIdx >= 0) and (d < roads[roadLeftIdx].getQual()))
  {
    d = roads[roadLeftIdx].getQual();
    i = roadLeftIdx;
  }
  // test for right road-line order
  if ( i >= 0)
  { // there is a best roadline
    if (roadLeftIdx >= 0)
      dl = -roads[roadLeftIdx].getLine()->getXYsignedDistance(rPos);
    if (roadRightIdx >= 0)
      dr = -roads[roadRightIdx].getLine()->getXYsignedDistance(rPos);
    if (roadCenterIdx >= 0)
      dc = -roads[roadCenterIdx].getLine()->getXYsignedDistance(rPos);
    switch (roads[i].getLineType())
    {
      case 0: // left is best
        if (roadCenterIdx >= 0)
          roadCenterIdx = rcList.getBest(&roads[roadCenterIdx], roadCenterIdx, -1e5, dl);
        if (roadCenterIdx >= 0)
          dc = -roads[roadCenterIdx].getLine()->getXYsignedDistance(rPos);
        else
          dc = dl;
        if (roadRightIdx >= 0)
          roadRightIdx = rrList.getBest(&roads[roadRightIdx], roadRightIdx, -1e5, dc);
        break;
      case 1: // center is best
        if (roadLeftIdx>= 0)
          roadLeftIdx = rlList.getBest(&roads[roadLeftIdx], roadLeftIdx, dc, 1e5);
        if (roadRightIdx>= 0)
          roadRightIdx = rrList.getBest(&roads[roadRightIdx], roadRightIdx, -1e5, dc);
        break;
      case 2: // right is best
        if (roadCenterIdx>= 0)
          roadCenterIdx = rcList.getBest(&roads[roadCenterIdx], roadCenterIdx, dr, 1e5);
        if (roadCenterIdx >= 0)
          dc = -roads[roadCenterIdx].getLine()->getXYsignedDistance(rPos);
        else
          dc = dr;
        if (roadLeftIdx>= 0)
          roadLeftIdx = rlList.getBest(&roads[roadLeftIdx], roadLeftIdx, dc, 1e5);
        break;
      default: break;
    }
  }
  return 0;
}

///////////////////////////////////////////////////////

void UResRoadLine::updateRoadVariables(UPoseTime * pose)
{
  UPosition rPos, pos;
  URoadLine * rd;
  UPosition lPos;
  int n;
  double q, sd, dc, dl, dr, t;
  // update road variables in var pool
  pos = varPoolIdx[12]->get3D();
  pos = pose->getPoseToMap(pos);
  getVarPool()->lock();
  if (roadLeftIdx < 0)
    // set quality only
    varPoolIdx[0]->setValued(0.0, 1);
  else
  {
    rd = &roads[roadLeftIdx];
    dl = -rd->getLine()->getXYsignedDistance(pos);
    q = rd->getQual();
    n = rd->getUpdateCnt();
    sd = rd->getEstSD();
    varPoolIdx[0]->setValued(dl, 0);
    varPoolIdx[0]->setValued(q, 1);
    varPoolIdx[2]->setValued(n, 0);
    varPoolIdx[3]->setValued(sd, 0);
  }
  if (roadCenterIdx < 0)
    // set quality only
    varPoolIdx[4]->setValued(0.0, 1);
  else
  { //set center distance and quality
    rd = &roads[roadCenterIdx];
    dc = -rd->getLine()->getXYsignedDistance(pos);
    q = rd->getQual();
    n = rd->getUpdateCnt();
    sd = rd->getEstSD();
    varPoolIdx[4]->setValued(dc, 0);
    varPoolIdx[4]->setValued(q, 1);
    varPoolIdx[6]->setValued(n, 0);
    varPoolIdx[7]->setValued(sd, 0);
  }
  if (roadRightIdx < 0)
    // set quality only
    varPoolIdx[8]->setValued(0.0, 1);
  else
  { //set center distance and quality
    rd = &roads[roadRightIdx];
    dr = -rd->getLine()->getXYsignedDistance(pos);
    q = rd->getQual();
    n = rd->getUpdateCnt();
    sd = rd->getEstSD();
    varPoolIdx[8]->setValued(dr, 0);
    varPoolIdx[8]->setValued(q, 1);
    varPoolIdx[10]->setValued(n, 0);
    varPoolIdx[11]->setValued(sd, 0);
  }
  getVarPool()->unlock();
  if (roadLog != NULL)
  {
    // format: time 3*[scan x y SD Updates index] (order left center right)
    fprintf(roadLog, "%lu.%06lu ", pose->t.getSec(), pose->t.getMicrosec());
    if (roadLeftIdx >= 0)
    {
      rd = &roads[roadLeftIdx];
      t = rd->getLine()->getPositionOnLine(pos);
      if (t > rd->getLine()->length)
        lPos = rd->getLine()->getOtherEnd();
      else
        lPos = rd->getLine()->getPositionOnLine(t);
      fprintf(roadLog, " %lu %.3f %.3f %.5f %d %d   ",
              rd->getScanSerial(), lPos.x, lPos.y, rd->getEstSD(), rd->getUpdateCnt(), roadLeftIdx);
    }
    else
      fprintf(roadLog, " 0 0.0 0.0 0.0 0 0   ");
    if (roadCenterIdx >= 0)
    {
      rd = &roads[roadCenterIdx];
      t = rd->getLine()->getPositionOnLine(pos);
      if (t > rd->getLine()->length)
        lPos = rd->getLine()->getOtherEnd();
      else
        lPos = rd->getLine()->getPositionOnLine(t);
      fprintf(roadLog, " %lu %.3f %.3f %.5f %d %d   ",
              rd->getScanSerial(), lPos.x, lPos.y, rd->getEstSD(), rd->getUpdateCnt(), roadCenterIdx);
    }
    else
      fprintf(roadLog, " 0 0.0 0.0 0.0 0 0   ");
    if (roadRightIdx >= 0)
    {
      rd = &roads[roadRightIdx];
      t = rd->getLine()->getPositionOnLine(pos);
      if (t > rd->getLine()->length)
        lPos = rd->getLine()->getOtherEnd();
      else
        lPos = rd->getLine()->getPositionOnLine(t);
      fprintf(roadLog, " %lu %.3f %.3f %.5f %d %d",
              rd->getScanSerial(), lPos.x, lPos.y, rd->getEstSD(), rd->getUpdateCnt(), roadRightIdx);
    }
    else
      fprintf(roadLog, " 0 0.0 0.0 0.0 0 0");
    fprintf(roadLog, "\n");
  }
}

////////////////////////////////////////////

