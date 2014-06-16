/** *************************************************************************
 *                                                                         *
 *   \file              uresScansaver.cpp                                  *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Jan 2007                                           *
 *   \brief             scansaver class implementation                     *
 *                                                                         *
 *                      Copyright (C) 2006 by DTU                          *
 *                      rse@oersted.dtu.dk                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <stdio.h>
#include <math.h>
#include <urob4/uvarcalc.h>

#include "uresauef.h"

///////////////////////////////////////////

void UResAuEf::UResAuEfInit()
{ // these first two lines is needed
  // to save the ID and version number
/*  setResID(getResID());
  resVersion = getResVersion();*/
  // other local initializations
  llog = NULL;
  llogName = "efline.log";
  createBaseVar();
  linesCnt = 0;
  for (int i = 0; i < 4; i++)
    wallsAge[i] = -1;
}

///////////////////////////////////////////

UResAuEf::~UResAuEf()
{
  if (llog != NULL)
    fclose(llog);
}

///////////////////////////////////////////

const char * UResAuEf::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s line finder (v%f) last found %d lines, logfile open: %s\n", preString,
           getResVersion()/100.0, linesCnt,
           llog != NULL ? "true" : "false");
  return buff;
}

///////////////////////////////////////////

void UResAuEf::createBaseVar()
{
  //
  //vp->addVar("version", getResVersion(), "d", "(r) Version of this the circle example resource");
  varSplit    = addVar("doSplit",       1.0,  "d", "(r/w) Split line segments using fit resudial [true/false]");
  varMerge    = addVar("doMerge",       1.0,  "d", "(r/w) Try merge line segments after extract [true/false]");
  varDiscard  = addVar("doDiscard",     1.0,  "d", "(r/w) Discard too small clusters [true/false]");
  varClustMinCnt = addVar("clustMin",   5.0,  "d", "(r/w) minimum number of measurements for one cluster");
  varClustDif = addVar("clustDiff",     0.1,  "d", "(r/w) measurement distance for clusters creation");
  varSplitDev = addVar("splitDev",      0.01, "d", "(r/w) Line fit deviation for cluster split [m]");
  varMergeDev = addVar("mergeDev",      0.01, "d", "(r/w) Line fit deviation for cluster merge [m]");
  varDiscardCnt  = addVar("discardCnt",   10.0,  "d", "(r/w) Cluster count below this number is discarded");
  varDiscardSize = addVar("discardSize", 0.12,"d", "(r/w) Discard if shorter than this [m]");
  varCnt       = addVar("cnt", 0.0, "d", "(r) Number of lines found in last scan");
  varTime      = addVar("time", 0.0, "d", "(r) Time of last feature extraction");
  //
  varWallAngLimit = addVar("wallAngLimit", M_PI / 8.0, "d", "(r/w) Angle limit for line correlation with established wall lines.");
  varWallDistLimit = addVar("wallDistLimit", 0.25, "d", "(r/w) Distance limit for line correlation with estabblished wall line");
  varWallAgeLimit = addVar("wallAgeLimit", 1000.0, "d", "(r/w) After this number of scans without update, the distance correlation limit is ignored");
  varMinLineLength = addVar("minLineLength", 0.30, "d", "(r/w) Lines shorter than this is not used to update a wall");
  //
  varSafeWallLength = addVar("safeWallLength", 1.7, "d",
  "(r/w) Lines longer than this overwrites the distance limit criteria");
  varMakeWalls90deg = addVar("MakeWalls90deg", 0.0, "d",
  "(r/w) Rectify wall corners to 90 degrees");
  //vp->addMethod(this, "distance", "", "Distance from position 1 to position 2");
}

/////////////////////////////////////////////

void UResAuEf::getParams()
{
  // set parameters in feature extraction function from global variables
  configMode = 0;           //!< Split and merge configuration
  if (varSplit->getBool())
    configMode |= EF_DO_SPLIT;
  if (varMerge->getBool())
    configMode |= EF_DO_MERGE;
  if (varDiscard->getBool())
    configMode |= EF_DO_DISCARD;
  //
  configMinSegment = varClustMinCnt->getInt();   //!< Minimum number of points in segment
  configClusterDiff = varClustDif->getValued();     //!< Range diff. resulting in cluster split
  configSplitMSQ = sqr(varSplitDev->getValued());     //!< Mean square error resulting in split
  configMergeMSQ = sqr(varMergeDev->getValued());     //!< Max mean square error accepted for merges
  configDiscardDots = varDiscardCnt->getInt();  //!< Discard elements exceeding this MSQ
  configDiscardSize = varDiscardSize->getValued();    //!< Discard segments with length/diameter smaller than this
  // and the new box limits
  wallAngLimit = varWallAngLimit->getValued();
  wallDistLimit = varWallDistLimit->getValued();
  wallAgeResetLimit = varWallAgeLimit->getInt();
  minLineLength = varMinLineLength->getValued();
  safeWallLength = varSafeWallLength->getValued();
  makeWalls90deg = varMakeWalls90deg->getBool();
}

///////////////////////////////////////////

bool UResAuEf::toCartesian(ULaserData *ldata, RangeData *rdata)
{
  bool success=true;
  double d;
  bool rangeValid;
  //
  rdata->points = ldata->getRangeCnt();
  for (int i = 0; i < rdata->points; i++) {
    // Get angle as radians
    rdata->point_th[i] = ldata->getAngleRad(i);
    // range is reported in integer in a specified unit
    // Get range and translate to meters
    d = ldata->getRangeMeter(i, &rangeValid);
    if (not rangeValid)
      d = OUT_OF_RANGE; // OUT_OF_RANGE is -1, defined in rangedata.h (aulibextractfeatures)
    rdata->point_r[i] = d;
  }
  rdata->polarToCart();
  return success;

}

////////////////////////////////////////////////////////

int UResAuEf::findFeatures(ULaserData * ldata)
{
  RangeData rdata;
  PolarLineFit * pLin;
  AUEFReturnStruct res;
  AU2DLineSeg * lin;
  U2Dlined * abc;
  UTime scantime;
  double x, y, th, len, resSq;
  //
  if (ldata != NULL)
  { // get parameters for extraction
    getParams();
    scantime = ldata->getScanTime();
    // convert polar laser scanner data to cartesian
    toCartesian(ldata, &rdata);
    // extract the lines seen in this laserscan acording to parameters
    // part of AULibExtractFeatures
    extractFeatures(&rdata);
    // result is now available in extraacted segments, that may be lines
    lin = &lines[0];
    linesCnt = 0;
    abc = abcs;
    for (int i = 0; i < getSegmentCount(); i++)
    { // get pointer to segment if it is a line (may be a circle too)
      pLin = getSegmentLine(i);
      if (pLin != NULL)
      { // it is a line, get the estimated cartesian coordinates
        pLin->asCart(&x, &y, &th, &len, &resSq);
        lin->setLine(x, y, th, len, resSq);
        abc->set(cos(pLin->getTh0()), sin(pLin->getTh0()), -pLin->getR0());
        if (linesCnt < MAX_LINES)
        { // move to next line - if space
          linesCnt++;
          if (linesCnt < MAX_LINES - 1)
          { // increase pointers only to last valid entry
            lin++;
            abc++;
          }
        }
      }
    }
    // set result in global vars
    varCnt->setInt(linesCnt);
    varTime->setTime(scantime);
    // save to log if logfile is open
    // log
    llogLock.lock();
    if (llog != NULL)
    { // write log in matlab usable format
      // format is time scan serial x y th length res
      for (int i = 0; i < linesCnt; i++)
      {
        lin = &lines[i];
        fprintf(llog, "%lu.%06lu %lu %.4f %.4f %.6f %.3f %g\n",
              scantime.getSec(), scantime.getMicrosec(), ldata->getSerial(),
                              lin->x, lin->y, lin->th, lin->length, sqrt(lin->resSQ));
      }
    }
    llogLock.unlock();
  }
  //
  return linesCnt;
}

//////////////////////////////////////////////////

int UResAuEf::updateWalls(UPose newPose)
{ // return number of sides found
  U2Dlined * abc, *abc2;
  int can[4][MAX_LINES]; // candidateLines
  int canCnt[4]; // candidate lines count
  double aa, aw, aA, aB, a, d;
  U2Dlined abcN;
  //
  double moved[4];
  int k, n, m;
  double x1, y1, x2, y2;
  bool isOK = true;

  //
  // init candidate list
  for (int i = 0; i < 4; i++)
    canCnt[i] = 0;
  // 1) initialize wall estimate based on longest line
  if (wallsAge[0] < 0)
  {
    isOK = initializeWalls();
    // save pose from this update
    lastPose = newPose;
  }
  //
  if (isOK)
  { // 2) associate walls
    // find angle orientation as average of all 4 walls
    // - as north angle (A,B parameters)
    aA = (walls[0].A() - walls[1].B() - walls[2].A() + walls[3].B()) / 4.0;
    aB = (walls[0].B() + walls[1].A() - walls[2].B() - walls[3].A()) / 4.0;
    aa = atan2(aB, aA); // angle to North line (average)
    // adjust angle with robot movement since last update
    aa = limitToPi(aa - (newPose.h - lastPose.h));
    // find which lines candidate to which wall
    for (int i = 0; i < linesCnt; i++)
    { // find candidate side
      abc = &abcs[i];
      // get angle of candidate line
      aw = atan2(abc->B(), abc->A());
      a = aa;
      for (int j = 0; j < 4; j++)
      { // use line if within angle limit and of a resonable length
        if (absf(limitToPi(a - aw)) < wallAngLimit and
            lines[i].length > minLineLength)
        { // and a distance check
          // - positive when candidate is further away than estimated wall
          d = walls[j].C() - abc->C();
          // use if close to last distance - or is very long - or
          // wall line is very old or wall is never updated
          if ((absf(d) < wallDistLimit) or
              (lines[i].length > safeWallLength) or
              (wallsAge[j] > wallAgeResetLimit) or
              (wallsAge[j] == 0))
            // close or further away, then OK for update
            // or if wall not updated for a long time (or never)
            can[j][canCnt[j]++] = i;
          // will not fit anyone else better, so break now
          break;
        }
        // advance to next wall angle (clockwise)
        a = limitToPi(a - M_PI / 2.0);
      }
    }
    // 3) update with best (longest) candidate
    // - a more intelligent approach should probably be implemented
    n = 0;
    aA = 0.0;
    aB = 0.0;
    for (int i = 0; i < 4; i++)
    { // assume first candidate
      if (canCnt[i] > 0)
      { // default is first candidate
        k = can[i][0];
        d = lines[k].length;
        for (int j = 1; j < canCnt[i]; j++)
        { // try all candidate lines for wall line i
          m = can[i][j];
          if (lines[m].length > d)
          { // use linecandidate m instead
            d = lines[m].length;
            k = m;
          }
        }
        // get line to be used for update
        abc = &abcs[k];
        // calculate if wall is moved (positive is further away from robot)
        moved[i] = walls[i].C() - abc->C();
        // update wall
        walls[i] = *abc;
        // wall is updated
        wallsAge[i] = 1;
        switch (i)
        { // summ known wall angles, to be able to update
          // angle of undetected walls
          case 0: aA +=  abc->A(); aB +=  abc->B(); break;
          case 1: aA += -abc->B(); aB +=  abc->A(); break;
          case 2: aA += -abc->A(); aB += -abc->B(); break;
          case 3: aA +=  abc->B(); aB += -abc->A(); break;
        }
        n++;
      }
      else if (wallsAge[i] != 0)
        // wall not updated, increase age if ever detected
        wallsAge[i] += 1;
    }
    // get average angle of side 0 (north)
    aa = atan2(aB, aA);
    // move also not updated walls with the same
    // distance as the opposite wall
    // - as long as the wall don't touch the robot.
    for (int i = 0; i < 4; i++)
    {
      if (wallsAge[i] != 1)
      { // wall not updated, so update as opposite wall
        k = (i + 2) % 4; // opposite wall number
        if (wallsAge[k] == 1)
        { // opposite wall is updated
          abc = &walls[k]; // opposite wall
          // new distance to other wall
          if (absf(moved[k]) < (2.0 * wallDistLimit))
            // move this wall correspondingly
            d = walls[i].C() + moved[k];
          else
            // else leave wall at current distance
            d = walls[i].C();
          if (d > -0.1)
            // do not move closer than 10cm from laser scanner
            d = -0.1;
          // update with same angle as opposite wall, and update distance
          walls[i].set(-abc->A(), -abc->B(), d);
        }
        else
        { // wall not updated, make sure wee do not cross old estimate
          abc = &walls[i];
          // new angle of this side
          a = aa - i * M_PI/2.0;
          // keep the wall angle updated - keep distance unchanged
          abc->set(cos(a), sin(a), abc->C());
        }
      }
    }
    //
    if (makeWalls90deg)
    { // rectify angles to be at right angles to each other
      // debug
      printf("rectifying to %.4f rad = %.1f deg (north)\n",
              aa, aa*180.0/M_PI);
      // debug end
      for (int i = 0; i < 4; i++)
      { // get average angle for this wall
        a = aa - i * M_PI/2.0;
        // getthe wall line (in Ax+By+C=0 format)
        abc = &walls[i];
        // set angle, but maintain distance to robot (C)
        abc->set(cos(a), sin(a), abc->C());
      }
    }
    // 4) save walls as line segments for the reply
    for (int i = 0; i < 4; i++)
    { // line segment to be found
      abc = &walls[i];
      // get more clockwise line
      abc2 = &walls[(i + 3) % 4];
      // find crossing of these lines
      isOK = abc->getCrossing(*abc2, &x1, &y1);
      if (not isOK)
      { // error report
        fprintf(stderr, "UResAuEf::updateWalls: Error, no crossing between box line %d and %d!\n",
              i, (i + 3) % 4);
        break;
      }
      abc2 = &walls[(i + 1) % 4];
      isOK = abc->getCrossing(*abc2, &x2, &y2);
      if (not isOK)
      {
        fprintf(stderr, "UResAuEf::updateWalls: Error, no crossing between box line %d and %d!\n",
              i, (i + 1) % 4);
        break;
      }
      d = hypot(y2 - y1, x2 - x1);
      a = atan2(y2 - y1, x2 - x1);
      wallSeg[i].setLine(x1, y1, a, d, -1.0);
    }
    // save also the box coordinate origin
    // this is the south-west corner.
    // Origin position is start of west line (line 3)
    boxPose.x = wallSeg[3].x;
    boxPose.y = wallSeg[3].y;
    // east heading is opposite to heading of line 2
    boxPose.h = limitToPi(wallSeg[2].th + M_PI);
    // the box width (east-west) origin of line 2 and 3
    boxEW = hypot(wallSeg[3].x - wallSeg[2].x, wallSeg[3].y - wallSeg[2].y);
    // the box height (north-south) origin of line 0 and 3
    boxNS = hypot(wallSeg[3].x - wallSeg[0].x, wallSeg[3].y - wallSeg[0].y);
    // count updated lines
    n = 0;
    if (isOK)
    { // wall lines are valid
      for (int i = 0; i < 4; i++)
        if (wallsAge[i] == 1)
          // 1 means updated this scan
          n++;
    }
  }
  else
    // no valid wall estimate
    n = -1;
  // save last pose
  lastPose = newPose;
  //
  return n;
}

///////////////////////////////////////////////

bool UResAuEf::initializeWalls()
{
  AU2DLineSeg * longest = NULL;
  int i, m = -1, w;
  U2Dlined * abc, *abcN = abcs;
  double cos45 = cos(M_PI/4.0);
  double d;
  // find longest wall
  if (linesCnt > 0)
  { // seed with first line
    d = lines[0].length;
    m = 0;
    for (i = 1; i < linesCnt; i++)
    {
      if (lines[i].length > d)
      {
        d = lines[i].length;
        m = i;
      }
    }
    // now is the longest line found - which wall is it?
    // Let this defines the room orientation
    longest = &lines[m];  // segment in cartesian format
    abc = &abcs[m];       // line in Ax+By+C=0 format (where A^2 + B^2 == 1)
    if (abc->B() >= cos45)
      w = 0; // line is to the left and mostly aligned with robot x, so north
    else if (abc->A() >= cos45)
      w = 1; // line is ahead, and mostly across, so east
    else if (abc->B() < -cos45)
      w = 2; // line is to the right and mostly aligned with robot x, so south
    else
      w = 3; // or else it must be behind robot (west)
    //
    //
    walls[w] = *abc;
    wallsAge[w] = 1; // wall number w is updated
    abcN = walls; // get pointer to north (first) line
    switch(w)
    { // set north based on best line at 10cm distance from robot
      case 0: break; // do nothing
      case 1: // longest is east
        abcN->set(-abc->B(), abc->A(), -0.1); break;
      case 2: // longest was south
        abcN->set(-abc->A(),-abc->B(), -0.1); break;
      case 3: // longest was west
        abcN->set( abc->B(),-abc->A(), -0.1); break;
    }
    if (w != 0)
      wallsAge[0] = 0; // wall set but not updated
    // set other yet undefined walls
    for (i = 1; i < 4; i++)
    {
      if ( i != w)
      { // do not set seed wall
        switch(i)
        {
          case 1:
            walls[1].set( abcN->B(),-abcN->A(), -0.1); break;
          case 2:
            walls[2].set(-abcN->A(),-abcN->B(), -0.1); break;
          case 3:
            walls[3].set(-abcN->B(), abcN->A(), -0.1); break;
        }
        wallsAge[i] = 0;
      }
    }
  }
  return (m >= 0);
}

//////////////////////////////////////////////////

void UResAuEf::openLog(bool close)
{ // open or close logfile
  const int MFL = MAX_FILENAME_LENGTH;
  char filename[MFL];
  //
  llogLock.lock();
  if (llog != NULL)
  {
    fclose(llog);
    llog = NULL;
    printf("Closed logfile %s/%s\n", dataPath, llogName);
  }
  if (not close)
  {
    snprintf(filename, MFL, "%s/%s", dataPath, llogName);
    // pront to server console
    printf("Opened logfile %s\n", filename);
    llog = fopen(filename, "w");
  }
  llogLock.unlock();
}

//////////////////////////////////////////////////

void UResAuEf::wallReset()
{
  for (int i = 0; i < 4; i++)
    wallsAge[i] = -1;
}

//////////////////////////////////////////////////

UTime UResAuEf::getLastUpdateTime()
{
  return varTime->getTime();
}
