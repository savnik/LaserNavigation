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
#include <ugen4/uline.h>
#include <umap4/umanseq.h>

#include "uresroaddrive.h"

// UResRoadDrive::UResRoadDrive()
// { // these first two lines is needed
//   // to save the ID and version number
//   setResID(getResID());
//   resVersion = getResVersion();
//   // other local initializations
//   createVarSpace(20, 0, 0, "Road line driver settings");
//   createBaseVar();
//   // create path pool
//   //roads = NULL;
//   poseHist = NULL;
//   varGlobal = NULL;
//   man = NULL;
// }


///////////////////////////////////////////

void UResRoadDrive::createBaseVar()
{
  addVar("version", getResVersion() / 100.0, "d", "Resource version");
  varUpdateTime = addVar("updateTime", 0.0, "d", "Last time the road drive command was calles");
  varLoopCnt = addVar("loopCnt", 0.0, "d", "Number of times called by sequencer");
  varEdgeDist = addVar("edgeDist", 0.35, "d", "Nominel distance to road edge");
  varFwdDist =  addVar("forwardDist", 6.0, "d", "Exit-pose distance from robot");
  varEdge =     addVar("edge", 0.0, "d", "Reference edge 0=left, 1=top, 2 = right");
  addVar("tgtRelX", 3.0, "pose", "(rw) target position (relative to robot) used in obstacle avoidance");
  //addVar("tgtRelY", 2.0, "pose", ""); // target position Y used in obstacle avoidance
  //addVar("tgtRelH", 1.0, "pose", ""); // target position H used in obstacle avoidance
  varTgtX   = addVar("tgtX", 0.0, "pose", "(ro) target position used in obstacle avoidance (set by call)");
  //varTgtY   = addVar("tgtY", 0.0, "pose", ""); // target position Y used in obstacle avoidance
  //varTgtH   = addVar("tgtH", 0.0, "pose", ""); // target position H used in obstacle avoidance
  varTgtVel = addVar("tgtVel",  1.0, "d", "target velocity at end of obstacle avoidance");
  varFailCnt = addVar("failCnt",  0.0, "d", "Path fails since last success");
  addVar("extra.ex1", 3.3, "d", "Extension 1");
  addVar("extra.ex2", 4.4, "d", "Extension 2");
  // and methods
  addMethod("road", "sd", "Drive along any roadside - with a minimum distance to this reference side");
  addMethod("left", "d", "Follow left side at this distance (should be negative to be on road)");
  addMethod("right", "d", "Follow left right at this distance (should be positive to be on road)");
  addMethod("top", "d", "Follow road center at this distance (positive is left of centerline)");
  addMethod("left", "dd", "Follow left side at this distance, "
      "the second parameter is number of times the function is called in "
      "rapid succession");
  addMethod("right", "dd", "Follow left right at this distance"
      "the second parameter is number of times the function is called in "
      "rapid succession");
  addMethod("top", "dd", "Follow road center at this distance"
      "the second parameter is number of times the function is called in "
      "rapid succession");
  addMethod("target", "sd", "get target pose with this reference, the 's' parameter may be either 'left', 'right', or 'top', the 'd' parameter is the desired distance from the reference line (positive is left)");
}

///////////////////////////////////////////

UResRoadDrive::~UResRoadDrive()
{
  if (man != NULL)
    man->unlock();
}

///////////////////////////////////////////

const char * UResRoadDrive::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Follow road drive command\n", preString);
  return buff;
}

///////////////////////////////////////////

bool UResRoadDrive::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  //
/*  if (resource->isA(UResLaserIfRoad::getResID()))
  {
    result = true;
    if (remove)
      roads = NULL;
    else if (roads != resource)
      roads = (UResLaserIfRoad *)resource;
    else
      // not used
      result = false;
  }
  else*/
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  {
    result = true;
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      // not used
      result = false;
  }
  else if (resource->isA(UResVarPool::getResID()))
  {
    result = true;
    if (remove)
      varGlobal = NULL;
    else if (varGlobal != resource)
      varGlobal = (UResVarPool *)resource;
    else
      // not used
      result = false;
  }
  else
    result = UResVarPool::setResource(resource, remove);
  return result;
}

//////////////////////////////////////////////////////

bool UResRoadDrive::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  if (varGlobal == NULL)
  {
    if (p1 != NULL)
    {
      strncpy(p1, UResVarPool::getResID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }
  if (poseHist == NULL)
  {
    if (p1 != NULL)
    {
      strncpy(p1, UResPoseHist::getOdoPoseID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }
/*  if (roads == NULL)
  {
    if (p1 != NULL)
    {
      strncpy(p1, UResLaserIfRoad::getResID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }*/
  result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////

bool UResRoadDrive::methodCall(const char * name, const char * paramOrder,
                       char ** strings, const double * pars,
                       double * value,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{
  bool result = true;
  // evaluate standard functions
  if ((strcasecmp(name, "road") == 0) and (strcmp(paramOrder, "sd") == 0))
    *value = driveRoad(strings[0], pars[0]);
  else if ((strcasecmp(name, "left") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = driveSide(0, pars[0], 0);
  else if ((strcasecmp(name, "top") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = driveSide(1, pars[0], 0);
  else if ((strcasecmp(name, "right") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = driveSide(2, pars[0], 0);
  else if ((strcasecmp(name, "left") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = driveSide(0, pars[0], roundi(pars[1]));
  else if ((strcasecmp(name, "top") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = driveSide(1, pars[0], roundi(pars[1]));
  else if ((strcasecmp(name, "right") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = driveSide(2, pars[0], roundi(pars[1]));
  else if ((strcasecmp(name, "target") == 0) and (strcmp(paramOrder, "sd") == 0))
    *value = setTarget(strings[0], pars[0]);
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////

double UResRoadDrive::driveRoad(const char * refSide, const double refDist)
{
  double result = 0.0;
  int side; // 0 is left, 1 is center 2 is right
  bool isOK = true;
  //
  // 1. dvs find exit position
  // 2. make auAvoid find a route to this exit position
  // 3. make ausmr follow this route
  if (strcmp(refSide, "left") == 0)
    side = 0;
  else if (strcmp(refSide, "center") == 0)
    side = 1;
  else if (strcmp(refSide, "top") == 0)
    side = 1;
  else if (strcmp(refSide, "right") == 0)
    side = 2;
  else
  {
    fprintf(stderr, "UResRoadDrive::driveRoad: Not a valid side of the road! '%s'\n", refSide);
    side = -1;
    isOK = false;
  }
  if (isOK)
  {
    result = driveSide(side, refDist, 0);
  }
  //
  return result;
}

/////////////////////////////////////////////////////

double UResRoadDrive::driveSide(const int refSide, const double refDist, int repeat)
{
  double d, vel, v;
  bool isOK;
  UPose exitPose;
  UVarPool * vp; //, *vpRoot = NULL;
  int n = 1;
  const int MPC = 4; // parameter count
  double pars[MPC];
  char * driver = (char *)"roadDriver";
  double updTime;
  UDataBase * manNew = NULL;
  UTime t;
  bool newData = (repeat == 0);
  const double minUpdatePeriod = 0.5;
  //
  vp = getVarPool();
  isOK = (vp != NULL);
  if (isOK)
  { // get root var-pool for global variable access
    //vpRoot = vp->getRootVarPool();
    // test if it is time to redo the obstacle avoidance test
    if (getGlobalValue("laserObst.time", &updTime))
    { // do we have new obstacle data
      t.setTime(updTime);
      if ((t - lastObstUpdate) > minUpdatePeriod)
      { // time for new update
        newData = true;
        lastObstUpdate = t;
      }
    }
    if(getGlobalValue("laserRoad.updateTime", &updTime))
    { // do we have new road data
      t.setTime(updTime);
      if ((t - lastRoadUpdate) > minUpdatePeriod)
      { // time for new update
        newData = true;
        lastRoadUpdate = t;
      }
    }
    if (newData or (refSide != lastRefSide) or (refDist != lastRefDist))
    { // or a new command - side or distance
      lastRefSide = refSide;
      lastRefDist = refDist;
      newData = true;
    }
  }
  if (newData)
  { // time to reevaluate route
    if (isOK)
    { // find desired exit pose
      d = varFwdDist->getValued();
      isOK = findExitPose(refSide, refDist, d, &exitPose);
      if (not isOK)
        fprintf(stderr, "UResRoadDrive::driveSide: failed to get road-side-segments'\n");
    }
    if (isOK)
    { // get smr-values for further calculations
      // get desired end velocity
      isOK  = getGlobalValue("smr.speed", &vel);
      isOK &= getGlobalValue("laserRoad.updateTime", &updTime);
      if (not isOK)
        fprintf(stderr, "UResRoadDrive::driveSide: is missing variable 'smr.speed' or 'laserRoad.updateTime'\n");
    }
    if (isOK)
    { // now exit pose and velociti is available,
      // so find best path to there
      pars[0] = exitPose.x;
      pars[1] = exitPose.y;
      pars[2] = exitPose.h;
      pars[3] = vel;
      isOK = callGlobal("avoid.getAvoidPath", "dddd", NULL, pars,
                                &v, &manNew, &n);
      if (not isOK)
        fprintf(stderr, "UResRoadDrive::driveSide: is missing method: 'avoid.getAvoidPath', 'dddd'\n");
      if (v == 0)
      { // no path found
        isOK = false;
        fprintf(stderr, "UResRoadDrive::driveSide: no path found to odo-pose (%.2fx, %.2fy, %.3frad)\n",
                exitPose.x, exitPose.y, exitPose.h);
      }
    }
    if (isOK)
    { // make status available - may trigger push commands (after this function has returned)
      varLoopCnt->add(1, 0);
      varUpdateTime->setDouble(updTime);
      varTgtX->setPose(&exitPose);
      //vp->setLocalVar(varTgtY,   exitPose.y, 0);
      //vp->setLocalVar(varTgtH,   exitPose.h, 0);
      varTgtVel->setDouble(vel); // target velocity at end of man
      varEdgeDist->setDouble(refDist);
      varEdge->setInt(refSide);
      varFailCnt->setInt(0);
    }
    else
    {
      varFailCnt->add(1.0, 0);
    }
    if (manNew != NULL)
    { // ensure there is no users f 'man'
      manLock.lock();
      // release old man to man-pool
      if (man != NULL)
        man->unlock();
      // set new man as most recent
      man = (UManSeq*)manNew;
      // let others access the new man
      manLock.unlock();
    }
    if (isOK)
    { // a path is found -tell the smr interface to implement
      isOK = callGlobal("smrCtl.setPlan", "sc", &driver, NULL,
                                &v, &manNew, &n);
      if (not isOK)
        fprintf(stderr, "UResRoadDrive::driveSide: is missing method: 'smrCtl.setPlan', 'sc'\n");
    }
  }
  //
  return isOK;
}

///////////////////////////////////////////////////////

double UResRoadDrive::setTarget(const char * sideStr, double dist)
{
  UPoseV pv;
  UVarPool * vp;
  int side;
  double d;
  UPose pose;
  bool isOK;
  //
  if (strcasecmp(sideStr, "left") == 0)
    side = 0;
  else if (strcasecmp(sideStr, "top") == 0)
    side = 1;
  else if (strcasecmp(sideStr, "right") == 0)
    side = 2;
  else
    side = -1;
  vp = getVarPool();
  if (vp != NULL)
  {
    if (not vp->getLocalValue("forwardDist", &d))
      d = 6.0;
    isOK = findExitPose(side, dist, d, &pv);
    if (not isOK)
      printf("UResRoadDrive::setTarget: failed to find exit pose\n");
    else
    { // exit pose is OK
      vp->setLocalVar("edge", side, true, UVariable::i);
      vp->setLocalVar("edgeDist", dist, true, UVariable::d);
      vp->setLocalVar("tgtRelX", pv.x, true, UVariable::d);
      vp->setLocalVar("tgtRelY", pv.y, true, UVariable::d);
      vp->setLocalVar("tgtRelH", pv.h, true, UVariable::d);
      vp->setLocalVar("tgtVel", pv.getVel(), false, UVariable::d); // target velocity at end of man
    }
  }
  //
  return 1.0; // true
}

/////////////////////////////////////////////////////////

bool UResRoadDrive::findExitPose(int side, double sideDist, double forwardDist,
                                 UPose * exitPose)
{
  UPoseTime pt;
  UPose pose;
  ULineSegment *pseg, seg, nseg;
  UDataBase * pdata;
  UVarPool *vp;
  double v, t;
  UPosition pos;
  bool isOK;
  int n;
  //
  vp = getVarPool();
  isOK = (vp != NULL);
  if (isOK)
  {
    //vpRoot = vp->getRootVarPool();
    pdata = (UDataBase*)&seg;
    n = 1;
    switch (side)
    {
      case 0:
        isOK = callGlobal("laserroad.leftSeg", "", NULL, NULL, &v, &pdata, &n);
        break;
      case 1:
        isOK = callGlobal("laserroad.centerSeg", "", NULL, NULL, &v, &pdata, &n);
        break;
      case 2:
        isOK = callGlobal("laserroad.rightSeg", "", NULL, NULL, &v, &pdata, &n);
        break;
      default:
        isOK = false;
        break;
    }
    if (not isOK)
      fprintf(stderr, "UResRoadDrive::findExitPose: Call to road-side segment failed\n");
  }
  if (isOK)
  {
    isOK = (poseHist != NULL);
    if (not isOK)
      fprintf(stderr, "UResRoadDrive::findExitPose: missing poseHist resource\n");
  }
  // now find
  if (isOK)
  { // get current robot pose
    pt = poseHist->getNewest(NULL);
    pos.set(pt.x, pt.y, 0.0);
    // find the closest position on the line to the robot
    pseg = (ULineSegment*)pdata;
    t = pseg->getPositionOnLine(pos);
    pos = pseg->getPositionOnLine(t);
    // make an exit pose by placing the pose at 'pos' and
    // side shift it to the desired distance and forward to the
    // exit pose
    pose.set( pos.x, pos.y, pseg->getXYHeading());
    // get the exit position in odometry coordinates
    pos = pose.getPoseToMap(forwardDist, sideDist);
    // and combine with heading of road line
    exitPose->set(pos.x, pos.y, pose.h);
  }
  //
  return isOK;
}

///////////////////////////////////////////////////////////////////

UManSeq * UResRoadDrive::getManLocked()
{
  manLock.lock();
  return man;
}


