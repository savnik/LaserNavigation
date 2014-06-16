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
#include <ugen4/uline.h>
#include <umap4/umanseq.h>

#include "uresdrivepos.h"

// UResDrivePos::UResDrivePos()
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

void UResDrivePos::createBaseVar()
{
//  UVarPool * vp;
  //
//  vp = getVarPool();
//  if (vp != NULL)
  {
    addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varUpdateTime = addVar("updateTime", 0.0, "d", "Last time the road drive command was calles");
    varLoopCnt = addVar("loopCnt", 0.0, "d", "Number of times called by sequencer");
    varOdoX   = addVar("odoX", "0.0 0.0 0.0", "pose", "(ro) target position used in obstacle avoidance (set by call)");
    varOdoV = addVar("odoV",  1.0, "d", "target velocity at end of obstacle avoidance");
    varFailCnt = addVar("failCnt",  0.0, "d", "Path fails since last success");
    varFinalDistance = addVar("finalDistance",  0.7, "d", "Distance from target position "
        "where no new path is calculated");
    // and methods
    addMethod("odo", "ddd", "Drive towards this odometry pose "
        "(x[m], y[m], h[rad])");
    addMethod("odo", "dddd", "Drive towards this odometry pose "
        "(x[m], y[m], h[rad]), the last (optional) parameter is a repeat count for calls "
            "in rapid succession.");
    addMethod("rel", "ddd", "Drive towards this pose relative to the robot "
        "(x[m], y[m], h[rad])");
    addMethod("rel", "dddd", "Drive towards this pose relative to the robot "
        "(x[m], y[m], h[rad]), the last (optional) parameter is a repeat count for calls "
            "in rapid succession. First call (repeat==0) determines origin for relative position");
    addMethod("odoVel", "dddd", "Drive towards this odometry pose and desired end velocity (x, y, h, vel)");
    addMethod("odoVel", "ddddd", "Drive towards this odometry pose and desired end velocity (x, y, h, vel), the last parameter is repeat count for drive commands.");
  }
}

///////////////////////////////////////////

UResDrivePos::~UResDrivePos()
{
}

///////////////////////////////////////////

const char * UResDrivePos::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Follow road drive command\n", preString);
  return buff;
}

///////////////////////////////////////////

bool UResDrivePos::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  //
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

bool UResDrivePos::gotAllResources(char * missingThese, int missingTheseCnt)
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

bool UResDrivePos::methodCall(const char * name, const char * paramOrder,
                       char ** strings, const double * pars,
                       double * value,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{
  bool result = true;
  UPoseV toPose;
  UPoseTime pt;
  int repeat;
  // evaluate standard functions
  if ((strcasecmp(name, "odo") == 0) and (strcmp(paramOrder, "ddd") == 0))
  {
    toPose.set(pars[0], pars[1], pars[2], 0.0);
    *value = driveOdo(toPose, true, 0);
  }
  else if ((strcasecmp(name, "odo") == 0) and ((strcmp(paramOrder, "dddd") == 0) or ((strcmp(paramOrder, "ddd") == 0))))
  {
    if (strlen(paramOrder) == 4)
      repeat = roundi(pars[3]);
    else
      repeat = 0;
    toPose.set(pars[0], pars[1], pars[2], 0.0);
    *value = driveOdo(toPose, true, repeat);
  }
  else if ((strcasecmp(name, "rel") == 0) and ((strcmp(paramOrder, "dddd") == 0) or ((strcmp(paramOrder, "ddd") == 0))))
  {
    if (strlen(paramOrder) == 4)
      repeat = roundi(pars[3]);
    else
      repeat = 0;
    toPose.set(pars[0], pars[1], pars[2], 0.0);
    // convert to real pose
    if (poseHist != NULL and (repeat == 0))
      repeat0startPose = poseHist->getNewest(NULL);
    toPose = repeat0startPose.getPoseToMapPose(toPose);
    *value = driveOdo(toPose, true, repeat);
  }
  else if ((strcasecmp(name, "odoVel") == 0) and (strcmp(paramOrder, "dddd") == 0))
  {
    toPose.set(pars[0], pars[1], pars[2], pars[3]);
    *value = driveOdo(toPose, false, 0);
  }
  else if ((strcasecmp(name, "odoVel") == 0) and (strcmp(paramOrder, "ddddd") == 0))
  {
    repeat = roundi(pars[4]);
    toPose.set(pars[0], pars[1], pars[2], pars[3]);
    *value = driveOdo(toPose, false, 0);
  }
  else
    result = false;
  return result;
}


/////////////////////////////////////////////////////

double UResDrivePos::driveOdo(UPoseV exitPose, bool ignoreVel, int repeat)
{
  double vel, v, result = true;
  bool isOK;
  UVarPool * vp;
  int n = 1;
  const int MPC = 4; // parameter count
  double pars[MPC];
  char * driver = (char*)"driveOdo";
  double updTime, d, d2;
  //UManSeq * manNew = NULL;
  UDataBase * manNew = NULL;
  bool newData;
  UTime t, tLast;
  double tq;
  const double MINOR_EXIT_POSE_MOVEMENT = 0.125; // relative to distance from exit pose
//  const double MINOR_EXIT_POSE_DISTANCE = 0.25; // meter
  double finalDistance = 0.7; // meter - no new man planning
  UPoseTime currentPose;
  int why = 0;
  //
  if (repeat == 0)
  {
    finalPart = false;
    newData = true;
    why = 1;
    odoDriveCnt++;
    // debug
/*    if (odoDriveCnt == 2)
      printf("UResDrivePos::driveOdo: odometry drive #%d\n", odoDriveCnt);
    else
      printf("UResDrivePos::driveOdo: odometry drive #%d\n", odoDriveCnt);*/
    // debug end
  }
  else
    newData = false;
  //
  vp = getVarPool();
  isOK = (vp != NULL);
  if (poseHist != NULL)
    currentPose = poseHist->getNewest(NULL);
  if (isOK)
  { // test if it is time to redo the obstacle avoidance test
    if (getGlobalValue("laserObst.time", &updTime))
    { // do we have new obstacle data
      t.setTime(updTime);
      if (t != lastObstUpdate)
      {
        newData = true;
        why = 2;
        lastObstUpdate = t;
      }
    }
    if (not newData)
    { // has exit pose moved? if so scale with distance to exit pose
      d = lastExitPose.getDistance(exitPose);
      d2 = maxd(1.0, currentPose.getDistance(&exitPose));
      if (d/d2 > MINOR_EXIT_POSE_MOVEMENT)
      { // or a new command - side or distance
        newData = true;
        why = 3;
        finalPart = false;
        // debug
        // printf("UResDrivePos::driveOdo why=3, as %g/%g=%g > %g\n", d, d2, d/d2, MINOR_EXIT_POSE_MOVEMENT);
        // debug end
      }
    }
  }
  // test for final part
  if (not finalPart and (repeat > 0))
  { // du not make new path calculations on the final part
    d = currentPose.getDistance(&exitPose);
    finalDistance = varFinalDistance->getValued();
    if (d < finalDistance)
    {
      finalPart = true;
      finalPose = currentPose;
      newData = false;
      // debug
/*      printf("UResDrivePos::driveOdo: on the final %.2fx %.2fy (%.2f > %.2f)\n",
            finalPose.x, finalPose.y, d, finalDistance);*/
      // debug end
    }
  }
  // test for implicit stop condition
  if (finalPart)
  {
    newData = false;
    d = finalPose.getDistance(&exitPose);
    d2 = finalPose.getDistance(&currentPose);
    if (d2 >= d)
      // set return value to mark that destination is reached
      result = 2.0;
    // debug
/*    printf("UResDrivePos::driveOdo: %.2f > %.2f %s\n",
           d2, d, bool2str(result == 2.0));*/
    // debug end
  }
  // test for time since last calculation - may be too early
  if (newData and (repeat > 0))
  { // get time to queue
    getGlobalValue("smrctl.manQueueingTime", &tq);
    // get last queue time
    getGlobalValue("smrctl.manEndTime", &tLast);
    // is it too soon to calculate a new manoeuvre
    if (fabs(tLast.getTimePassed()) < tq)
    {
      newData = false;
      // debug
      printf("Too soon to calculate new route %.3f sec passed, but %.3fs needed\n", tLast.getTimePassed(), tq);
      // debug end
    }
  }
  // debug
/*  if (newData)
    printf("UResDrivePos::driveOdo: why=%d, repeat=%d, odometry drive #%d\n", why, repeat, odoDriveCnt);*/
  // debug end
  //
  if (newData)
  {
    lastExitPose = exitPose;
    if (isOK)
    { // get smr-values for further calculations
      // get desired end velocity
      if (ignoreVel)
      {
        isOK  &= getGlobalValue("smr.speed", &vel);
        exitPose.setVel(vel);
      }
      if (not isOK)
        fprintf(stderr, "UResDrivePos::driveOdo: is missing 'smr.speed'\n");
    }
    if (isOK)
    { // now exit pose and velociti is available,
      // so find best path to there
      pars[0] = exitPose.x;
      pars[1] = exitPose.y;
      pars[2] = exitPose.h;
      pars[3] = exitPose.getVel();
      isOK = callGlobal("avoid.getAvoidPath", "dddd", NULL, pars,
                                &v, &manNew, &n);
      if (not isOK)
        fprintf(stderr, "UResDrivePos::driveOdo: is missing method: 'avoid.getAvoidPath', 'dddd'\n");
      if (v == 0)
      { // no path found
        isOK = false;
        fprintf(stderr, "UResDrivePos::driveOdo: no path found to odo-pose (%.2fx, %.2fy, %.3frad) (manCnt=%d)\n",
                exitPose.x, exitPose.y, exitPose.h, n);
      }
    }
    if (isOK)
    { // make status available - may trigger push commands (after this function has returned)
      varLoopCnt->add(1.0, 0);
      varUpdateTime->setValued(updTime, 0, false);
      varOdoX->setPose(&exitPose);
//      varOdoY->setValued(exitPose.y, 0);
//      varOdoH->setValued(exitPose.h, 0);
      varOdoV->setValued(exitPose.getVel(), 0, false); // target velocity at end of man
      varFailCnt->setInt(0);
    }
    else
    {
      varFailCnt->add(1.0, 0);
    }
    if (manNew != NULL)
    { // ensure there is no users of 'man' (current plan)
      // debug
      if (not isOK)
        printf("UResDrivePos::driveOdo: logic error - found solution, but no manoeuvre structure!\n");
      // debug end
      isOK = manLock.tryLock();
      // debug
      if (not isOK)
      {
        printf("UResDrivePos::driveOdo: found current manoeuvre locked - waiting for man to free !\n");
        while (not isOK)
          isOK = manLock.tryLock();
      }
      // debug end
      // release old man to man-pool
      if (man != NULL)
      {
        // debug
        // printf("UResDrivePos::driveOdo: releasing path\n");
        // debug end
        man->unlock();
      }
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
        fprintf(stderr, "UResDrivePos::driveOdo: is missing method: 'smrCtl.setPlan', 'sc'\n");
      else if (v == 0.0)
        fprintf(stderr, "UResDrivePos::driveOdo: smrCtl failed to set plan\n");
      lastStartPose = poseHist->getNewest(NULL);
    }
    result = isOK;
  }
  else
  { // test if smr has finished all posted commands
    isOK = callGlobal("smrctl.finished", "", NULL, NULL,
                        &v, NULL, NULL);
    if (isOK)
    {
      if (v > 0.5)
      { // hit user event, so planned distance is complete
        result = 2; //
        // debug
        printf("UResDrivePos::driveOdo: smr hit the user event - continue script!\n");
        // debug end
      }
      else
        result = 1; // not finished (as expected)
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////

UManSeq * UResDrivePos::getManLocked()
{
  manLock.lock();
  return man;
}


UPoseV UResDrivePos::getCurrentPose()
{
  UPoseV result;
  UPoseTime pt;
  double v;
  //
  if (poseHist != NULL)
  {
    pt = poseHist->getNewest(&v);
    result.set(pt.x, pt.y, pt.h, v);
  }
  return result;
}
