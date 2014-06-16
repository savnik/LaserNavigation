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

#include "uresavoid.h"



void UResAvoid::UResAvoidInit()
{ // these first two lines is needed
  // to save the ID and version number
//  setResID(getResID());
//  resVersion = getResVersion();
  // other local initializations
  createVarSpace(20, 0, 0, "Obstacle avoidance settings", false);
  createBaseVar();
  // create path pool
  paths = new UAvoidPathPool();
  poseHist = NULL;
  mapPose = NULL;
  globVar = NULL;
  paths->par.logAvoid = new ULogFile();
  paths->par.logAvoid->setLogName("avoid");
  strncpy(logAvoidName, paths->par.logAvoid->getLogFileName(), MAX_FILENAME_LENGTH);
}


///////////////////////////////////////////

void UResAvoid::createBaseVar()
{
  UVarPool * conf;
  UVarPool * debug;
  //
  //  addVar("version", getResVersion() / 100.0, "d", "Resource version");
  varCalcTime = addVar("updateTime", 0.0, "d", "Time of last avoidance evaluation");
  conf = addStruct("conf", "configuration variables");
  varManCnt = addVar("manCnt", 0.0, "d", "Count of valid manoeuvres in man-pool");
  varSerial = addVar("serial", 0.0, "d", "Serial number for latest obstacle avoidence calculation");
  if (conf != NULL)
  {
    varObstMinMinDist = conf->addVar("obstMinMinDist", 0.35, "d",
              "Absolute minimum distance to obstacles (else stop) (old method)");
    varObstMinDist = conf->addVar("obstMinDist", 0.5, "d", "Minimum distance to obstacles (old method)");
    //varSpawnLimit = vp->addVar("spawnLimit", 10.0, "d", "Maximum number of alternative routes");
    varMaxAcc        = conf->addVar("maxAcc", 0.3, "d", "Maximum lateral acceleration in manoeuver planning");
    varMaxTurnAcc    = conf->addVar("maxTurnAcc", 0.3, "d", "Maximum acceleration in turns");
    varMinTurnRadius = conf->addVar("minTurnRadius", 0.1, "d", "Minimum allowed turn radius");
    varClearenceDesired = conf->addVar("clearenceDesired", 0.2, "d",
          "Desired distance from robot to obstacles - during path planning on open areas (new method)");
    varClearenceMinimum = conf->addVar("clearenceMinimum", 0.05, "d",
          "Minimum distance from robot to obstacles - during obstacle avoidance path planning (new method)");
    varFrontLeft    = conf->addVarA("frontLeftX", "0.8 0.35 0.25", "3d",
          "Most critical front left position X-value is forward direction, y is positive left, z is ignored");
    varFrontRight   = conf->addVarA("frontRightX", "0.8 -0.35 0.25", "3d",
          "Most critical front right position X-value is forward direction, y is positive left, z is ignored");
    varRev2         = conf->addVar("rev2", 0.0, "d", "Use new obstacle avoidance method (else old dyrehaven method)");
    varRev2cell     = conf->addVar("rev2cell", 0.0, "d", "Use new absolute celle decomposition (else visual graph)");
    varUseDriveon   = conf->addVar("useDriveon", 1.0, "d", "(rw) use to line (i.e. 'driveon') manoeuvres rather than to pose (turn-drive-turn...)");
    varDriveonGA    = conf->addVar("driveonGA", 2.0, "d", "(rw) driveon parameter gain for angle correction.");
    varDriveonGD    = conf->addVar("driveonGD", 0.75, "d", "(rw) driveon parameter gain for line distance");
    varForwardAngle = conf->addVar("forwardOnly", 1.6, "d", "(rw) allow obstacle avoidance solutions starting within +/- radians from current heading (>=3.15 allows any angle)");
  varMaxOG        = conf->addVar("maxOG", 1.0, "d", "Maximum number of used obstacle groups (use 1 with v360, else 2 or 3 to get history behind robot)");
    varFollowLineOnLastPose = conf->addVar("followLineOnLastPose", 1.0, "d", "(rw) should robot try to get to destination pose line as fast as possible, else as late as possible");
  }
  debug = addStruct("debug", "debug options");
  if (debug != NULL)
  {
    varIgnoreObstacles = debug->addVar("ignoreObstacles", 0.0, "d", "Ignore all obstacles if true");
    varCrashTest = debug->addVar("crashTest", 0.0, "d", "Let crashed paths stay for reporting (1=true)");
    varMaxNestedSpawns = debug->addVar("maxNestedSpawns", 1.0, "d", "Maximum number of nested levels in path search");
    varMaxAvoidLoops = debug->addVar("maxAvoidLoops", 10.0, "d", "Maximum number of loops for midPose search");
    varMaxSpawnCount = debug->addVar("maxSpawnCount", 6.0, "d", "Maximum spawn count limit");
    varMaxTangentDepth = debug->addVar("maxTangentDepth", 5.0, "d", "(rw) max number of visibility tangents allowed from current pose to target pose. Reduces search time when many obstacles are present.");
    varMaxTangentToMan = debug->addVar("maxTangentToMan", 2.0, "d", "(rw) max number of tangent graphs tested, if better graphs fail to generate a manoeuvre sequence.");
    varIgnoreCloseObst = debug->addVar("ignoreCloseObst", 8.0, "d", "(rw) Ignore obstacles after this count, if they gets too near manoeuvre path along visibility graph");
    varAcceptSolution = debug->addVar("acceptSolution", 88.0, "d", "(rw) Stop mid-point calculation after this number of attempted solutions (debug value)");
    varUseAnyResult = debug->addVar("useAnyResult", 0.0, "d", "(rw) Use any result after calculation OK or not (debug flag)");
    varMakeCellPolygon = debug->addVar("makeCellPolygon", 0.0, "d", "(rw) make avoid cell polygon: 0=no, 1= all cells, 2=traversed cells only (requires rev2cell).");
    varMakeCellPolygonMaxY = debug->addVar("makeCellPolygonMaxY", 30.0, "d", "(rw) Limits the (x and y) size of the cells pollygon, when it should be infinity.");
    varMakeFootprintPolygon = debug->addVar("makeFootprintPolygon", 0.0, "d", "(rw) make polygons during manoeuver calculation to show area travered by robot.");
    varMakeAvoidObstPolygon = debug->addVar("makeAvoidObstPolygon", 0.0, "d", "(r/w) convert all obstacles hold and generated by avoid function to obstacle polygons");
    varMakeCellCostLine = debug->addVar("makeCellCostLine", 1.0, "d", "(rw) debug flag - make polyline from cost-lines during cell A* search (requires rev2cell).");
    varMakeCellVertexLine = debug->addVar("makeCellVertexLine", 1.0, "d", "(rw) debug flag - make polyline from waypoint vertex points from start to exit (requires rev2cell).");
    varDebugDump = debug->addVar("debugDump", 0.0, "d", "(rw) debug flag that will trigger much data beeing flushed to logfile (p.t. avoidCell.log)");
  }
  // add methods
  addMethod("getAvoidPath", "dddd", "Find a path to this (x,y,h) "
                      "pose (in pose coordinates) and the desired exit velocity, "
                      "result is a prioritized array of to UManSeq pointers, each with a full "
                      "manoeuvre sequence, and prioritized for short time");
  addMethodV("getAvoidPath", "cd", "Find an obstacle free path to a pose relative "
      "to the robot. First parameter is the desired (short term) destination pose, "
      "second parameter is the desired exit velocity. The result is returned as a "
      "pointer to the calculated path (if the return structure points to a NULL "
      "pointer), else a copy of the path, if the return structure points to an "
      "UManSeq object. Returns exactly one structure.");
  addMethod("directOK", "dddd", "Find a path to this (x,y,h) "
      "pose (in pose coordinates) and last parameter the desired exit velocity, "
          "result is a prioritized array of to UManSeq pointers, each with a full "
          "manoeuvre sequence, and prioritized for short time");
  addMethodV("directOK", "cd", "Test direct path to a pose relative "
      "to the robot. First parameter is the desired (short term) destination pose, "
          "second parameter is the desired exit velocity. The result is the direct path, "
          "except if the return structure is a UVariable, then "
          "the result is 1 for OK and false if not");
}

///////////////////////////////////////////

UResAvoid::~UResAvoid()
{
  delete paths;
}

///////////////////////////////////////////

const char * UResAvoid::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s reactive path planning\n", preString);
  return buff;
}

///////////////////////////////////////////

bool UResAvoid::setResource(UResBase * resource, bool remove)
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
  if (resource->isA(UResPoseHist::getMapPoseID()))
  {
    result = true;
    if (remove)
      mapPose = NULL;
    else if (mapPose != resource)
      mapPose = (UResPoseHist *)resource;
    else
      // not used
      result = false;
  }
  else if (resource->isA(UResVarPool::getResClassID()))
  {
    result = true;
    if (remove)
      globVar = NULL;
    else if (globVar != resource)
      globVar = (UResVarPool *)resource;
    else
      // not used
      result = false;
  }
  result &= UResVarPool::setResource(resource, remove);
  return result;
}

////////////////////////////////////////////////////

UManSeq * UResAvoid::findPathToHere(UPose exitPose, double endVel, bool exitPoseRel,
                                    UTime * tod, bool directTestOnly)
{
  UManSeq * result = NULL;
  UReactivePath * resRP = NULL;
  UAvoidPath2 * resAP = NULL;
  UVarPool * vp;
  double v1[2];
  int i, n;
  UPoseTime startPose, obstPose, mPose;
  double velocity;
  UPoseV pv;
  bool ignoreObstacles = false;
/*  ULineSegment * seg = NULL;
  UPolygon * poly;*/
  UDataBase * dSeg = NULL;
  UDataBase * dPoly = NULL;
  bool isOK;
  const int MOG = 4;
  int maxOG = 0; // number of obstacle groups used
  //UObstacleGroup * oGps[MOG];
  UDataBase * dOGps[MOG + 1];
  bool useRev2 = false;
  UPosition pos;
  int serial;
  UAvoidParams * par = NULL;
  UObstacleGroup * og;
  double val;
  //
  // set path find parameters
  vp = getVarPool();
  if (vp != NULL)
  {
    par = paths->getParamStruct();
    par->obstMinDist = varObstMinDist->getValued();
    par->obstMinMinDist = varObstMinMinDist->getValued();
    par->maxAcceleration = varMaxAcc->getValued();  // maximum lateral acceleration
    par->maxTurnAcceleration = varMaxTurnAcc->getValued();  // maximum acceleration in turns
    par->minTurnRadius = varMinTurnRadius->getValued(); // minimum allowed turn radius
    par->maxNestedLevels = varMaxNestedSpawns->getInt();  // number of nested levels
    par->maxAvoidLoops = varMaxAvoidLoops->getInt();   // maximum number of loops for midPose search
    par->maxSpawnCnt = varMaxSpawnCount->getInt();   // maximum spawn count limit
    par->obstClearanceMinimum = varClearenceMinimum->getValued(); // minimum allowed clearence to obstacles
    par->obstClearanceDesired = varClearenceDesired->getValued(); // desired clearence to obstacles
    par->frontLeft = varFrontLeft->get3D();
    par->frontRight = varFrontRight->get3D();
    par->doCrashTest = varCrashTest->getValued();   // let crashed paths stay for reporting
    par->useDriveon = varUseDriveon->getBool();
    par->driveonGA = varDriveonGA->getValued();
    par->driveonGD = varDriveonGD->getValued();
    par->forwardOnly = varForwardAngle->getDouble();
    par->maxTangentDepth = varMaxTangentDepth->getInt();
    par->maxTangentToMan = varMaxTangentToMan->getInt();
    par->ignoreCloseObstAfter = varIgnoreCloseObst->getInt();
    par->acceptAfterSolution = varAcceptSolution->getInt();
    par->followLineLastPose = varFollowLineOnLastPose->getBool();
    par->useAnyResult = varUseAnyResult->getBool();
    par->cellRev2 = varRev2cell->getBool();
    par->makeCellPolygon = varMakeCellPolygon->getInt();
    par->makeCellPolygonMaxY = varMakeCellPolygonMaxY->getValued();
    par->makeFootprintPolygon = varMakeFootprintPolygon->getBool();
    if (getGlobalValue("lobst.scan", &val))
    {
      par->scanSerial = int(val);
    }
    //
    ignoreObstacles = varIgnoreObstacles->getValued(); // possible flag to ignore obstacles
    useRev2 = varRev2->getBool();
    n = varMaxOG->getInt();
    maxOG = mini(MOG, n);
    // obstacle avoidance calculation serial number
    serial = varSerial->getInt();  //
    serial += 1;
    varSerial->setInt(serial);
    par->avoidSerial = serial;
    //
    //paths->setParameters(v1, v2, v3, bCrash,
    //                      roundi(bNest), roundi(bLoops), roundi(bSpawn));
  }
  if (poseHist != NULL and globVar != NULL)
  {
    // try to lock neded resources
    lock();
    // set current pose
    startPose = poseHist->getNewest(&velocity);
    if (tod != NULL)
      *tod = startPose.t;
    varCalcTime->setTime(startPose.t);
    if (fabs(velocity) > fabs(endVel))
      // do not use a too high initial velocoty
      // (assumed to be an error, reduce to avoid spin-up of
      // velocity at start of command due to measurement errors.
      // but allow slow reduction in velocity if end velocity is close to zero.
      velocity = endVel + (velocity - endVel) / 2.0;
    // save start pose in obstacle avoid parameter list.
    par->startPose.set(startPose, velocity);
    //
    if (exitPoseRel)
      pv.set(startPose + exitPose, endVel);
    else
      pv.set(exitPose, endVel);
    // get current road lines
    n = 1;
    roadLines.clear();
    obsts.clear();
    if (ignoreObstacles)
      // obstacle situation is just fine
      isOK = true;
    else
    { // get potential obstacle info
      isOK = callGlobal("laserroad.leftSeg", "", NULL, NULL, NULL, &dSeg, &n);
      if (isOK)
        isOK = callGlobal("laserRoad.leftLine", "", NULL, NULL, NULL, &dPoly, &n);
      // else
      //  fprintf(stderr, "UResAvoid::findPathToHere: no road lines (continues)\n");
      if (isOK and dSeg != NULL and dPoly != NULL)
      {
        roadLines.addLeftLine((ULineSegment*)dSeg, (UPolygon*)dPoly);
      }
      if (isOK)
        isOK = callGlobal("laserRoad.rightSeg", "", NULL, NULL, NULL, &dSeg, &n);
      if (isOK)
        isOK = callGlobal("laserRoad.rightLine", "", NULL, NULL, NULL, &dPoly, &n);
      if (isOK and dSeg != NULL and dPoly != NULL)
      {
        roadLines.addRightLine((ULineSegment*)dSeg, (UPolygon*)dPoly);
      }
      // road lines are not strictly needed
      isOK = true;
      //
      // get current obstacles
      if (isOK)
      {
        v1[0] = maxOG; // get (max) this many obstacle groups
        v1[1] = 1.0; // get fixed obstacles too (if any)
        v1[2] = 1.0; // obstacles should be locked
        n = maxOG + 1; // allow for fixed obstacle groups too
        // get relevant obstacle groups locked
        isOK = callGlobal("laserObst.obstacles", "ddd", NULL, v1, NULL, dOGps, &n);
      }
      if (isOK)
      {
        obstPose.t.setTime(0.0);
        for (i = 0; i < n; i++)
        { // add the obstacle groups owned by the obstacle interface
          og = (UObstacleGroup*)dOGps[i];
          // obstacles need to be locked during
          // avoidance path planning
          // (any update to a clocked obstacle is lost (waits for next update))
          obsts.addObstGrp(og);
          if (og->getPoseLast().t > obstPose.t)
            obstPose = og->getPoseLast();
        }
        if (obstPose.t.getSec() < 10)
          obstPose = startPose;
        if (paths->par.logAvoid->isOpen())
        {
          if (n == 0)
            paths->par.logAvoid->toLog("no obstacle groups available (but call is OK)");
          else
            for (i = 0; i < n; i++)
            {
              og = (UObstacleGroup*)dOGps[i];
              paths->par.logAvoid->toLog("Obstacle grp ", og->getSerial(), og->getObstsCnt(), "obstacles");
            }
        }
      }
      else
      { // no obstacles available
        fprintf(stderr, "UResAvoid::findPathToHere: no obstacles available (continues)\n");
        if (paths->par.logAvoid->isOpen())
          paths->par.logAvoid->toLog("no obstacle groups available (call failed)");
        obsts.clear();
        ignoreObstacles = true;
        isOK = true;
      }
    }
    n = 0;
    if (isOK)
    {
      if (useRev2 and not directTestOnly)
      {
        resAP = paths->findPathToHere2(pv, &obsts, &roadLines, ignoreObstacles, varDebugDump->getBool());
        if (resAP != NULL)
        { // copy debug footprint polygons to polygon plug-in
          copyCellPolys(resAP->avCellGraph);
          if (resAP->isValid() and varMakeFootprintPolygon->getBool())
            copyFootprintPolys(resAP, resAP->oldFootCnt);
          if (varMakeAvoidObstPolygon->getBool())
            copyAvoidObstacles(resAP);
          //
          if (resAP->isValid())
            result = resAP->getManSeq();
        }
        n = paths->getValidAvoidPathsCnt();
      }
      else
      {
        resRP = paths->findPathToHere(pv, &obsts, &roadLines, ignoreObstacles, directTestOnly,
                                     par->useDriveon, par->getMinTurnRad());
        if (resRP != NULL)
          result = resRP->getManSeq();
        n = paths->getValidPathsCnt();
      }
    }
    // release resource
    for(i =0; i < obsts.getGroupsCnt(); i++)
    {
      // debug
      // printf("UResAvoid::findPathToHere: Released obstgrp. serial %lu\n", obsts.getGroup(i)->getSerial());
      // debug end
      obsts.getGroup(i)->unlock();
    }
    unlock();
    paths->lock();
    varManCnt->setInt(n, 0);
    paths->unlock();
  }
  else
    printf("UResAvoid::findPathToHere: is missing resources\n");
  return result;
}

/////////////////////////////////////////////////////

bool UResAvoid::methodCall(const char * name, const char * paramOrder,
                               char ** strings, const double * pars,
                               double * value,
                               UDataBase ** returnStruct,
                               int * returnStructCnt)
{
  bool result = true;
  UPose pose;
  //UReactivePath * path;
  UManSeq * path;
  UTime t;
  //double minTurnRad;
  // evaluate standard functions
  if ((strcasecmp(name, "getAvoidPath") == 0) and (strcmp(paramOrder, "dddd") == 0))
  {
    if ((pars != NULL) and (returnStruct != NULL) and (returnStructCnt != NULL))
    { // a calculation is needed
      pose.set(pars[0], pars[1], pars[2]);
      // exit pose is in odometry coordinates (rel=false)
      // debug
      //minTurnRad = paths->par.minTurnRadius;
      //if (minTurnRad < 2)
      //  printf("debug 1: not usable for hako!\n");
      // debug end
      path = findPathToHere(pose, pars[3], false, &t, false);
      // return 'true' value is a path is found
      if (paths->par.logAvoid->isOpen() and path != NULL)
      {
        // debug
        //minTurnRad = paths->par.minTurnRadius;
        //if (minTurnRad < 2)
        //  printf("debug 2: not usable for hako!\n");
        // debug end
        path->print("getAvoidPath", sBuff, MSL);
        paths->par.logAvoid->toLog(sBuff);
      }
      *value = (path != NULL);
      // return also a pointer to the man sequence
      if (*returnStructCnt > 0 and path != NULL)
      {
        *returnStructCnt = 1;
        returnStruct[0] = path;
        //returnStruct[0] = path->getManSeq();
      }
    }
  }
  else if ((strcasecmp(name, "directOK") == 0) and (strcmp(paramOrder, "dddd") == 0))
  {
    if ((pars != NULL) and (returnStruct != NULL) and (returnStructCnt != NULL))
    { // a calculation is needed
      pose.set(pars[0], pars[1], pars[2]);
      // exit pose is in odometry coordinates (rel=false)
      path = findPathToHere(pose, pars[3], false, &t, true);
      if (paths->par.logAvoid->isOpen() and path!= NULL)
      {
        // debug
        //minTurnRad = paths->par.minTurnRadius;
        //if (minTurnRad < 2)
        //  printf("debug: not usable for hako!\n");
        // debug end
        path->print("directOK", sBuff, MSL);
        paths->par.logAvoid->toLog(sBuff);
      }
      // return 'true' value is a path is found
      if (path != NULL and poseHist != NULL)
      {
        if (path->getMaxTurnArc() > M_PI/2.0 + fabs(limitToPi(poseHist->getNewest().h - pose.h)))
          // there is probable a 360 deg turn - then not direct
          *value = false;
        else
          *value = true;
      }
      else
        *value = false;
      // return also a pointer to the man sequence
      if (*returnStructCnt > 0 and path != NULL)
      {
        *returnStructCnt = 1;
        returnStruct[0] = path;
        //returnStruct[0] = path->getManSeq();
      }
    }
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////

bool UResAvoid::methodCallV(const char * name, const char * paramOrder,
                                UVariable * params[],
                                UDataBase ** returnStruct,
                                int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  UTime t;
  UPose pose1, pose2;
  bool setStructCnt = false;
  UManSeq * path;
  const int MSL = 10000;
  char s[MSL];
  // debug
  //double minTurnRad = 0.0;
  // debug end
  //
  if ((strcasecmp(name, "getAvoidPath") == 0) and (strcmp(paramOrder, "cd") == 0))
  {
    pose1 = params[0]->getPose();
    // exit pose is in odometry coordinates (rel=false)
    // debug
    //minTurnRad = paths->par.minTurnRadius;
    //if (minTurnRad < 2)
    //  printf("debug (V3): not usable for hako!\n");
    // debug end
    path = findPathToHere(pose1, params[1]->getValued(), false, &t, false);
    if (paths->par.logAvoid->isOpen() and path!= NULL)
    {
      // debug
      //minTurnRad = paths->par.minTurnRadius;
      //if (minTurnRad < 2)
      //  printf("debug (V4): not usable for hako!\n");
      // debug end
      path->print("getAvoidPath(V)", s, MSL);
      paths->par.logAvoid->toLog(s);
    }
      // return also a pointer to the man sequence
    if (*returnStructCnt > 0)
    {
      if (returnStruct[0] == NULL)
        // keep found path in the present structure
        returnStruct[0] = path;
      else if (returnStruct[0]->isA("manseq"))
        // want a copy of man sequence
        ((UManSeq*)returnStruct[0])->copy(path);
      else if (returnStruct[0]->isA("var"))
        // want a copy of man sequence
        ((UVariable*)returnStruct[0])->setBool(path != NULL, 0);
      else
        printf("getAvoidPath(dd) requires a NULL a UManSeq* or a UVariable* as first return parameter\n");
      *returnStructCnt = 1;
    }
  }
  else if ((strcasecmp(name, "directOK") == 0) and (strcmp(paramOrder, "cd") == 0))
  {
    pose1 = params[0]->getPose();
    pose2 = poseHist->getNewest(); // current pose
    // exit pose is in odometry coordinates (rel=false)
      // debug
    //minTurnRad = paths->par.minTurnRadius;
/*    if (minTurnRad < 2)
      printf("debug (V2): not usable for hako!\n");*/
      // debug end
    path = findPathToHere(pose1, params[1]->getValued(), false, &t, true);
    if (paths->par.logAvoid->isOpen() and path!= NULL)
    {
      // debug
      //minTurnRad = paths->par.minTurnRadius;
/*      if (minTurnRad < 2)
        printf("debug (V2): not usable for hako!\n");*/
      // debug end
      path->print("directOK(V)", s, MSL);
      paths->par.logAvoid->toLog(s);
    }
    // return also a pointer to the man sequence
    if (*returnStructCnt > 0)
    {
      if (returnStruct[0] == NULL)
      { // returns path in the present structure
        returnStruct[0] = path;
        path = NULL;
        //leave the path locked until no longer used
      }
      else if (returnStruct[0]->isA("manseq"))
      { // want a copy of man sequence
        if (path != NULL)
          ((UManSeq*)returnStruct[0])->copy(path);
        else
          // remove any old path in this structure
          ((UManSeq*)returnStruct[0])->releaseAllMan();
      }
      else if (returnStruct[0]->isA("var"))
      { // want a to know if path is direct
        if (path != NULL)
        {
          if (false) // path->getMaxTurnArc() > (M_PI*0.7 +
                     // fabs(limitToPi(pose2.h - pose1.h))))
          {
            // debug
            printf("Man failed on angle limit: maxArc=%g from %g,%g,%g to %g,%g,%g\n",
                   path->getMaxTurnArc(),
                   pose2.x, pose2.y, pose2.h,
                   pose1.x, pose1.y, pose1.h);
            // debug end
            // there is probable a 360 deg turn - then not direct
            ((UVariable*)returnStruct[0])->setBool(false);
          }
          else
            ((UVariable*)returnStruct[0])->setBool(true);
        }
        else
          ((UVariable*)returnStruct[0])->setBool(false);
      }
      else
      {
        printf("directOK(dd) requires a NULL a UManSeq* or a UVariable* as first return parameter\n");
      }
      if (path != NULL)
      { // release path
        path->unlock();
      }
      *returnStructCnt = 1;
    }
  }
  else
    result = false;
  //
  if (not setStructCnt)
    *returnStructCnt = 1;
  return result;
}

/////////////////////////////////////////////

bool UResAvoid::useRev2()
{
  return varRev2->getBool();
}

/////////////////////////////////////////////

bool UResAvoid::openLogAvoid()
{
  if (paths->par.logAvoid->isOpen())
    paths->par.logAvoid->closeLog();
  paths->par.logAvoid->openLog();
  return paths->par.logAvoid->isOpen();
}

/////////////////////////////////////////////

void UResAvoid::closeLogAvoid()
{
  if (paths->par.logAvoid != NULL)
    paths->par.logAvoid->closeLog();
}

/////////////////////////////////////////////////

void UResAvoid::copyFootprintPolys(UAvoidPath2 * resAP, int oldCnt)
{
  UResBase * pres;
  int i, n;
  const int MSL = 30;
  char s[MSL];
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase * db, *dbr;
  bool isOK;
  //
  pres = getStaticResource("poly", false, false);
  if (pres != NULL)
  { // delete old footprint polygons in polygon resource
    snprintf(s, MSL, "footprint.*");
    vs.setValues(s, 0, true);
    par[0] = &vs;
    dbr = &vr;
    isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
    //
    // set polygon coordinate system to odometry (0=odo, 1=utm, 2=map)
    vCoo.setDouble(0.0);
    par[2] = &vCoo;
    for (i = 0; i < resAP->polysCnt; i++)
    {
      snprintf(s, MSL, "footprint.%03d", i);
      vs.setValues(s, 0, true);
      db = resAP->polys[i];
      par[1] = (UVariable *) db;
      isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
      if ((not isOK and i == 0) or not vr.getBool())
        printf("UResAvoid::copyFootprintPolys: failed to 'poly.setPoly(%s, %d)'\n", s, i);
    }
/*    while (i < oldCnt)
    { // delete remaining footprint polygons
      snprintf(s, MSL, "footprint.%03d", i);
      vs.setValues(s, 0, true);
      n = 1;
      isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
      if (not isOK or not vr.getBool())
        printf("UResAvoid::copyFootprintPolys: failed to 'poly.del(%s)'\n", s);
      i++;
    }*/
  }
}

////////////////////////////////////////////////////////////


void UResAvoid::copyAvoidObstacles(UAvoidPath2 * resAP)
{
  UResBase * pres;
  int i, n;
  const int MSL = 30;
  char s[MSL];
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase * db, *dbr;
  bool isOK;
  UAvoidObst * og;
  //
  pres = getStaticResource("poly", false, false);
  if (pres != NULL)
  { // delete old footprint polygons in polygon resource
    snprintf(s, MSL, "aObst.*");
    vs.setValues(s, 0, true);
    par[0] = &vs;
    dbr = &vr;
    isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
    //
    // set polygon coordinate system to odometry (0=odo, 1=utm, 2=map)
    vCoo.setDouble(0.0);
    par[2] = &vCoo;
    for (i = 0; i < resAP->aogsCnt; i++)
    {
      og = resAP->aogs[i];
      while (og != NULL)
      {
        if (og->obst->getPointsCnt() > 2)
        {
          snprintf(s, MSL, "aObst.%03d.%03lu", i, og->obst->getSerial());
          vs.setValues(s, 0, true);
          switch (i)
          {
            case 0:  og->obst->color[0] = 'b'; break;
            case 1:  og->obst->color[0] = 'r'; break;
            case 2:  og->obst->color[0] = 'm'; break;
            case 3:  og->obst->color[0] = 'c'; break;
            case 4:  og->obst->color[0] = 'g'; break;
            default: og->obst->color[0] = (i % 10) + '0'; break;
          }
          og->obst->color[1] = '1'; // pixels wide
          og->obst->color[2] = '-'; // no marker
          og->obst->color[3] = 'd'; // default line style
          db = og->obst;
          par[1] = (UVariable *) db;
          isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
          if ((not isOK and i == 0) or not vr.getBool())
            printf("UResAvoid::copyFootprintPolys: failed to 'poly.setPoly(%s, %d)'\n", s, i);
        }
        og = og->grp;
      }
    }
/*    while (i < oldCnt)
    { // delete remaining footprint polygons
      snprintf(s, MSL, "footprint.%03d", i);
      vs.setValues(s, 0, true);
      n = 1;
      isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
      if (not isOK or not vr.getBool())
        printf("UResAvoid::copyFootprintPolys: failed to 'poly.del(%s)'\n", s);
      i++;
    }*/
  }
}


/////////////////////////////////////////////////

void UResAvoid::copyCellPolys(UAvoidCellGraph * avcg)
{
  UResBase * pres;
  int i = 0, n;
  const int MSL = 30;
  char s[MSL];
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase * db, *dbr;
  bool isOK;
  UPolygon40 poly;
  //
  pres = getStaticResource("poly", false, false);
  if (pres != NULL and avcg != NULL)
  { // delete old footprint polygons in polygon resource
    snprintf(s, MSL, "avoid*");
    vs.setValues(s, 0, true);
    par[0] = &vs;
    dbr = &vr;
    isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
     //
    // set polygon coordinate system to odometry (0=odo, 1=map, 3=utm)
    vCoo.setDouble(0.0);
    par[2] = &vCoo;
    if (varMakeCellPolygon->getInt() == 1)
    { // all cells
      for (i = 0; i < avcg->getCellsCnt(); i++)
      {
        isOK = avcg->getCellPoly(i, &poly, varMakeCellPolygonMaxY->getValued());
        if (isOK)
        {
          poly.color[0] = (i % 10) + '0';
          poly.color[1] = '1'; // two pixels wide
          poly.color[2] = '-'; // no marker
          poly.color[3] = 'd'; // default line style
          snprintf(s, MSL, "avoidCell.%03d", i);
          vs.setValues(s, 0, true);
          db = &poly;
          par[1] = (UVariable *) db;
          isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
          if ((not isOK and i == 0) or not vr.getBool())
            printf("UResAvoid::copyCellPolys: failed to 'poly.setPoly(%s, %d)'\n", s, i);
        }
      }
    }
    else if (varMakeCellPolygon->getInt() == 2)
    { // just used cells
      for (i = 0; i < avcg->getCellsPathCnt(); i++)
      {
        int n;
        poly.clear();
        n = avcg->getCellPathPoly(i, &poly, varMakeCellPolygonMaxY->getValued());
        poly.color[0] = (i % 10) + '0';
        poly.color[1] = '1'; // two pixels wide
        poly.color[2] = '-'; // no marker
        poly.color[3] = 'd'; // default line style
        snprintf(s, MSL, "avoidPathCell-%d.%03d", n, i);
        vs.setValues(s, 0, true);
        db = &poly;
        par[1] = (UVariable *) db;
        isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
        if ((not isOK and i == 0) or not vr.getBool())
          printf("UResAvoid::copyCellPolys: failed to 'poly.setPoly(%s, %d)'\n", s, i);
      }
    }
    if (avcg->polyCell.getPointsCnt() > 0 and varMakeCellCostLine->getBool())
    {
      avcg->polyCell.color[0] = 'p'; // pink
      avcg->polyCell.color[1] = '2'; // two pixels wide
      avcg->polyCell.color[2] = 'o'; // circle at vertex
      avcg->polyCell.color[3] = 'd'; // default line style
      vs.setValues("avoidRawCellPath", 0, true);
      db = &avcg->polyCell;
      par[1] = (UVariable *) db;
      isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
      if ((not isOK and i == 0) or not vr.getBool())
        printf("UResAvoid::copyCellPolys: failed to copy raw cell path\n");
    }
    if (avcg->polyVertex.getPointsCnt() > 0 and varMakeCellVertexLine->getBool())
    {
      avcg->polyVertex.color[0] = 'b'; // blue
      avcg->polyVertex.color[1] = '2'; // two pixels wide
      avcg->polyVertex.color[2] = 'o'; // circle at vertex
      avcg->polyVertex.color[3] = 'd'; // default line style
      vs.setValues("avoidVertexPath", 0, true);
      db = &avcg->polyVertex;
      par[1] = (UVariable *) db;
      isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
      if ((not isOK and i == 0) or not vr.getBool())
        printf("UResAvoid::copyCellPolys: failed to copy raw cell path\n");
    }
  }
}

/////////////////////////////////////////////////////////

const char * UResAvoid::codeXmlRobotFront(char * buff, const int buffCnt)
{
  UPose rPose, po1;
  char * p1 = buff;
  int n = 0;
  UPosition frontLeft, frontRight;
  //
  frontLeft = varFrontLeft->get3D();
  frontRight = varFrontRight->get3D();
  rPose = poseHist->getNewest();
  po1 = rPose.getPoseToMapPose(frontLeft.x, frontLeft.y, M_PI);
  po1.codeXml("frLeft", p1, buffCnt - n, NULL);
  n += strlen(p1);
  p1 = &buff[n];
  po1 = rPose.getPoseToMapPose(frontRight.x, frontRight.y, M_PI);
  po1.codeXml("frRght", p1, buffCnt - n, NULL);
  //
  return buff;
}
