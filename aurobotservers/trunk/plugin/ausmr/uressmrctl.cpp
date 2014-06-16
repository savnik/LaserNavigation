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
#include <umap4/umanseq.h>
#include <urob4/uresposehist.h>

#include "uressmrctl.h"

void UResSmrCtl::UResSmrCtlInit()
{ // these first two lines is needed
  // other local initializations
  createVarSpace(20, 0, 0, "Non real-time robot controller", false);
  createBaseVar();
  //
  smrif = NULL;
  currentPlan = NULL;
  currentPlanNew = false;
  implementedPlan = NULL;
  driver = "none";
  stopFlag = false;
  running = false;
  driveCmdFinishedID = -1;
  poseHist = NULL;
  logctl.setLogName("mrcctl");
  logprim.setLogName("mrcprim");
  //logctl.openLog();
  // start the control loop
  start();
}

///////////////////////////////////////////

UResSmrCtl::~UResSmrCtl()
{
  stop(true);
  logctl.closeLog();
  logprim.closeLog();
}

///////////////////////////////////////////

void UResSmrCtl::createBaseVar()
{
  addVar("version", getResVersion() / 100.0, "d", "(r) version of this resource");
  varRunning = addVar("running", 0.0, "d", "(r) Is control loop (thread) running");
  varManEndID = addVar("manEndID", -1.0, "d", "(r) Event ID posted to mark end of current manoeuvre (see smr.idUserEvent)");
  varManEndTime = addVar("manEndTime", 0.0, "d", "(r) Event ID posted at this time");
  varManQueueingTime = addVar("manQueueingTime", 0.5, "d", "(r) Time to queue last manouvre");
  varRenewDriveDist = addVar("manRenewDriveDist", 1.2, "d", "(rw) Max distance to travel using the same old manoeuver plan, when the new plan looks the same.");
  varRenewDestDist = addVar("renewDestDist", 0.1, "d", "(rw) Max destination distance change (m) before command is renewed.");
  varRenewDestHeading = addVar("renewDestHeading", 0.1, "d", "(rw) Max destination heading change (radians) before command is renewed.");
  varRenewDestVel = addVar("renewDestVel", 0.1, "d", "(rw) Max destination velocity change (m/s) berfore command is renewed.");
  varManPlanDist = addVar("manPlanDist", 2.2, "d", "(rw) Max distance to use of a plan shut-down is implemented.");
  varUseDriveonHeading = addVar("useDriveonHeading", 0.78, "d", "(r) Max heading deviation relative to end pose for using direct drive (rather than arc-line-arc).");
  varUseDriveonDist = addVar("useDriveonDist", 0.1, "d", "(r) Max distance from end pose line where a driveon command should be used (rather than arc-line-arc).");
  varManIsDirect = addVar("manIsDirect", -1.0, "d", "(r) Is the received plan direct - i.e. one turn-drive-turn set only (-1 os no plan)");
  varUseDriveonIfDirect = addVar("useDriveonIfDirect", 0.0, "d", "(r/w) Use a driveon command, if the manIsDirect flag is true");
  varDirectOrWait = addVar("directOrWait", 0.0, "d", "(rw) Are we keeping a path with no space for avoidance manoeuvres, then set this flag to true.");
  varDirectWait = addVar("directWait", 0.0, "d", "(r) Are we waiting for a direct manoeuvre (and discarded last manoeuvre).");

  //
  addMethod("setPlan", "sc", "Set the path plan to be implemented."
                      " the first string parameter is a pointer to a driver name,"
                      " the second parameter is a structure 'UManSeq' holding"
                      " the lines and arcs to follow");
  addMethod("driver", "", "Get current driver name as the first returned structure");
  addMethod("finished", "", "Checks the ID that is finished from the smr with the "
                      "last of the main commands posted to the smr. If this is finished, "
                      "then the returned value is true (1) else false (0)");
}

///////////////////////////////////////////

const char * UResSmrCtl::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s smr current driver is '%s'\n", preString,
           driver);
  return buff;
}

///////////////////////////////////////////

bool UResSmrCtl::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  bool isUsed = false;
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
  else if (resource->isA(UResSmrIf::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    result = true;
    if (remove)
      smrif = NULL;
    else if (smrif != resource)
      smrif = (UResSmrIf *) resource;
    else
      result = false;
  }
  isUsed = UResVarPool::setResource(resource, remove);
  //
  return result or isUsed;
}

///////////////////////////////////////////

bool UResSmrCtl::methodCall(const char * name, const char * paramOrder,
                               char ** strings, const double * pars,
                               double * value,
                               UDataBase ** returnStruct,
                               int * returnStructCnt)
{
  bool result = true;
  char * driver = NULL;
  UDataString * str;
  int v;
  // evaluate standard functions
  if ((strcasecmp(name, "setPlan") == 0) and (strcmp(paramOrder, "sc") == 0))
  { // the provided class is a UManSeq with the new path
    if (strings != NULL)
      driver = strings[0];
    *value = implementDrivePath(driver, (UManSeq*)returnStruct[0]);
    *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "finished") == 0) and (strcmp(paramOrder, "") == 0))
  { // is the last driver sequence finished, i.e. the last 2 meter of drive commands
    // doUserEvent return
    // 0 = not valid
    // 1 found but not finished
    // 2 found and finished
    v = roundi(smrif->doUserEvent(1));
    if (v == 2)
      *value = 1.0;
    else
      *value = 0.0;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "driver") == 0) and (strlen(paramOrder) == 0))
  { // the provided class is a UManSeq with the new path
    *value = 0.0;
    if (returnStructCnt != NULL)
    {
      if (*returnStructCnt > 0)
      { // return structs are available
        if (returnStruct[0] != NULL)
        { // first struct is not null, but is it a string
          if (returnStruct[0]->isA("string"))
          { // a string struct is available - set value into this
            str = (UDataString *) returnStruct[0];
            if (driver != NULL)
              // we have a known driver - e.g. drivepos or roaddrive
              str->setStr((char *)driver, strlen(driver));
            else
              // noone claims to be driver
              str->makeString("NONE");
            *returnStructCnt = 1;
            *value = 1.0;
          }
          else
            // nothing returned
            *returnStructCnt = 0;
        }
         else
           // nothing returned
           *returnStructCnt = 0;
      }
    }
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////////////////

bool UResSmrCtl::implementDrivePath(const char * driverName, UManSeq * man)
{ //
  if (currentPlan == NULL)
    currentPlan = new UManSeq();
  //
  if (not currentPlan->tryLock())
  { // debug
    printf("UResSmrCtl::implementDrivePath: (seq thread) waiting for current plan\n");
    // debug end
    currentPlan->lock();
    // debug
    printf("UResSmrCtl::implementDrivePath: (seq thread) current plan free\n");
    // debug end
  }
  driver = driverName;
  currentPlan->copy(man);
  currentPlanNew = true;
  currentPlan->unlock();
  //
  return true;
}

///////////////////////////////////////////////////////////

bool UResSmrCtl::sendNewManoeuvreToSMR(UManSeq * manseq)
{
  const int MCL = 10000;
  char cmd[MCL];
  double maxAcc, maxTurnAcc;
  UManPPSeq * mpp;
  bool result;
  double vel, dist, manDist1, manDist2;
  UPoseV startPose, endPose, pv;
  UPoseTime pose;
  int i, j;
  UManoeuvre * mm;
  double manDist; // distance of manoeuvre
  double orderDist; // length [m] of path send to MRC
  double distSum = 0.0;
  const int MAX_CMD_LINE_CNT = 10; // command sequence
  int lineCnt = 0;
  const double FIRST_DISTANCE = 0.12; // [m] no target position before this much ahead
  bool first = true;
  UTime t, tq;
  double minTurnRad;
  double okVel, d, dd, hh;
  bool isLast;
  //double smrTimeout = 0.5; // sec
  const char * p1;
  char * p2 = cmd;
  bool userEventSet = false;
  bool lastOrder;
  bool useDriveon;
  //
  maxAcc = smrif->getAcc();
  maxTurnAcc = smrif->getTurnAcc();
  minTurnRad = smrif->getMinTurnRad();
  pose = poseHist->getNewest(&vel);
  useDriveon = varManIsDirect->getBool() and varUseDriveonIfDirect->getBool();
  if (logctl.isOpen())
  {
    fprintf(logctl.getF(), "%lu.%06lu current odopose %g %g %g\n",
            pose.t.getSec(), pose.t.getMicrosec(),
                          pose.x, pose.y, pose.h);
    pv = manseq->getStartPoseV();
    fprintf(logctl.getF(), "%lu.%06lu man startpose %g %g %g %g\n",
            pose.t.getSec(), pose.t.getMicrosec(),
            pv.x, pv.y, pv.h, pv.vel);
    pv = manseq->getEndPoseV();
    fprintf(logctl.getF(), "%lu.%06lu man endpose %g %g %g %g\n",
            pose.t.getSec(), pose.t.getMicrosec(),
                          pv.x, pv.y, pv.h, pv.vel);
    manseq->print("man-org", cmd, MCL);
    logctl.toLog(cmd);
  }
  // recalclate first sequence to curerent robot pose
  mpp = manseq->getP2P(0);
  result = (mpp != NULL);
  if (result)
  {
    startPose.set(pose.x, pose.y, pose.h, vel);
    endPose = mpp->getEndPoseV();
  }
  // may avoidance manoeuvres be used?
  if (result and varDirectOrWait->getBool() and manseq->getP2PCnt() > 1)
  { // manoeuver is complicated, and we asked for direct
    result = false;
    varDirectWait->setValued(1);
  }
  else if (varDirectWait->getBool())
    varDirectWait->setValued(0);
  //
  // recalculate start using current pose
  if (result and not useDriveon)
  { // manoeuver is calculated using old data
    // recalculate first manoeuvre using current pose.
    // should start of manoeuvre be recalculated?
    d = pose.getDistance(manseq->getStartPoseV());
    if (d > 0.05 and not useDriveon)
    { // recalculate
      dist = startPose.getDistance(endPose);
      manDist1 = manseq->getDistance();
      if (dist < 2.0 * minTurnRad)
      // getting close allow tighter turns to avoid no-solution lockups
        minTurnRad *= 0.6;
      // is this last manoeuvre in sequence (then more relaxed end pose requirements)
      isLast = (manseq->getP2PCnt() == 1);
      //reevaluate first manoeuvre, to include ...
      // ... new distance to start of manoeuvre.
      result = manseq->expandMan(startPose, endPose, isLast, maxAcc, maxTurnAcc, minTurnRad, mpp, &okVel);
      if (false and logctl.isOpen())
      { // log the new values
        manDist2 = manseq->getDistance();
        fprintf(logctl.getF(), "%lu.%06lu oldman manDist=%5.2f dist=%g isLast=%s\n",
                pose.t.getSec(), pose.t.getMicrosec(),
                manDist1, dist, bool2str(isLast));
        fprintf(logctl.getF(), "%lu.%06lu newMan manDist=%5.2f minTurnRad=%5.2f result=%s\n",
                pose.t.getSec(), pose.t.getMicrosec(),
                manDist2, minTurnRad, bool2str(result));
      }
    }
  }
  if (result)
  {  // save the time it takes to queue the total manoeuvre
    tq.now();
    // flush old commands in MRC
    //result = smrif->sendSMRgetId("flushcmds", smrTimeout, NULL, false);
    result = smrif->sendString("flushcmds\n");
    result = smrif->sendString("targethere\n");
    if (logctl.isOpen())
    { // log first manoeuver, as it stands now
      manseq->print("man-after", cmd, MCL);
      logctl.toLog(cmd);
      logctl.toLog("flushcmds");
      logctl.toLog("targethere");
    }
    if (logprim.isOpen())
    { // save new drive scope
      fprintf(logprim.getF(), "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
              pose.t.getSec(), pose.t.getMicrosec(),
              1,
              startPose.x, startPose.y, startPose.h, startPose.vel,
              endPose.x, endPose.y, endPose.h, endPose.vel);
    }
    // take this pose as reference (especially heading)
    // this should in most cases not be needed
    // result = smrif->sendSMRgetId("targethere", smrTimeout, NULL, true);
    // get length of manoeuvre
    manDist = manseq->getDistance();
    // but limit to a 'got-lost' distance
    lastOrder = (manDist < varManPlanDist->getValued()*1.1);
    if (lastOrder)
    { // leave space for last order
      orderDist = manDist - varManPlanDist->getValued() * 0.5;
    }
    else
    { // implement the first part of manoeuver only
      orderDist = varManPlanDist->getValued();
    }
    if (orderDist > 0.1)
    { // expad to string
      // first test if better server using a line drive only
      dd = manseq->getDistanceFromEndPoseLine(&hh);
      if (useDriveon or
          (fabs(dd) < varUseDriveonDist->getValued() and
           fabs(hh) < varUseDriveonHeading->getValued() and
           (lastOrder or manseq->getP2PCnt() == 1)))
      { // replace all manoeuvres with a single drive command
        // direct driveon
        getSMRCLDrive2cmd(cmd, MCL, startPose, endPose, orderDist/2.0, orderDist, lastOrder, &pose.t);
        //
        p2 = cmd;
        while (p2 != NULL)
        {
          p1 = strsep(&p2, "\n");
          if (*p1 == '\t')
          { // place a break event her to indicat that it is time for next command
            driveCmdFinishedID = smrif->sendUserEvent("", -1);
            if (logctl.isOpen())
              logctl.toLog("userevent", driveCmdFinishedID, "last", bool2str(lastOrder));
            userEventSet = true;
          }
          else
          { // normal command line - send with a termination new-line
            result = smrif->sendString(p1, "\n");
            if (logctl.isOpen())
              logctl.toLog(p1);
          }
        }
      }
      else
      { // use full sequence of manoeuvres in 'manseq'
        for (i = 0; i < manseq->getP2PCnt(); i++)
        {
          mpp = manseq->getP2P(i);
          // debug workaround
          /// @todo fix this the right way - ensure it never happens
          mpp->setMaxVel(smrif->getSpeed());
          // end workaround
          startPose = mpp->getStartPoseV();
          for (j = 0; j < mpp->getSeqCnt(); j++)
          {
            mm = mpp->getMan(j);
            mm->getSMRCLcmd2(cmd, MCL, &startPose,
                            &first, FIRST_DISTANCE,
                            &lineCnt, MAX_CMD_LINE_CNT,
                            &distSum, orderDist,
                            &pose.t, logprim.getF());
            if (not first) // skip first command
            { // command is usable, send to MRC - one line at a time
              p2 = cmd;
              while (p2 != NULL)
              {
                p1 = strsep(&p2, "\n");
                if (*p1 == '\t')
                {
                  driveCmdFinishedID = smrif->sendUserEvent("", -1);
                  if (logctl.isOpen())
                    logctl.toLog("userevent", driveCmdFinishedID, j, "");
                  userEventSet = true;
                }
                else
                {
                  //result = smrif->sendSMRgetId(p1, smrTimeout, NULL, true);
                  result = smrif->sendString(p1, "\n");
                  if (logctl.isOpen())
                    logctl.toLog(p1);
                }
              }
            }
            if (lineCnt >= MAX_CMD_LINE_CNT)
              break;
            if (distSum >= orderDist)
              break;
          }
          if (lineCnt >= MAX_CMD_LINE_CNT)
            break;
          if (distSum >= orderDist)
            break;
        }
      }
    }
    else
      orderDist = 0.0;
    //
    // now all the main drive commands are finished - put a user marker here
    // if path is not ending
    if (not userEventSet)
      driveCmdFinishedID = smrif->sendUserEvent("", -1);
    /// @todo set time - send or not - helps during replay, but should
    /// realy set replay time (from smr-IO log?- once this is available)
    driveCmdFinishedIDTime.Now();
    varManEndTime->setValued(driveCmdFinishedIDTime.getDecSec(), 0);
    // if not sent user event, ID is -1
    if (driveCmdFinishedID < 0)
      printf("UResSmrCtl::sendNewManoeuvreToSMR failed to pose a SMRCL user event\n");
    else
    { // save also the time - to disallow too rapid change
      varManEndID->setValued(driveCmdFinishedID, 0);
      driveCmdStartPose = manseq->getStartPoseV();
    }
    // continue straight for a short while (30cm) at current speed
    // to allow new plan to be provided
    if (lastOrder)
    { // ordered distance ends just before the target, so extend
      // the target by a small drive to target pose at current speed
      snprintf(cmd, MCL, "driveon %g %g %g \"rad\" : ($targetdist < 0.0)",
               endPose.x, endPose.y, endPose.h);
      //result = smrif->sendSMRgetId(cmd, smrTimeout, NULL, true);
      result = smrif->sendString(cmd, "\n");
      if (logctl.isOpen())
        logctl.toLog(cmd);
    }
    // then stop the robot queietly
    snprintf(cmd, MCL, "fwd 0.45 @v0.2 @a0.5 : ($odovelocity < 0.3)");
    //result = smrif->sendSMRgetId(cmd, smrTimeout, NULL, true);
    result = smrif->sendString(cmd, "\n");
    if (logctl.isOpen())
      logctl.toLog(cmd);
    // stop completely
    snprintf(cmd, MCL, "idle");
    //result = smrif->sendSMRgetId(cmd, smrTimeout, NULL, true);
    result = smrif->sendString(cmd, "\n");
    if (logctl.isOpen())
      logctl.toLog(cmd);
    //
    // save queueing time
    varManQueueingTime->setValued(tq.getTimePassed(), 0);
  }
  // save a copy of the implemented plan
  if (implementedPlan == NULL)
    implementedPlan = new UManSeq();
  implementedPlan->copy(manseq);
  //
  return result;
}

////////////////////////////////////////////

void *startDrive(void *ptr)
{ // called by create_thread
  UResSmrCtl * obj;
  // pointer is an UResSmrCtl instance
  obj = (UResSmrCtl *) ptr;
  // run main loop
  obj->run();
  return NULL;
}

/////////////////////////////////////////////

bool UResSmrCtl::start()
{
  bool result = false;
  pthread_attr_t  thConAttr;
  int err;
  //
  if (not varRunning->getBool())
  {
    stopFlag = false;
    // Starts socket client thread 'runSockClient'
    pthread_attr_init(&thConAttr);
    // create socket client thread
    err = pthread_create(&thDrive, &thConAttr, &startDrive, (void *)this);
    result = (err == 0);
    pthread_attr_destroy(&thConAttr);
  }
  //
  return result;
}

/////////////////////////////////////////////

void UResSmrCtl::stop(bool andWait)
{
  stopFlag = true;
  if (running)
  {
    if (andWait)
    {
      printf("UResSmrCtl:: stopping ctl loop ...\n");
      pthread_join(thDrive, NULL);
      printf("\t[OK]\n");
    }
    else
      Wait(0.05);
  }
}

/////////////////////////////////////////////


void UResSmrCtl::run()
{
  const char * oldDriver = NULL;
  bool newDriver;
  bool implementNewPlan;
  double d;
  UPosition pos;
  UPoseTime pose;
  UPoseV poseV, poseVD;
  UPose pose2;
/*  const double MAX_OFF_DISTANCE = 0.1;  // meter away from current line
  const double MAX_OFF_HEADING  = 0.1;  // radians (6 deg)
  const double MAX_OFF_VEL      = 0.05; // m/s*/
  UManPPSeq *mImpl, *mNew;
  //
  running = true;
  varRunning->setValued(true, 0);
  while (not stopFlag)
  { // test driver status
    newDriver = (oldDriver != driver);
    if (newDriver)
      oldDriver = driver;
    // test for new route plan
    if (currentPlanNew)
    {
      if (not currentPlan->tryLock())
      { // a debug printout if plan is locked already
        printf("UResSmrCtl::run: man found locked - waiting for it to be freed\n");
        currentPlan->lock();
      }
      varManIsDirect->setValued(currentPlan->getP2PCnt() == 1);
      currentPlanNew = false;
      // implement always if new driver
      implementNewPlan = newDriver or implementedPlan == NULL;
      // or if too old (based on distance)
      if (not implementNewPlan)
      { // test distance traveled since last update
        pose = poseHist->getNewest(NULL);
        d = driveCmdStartPose.getDistance(&pose);
        if (d > varRenewDriveDist->getValued())
          // old plan is almost ended, so renew
          implementNewPlan = true;
        else
        { // or if too much change in plans
          // compare new and implemented plan
          mImpl = implementedPlan->getP2P(0);
          mNew = currentPlan->getP2P(0);
          if (mImpl == NULL or mNew == NULL)
            printf("UResSmrCtl::run found no plan set: Impl=%s, New=%s !!!\n",
                   bool2str(mImpl==NULL), bool2str(mNew == NULL));
          else
          { // OK to test for potential implement
            poseV = mNew->getEndPoseV();
            d = mImpl->getEndPoseV().getDistToPoseLine(poseV.x, poseV.y);
            poseVD = poseV - mImpl->getEndPoseV();
            //d = hypot(poseV.x, poseV.y);
            if (d > varRenewDestDist->getValued())
              implementNewPlan = true;
            else if (absf(poseVD.h) > varRenewDestDist->getValued())
              implementNewPlan = true;
            else if (absf(poseVD.getVel()) > varRenewDestVel->getValued())
              implementNewPlan = true;
          }
        }
      }
      if (implementNewPlan)
      { // skip old plan and send this
        sendNewManoeuvreToSMR(currentPlan);
      }
      currentPlan->unlock();
    }
    else
      Wait(0.03);
    newDriver = false;
  }
  varRunning->setValued(false, 0);
  running = false;
}

////////////////////////////////////////////////

bool UResSmrCtl::startPoseStreaming(int interval)
{
  bool result = false;
  //
  if (varRunning->getBool() and smrif != NULL)
  {
    if (not smrif->isStreaming())
      // this can be done once only
      result = smrif->startOdoStream(interval);
  }
  return result;

}

//////////////////////////////////////////////////////////////

void UResSmrCtl::getSMRCLDrive2cmd(char * cmd, const int cmdCnt,
                                   UPoseV startPose, UPoseV endPose,
                                   double breakAt, double orderDist,
                                   bool final, UTime * t)
{
  char * p1 = cmd;
  int n = 0;
  UPose dp;
  UPoseV pose, breakPose, orderPose;
  U2Dlined line;
  //
  line.setPH(endPose.x, endPose.y, endPose.h);
  line.getOnLine(startPose.x, startPose.y, &pose.x, &pose.y);
  pose.h = endPose.h;
  pose.vel = startPose.vel; // for log only
  // debug
  // printf("UResSmrCtl::getSMRCLDrive2cmd: current-online %.2fx %.2fy %.1f deg-mat\n",
  //       pose.x, pose.y, pose.h*180.0/M_PI);
  // debug end
  // advance break distance from current pose
  dp.set(breakAt, 0.0, 0.0);
  breakPose = pose + dp;
  // advanve to ordered distance from current pose
  if (final)
  {
    orderPose = endPose;
  }
  else
  {
    dp.set(orderDist, 0.0, 0.0);
    orderPose = pose + dp;
  }
  // first drive to break-event distance
  snprintf(p1, cmdCnt - n, "driveon %.3f %.3f %.3f \"rad\" @v%.2f :($targetdist < 0.0)\n\t\n",
           breakPose.x, breakPose.y, breakPose.h, endPose.getVel());
  n += strlen(p1);
  p1 = &cmd[n];
  // then to the ordered distance
  snprintf(p1, cmdCnt - n, "driveon %.3f %.3f %.3f \"rad\" @v%.2f :($targetdist < 0.0)",
           orderPose.x, orderPose.y, orderPose.h, endPose.getVel());
  if (logprim.isOpen())
  {
    fprintf(logprim.getF(), "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
            t->getSec(), t->getMicrosec(),
                     2,
                     pose.x, pose.y, pose.h, pose.vel,
                     breakPose.x, breakPose.y, breakPose.h, breakPose.vel);
    fprintf(logprim.getF(), "%lu.%06lu %d %.3f %.3f %.5f %.2f %.3f %.3f %.5f %.2f\n",
            t->getSec(), t->getMicrosec(),
                     102,
                     breakPose.x, breakPose.y, breakPose.h, breakPose.vel,
                     orderPose.x, orderPose.y, orderPose.h, orderPose.vel);
  }
}
