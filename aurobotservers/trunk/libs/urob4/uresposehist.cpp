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

#include <ugen4/u2dline.h>
#include <umap4/uposev.h>

#include "uvarcalc.h"
#include "uresposehist.h"

// UResPoseHist::UResPoseHist()
// {
//   posesCnt = 0;
//   newest = -1;
//   resVersion = getResVersion();
//   replay = false;
//   strncpy(replaySubPath, "replay", MAX_SUBPATH_LENGTH);
//   replayFile = NULL;
//   replayLine[0] = '\0';
//   replayLogLine = 0;
//   logPose = NULL;
//   newestSource = -1;
//   replayTime2 = false;
//   setResID("odoPose");
//   createVarSpace( 15, 0, 5, "Holds the most recent robot position and the near pose history");
//   createBaseVar();
// }

/////////////////////////////////////////////////////////

UResPoseHist::~UResPoseHist()
{
  closeLogfile();
}

/////////////////////////////////////////////////////////

void UResPoseHist::clear()
{
  newest = -1;
  posesCnt = 0;
}

/////////////////////////////////////////////////////////

void UResPoseHist::setResID(const char * aliasName)
{
  //
  UResBase::setResID(aliasName, getResVersion());
  logPose.setLogName(aliasName);
/*  if (strcmp(getResID(), getUtmPoseID()) == 0)
  { // estimate velocity and heading from movement for UTM-poses
    varEstVel->setValued(1.0, 0);
    varEstHead->setValued(1.0, 0);
  }*/
  setDescription("Pose history maintainer (UResPoseHist)", false);
}

/////////////////////////////////////////////////////////

void  UResPoseHist::createBaseVar()
{
  //addVar("version", getResVersion() / 100.0, "d", "Resource version");
  varMinDist = addVar("minDist", 0.003, "d", "Adding history if distance is more than this");
  varMinTurn = addVar("minTurn", 0.3 * M_PI / 180.0, "d", "Adding history if turned more than this");
  varMinTime = addVar("minTime", 10.0, "d", "Adding history if time passed is more than this");
  varTrip = addVar("trip", 0.0, "d", "Total distance (should not be reset)");
  varTripA = addVar("tripA", 0.0, "d", "Distance since reset - mission distance)");
  varTripB = addVar("tripB", 0.0, "d", "Distance since reset - any trip");
  varPose  = addVar("pose", "0.0 0.0 0.0", "pose", "Current robot pose");
//  varPoseX = addVar("poseX", "0.0 0.0 0.0", "pose", "Current robot pose (deprecated, use pose)");
//  varPoseY = addVar("poseY", 0.0, "d", "(deprecated, use pose)");
//  varPoseH = addVar("poseH", 0.0, "d", "(deprecated, use pose)");
  varVel = addVar("vel", 0.0, "d", "Current velocity [m/s]");
  varQ = addVar("poseQ", -1.0, "d", "Current pose quality (-1 is no data, GPS: 1=no, 2=float, 3=fix)");
  varEstVel = addVar("estimateVel", 0.0, "d", "Estimate velocity from position updates");
  varEstHead = addVar("estimateHeading", 0.0, "d", "Estimate heading from position updates (GPS)");
  varTime = addVar("time", 0.0, "t", "current time (in sec since 1 jan 1970)");
  varTripTime = addVar("tripTime", 0.0, "d", "Time since restart [sec] (should not be reset)");
  varTripTimeA = addVar("tripTimeA", 0.0, "d", "Time since reset - mission time)");
  varTripTimeB = addVar("tripTimeB", 0.0, "d", "Time since reset - any sub time)");
  varPoseh5m = addVar("poseh5m", "0.0 0.0 0.0", "pose", "Average heading of the most recent 5 (but 1) meter");
  varPoseh5mUse = addVar("poseh5mUse", 0.0, "d", "Maintain the pose5m value (1=yes, 0=no)");
  //varDistLine20m = addVar("distLine20m", 0.0, "d", "Distance away from a fittet line based on the last 20 (but 5) meter");
  varCalcPose = addVar("calcPose", "0.0 0.0 0.0", "pose", "Calculated pose by one of the pose returning functions");
  varOdoPoseOrigin = addVar("odoPoseOrigin", "0.0 0.0 0.0", "pose", "position of the origin of the odometry coordinate system");
//  varCalcY = addVar("calcY", 0.0, "d", "");
//  varCalcH = addVar("calcH", 0.0, "d", "");
  addMethod("poseAtTime", "d",
                "Get the robot pose at this time (result in calcX, -Y, -H)"
                " or as structure: 3d, pose, posetime, posev");
  addMethod("poseAtDist", "d",
                "Get the robot pose this distance back (result in calcX, -Y, -H)"
                    " or as structure: 3d, pose, posetime, posev");
  addMethod("poseFitAt", "dd",
                "Get average pose using history pose up to d [meters] (newest) from d"
                    " [meters] from current position, may be returned as structure (3d or pose)");
  addMethod("histHeading", "dd",
                "Get average heading using history pose up to d (newest)"
                    " [meters] from d [meter] from current position.");
}

//////////////////////////////////////////////////////////////////

bool UResPoseHist::setResource(UResBase * resource, bool remove)
{
  bool result = true;

  if (resource->isA(UCmdExe::getResClassID()))
  { // ressource may change
    if (remove)
      setCmdExe(NULL);
    else if (not gotCmdExe())
      setCmdExe((UCmdExe *)resource);
    else
      result = false;
  }
  else if (resource->isA(getOdoPoseID()) and not isA(getOdoPoseID()))
  { // I am not an odo-pose, so get odoPose reference
    if (remove)
      odoPoseHist = NULL;
    else if (odoPoseHist != (UResPoseHist *)resource)
      odoPoseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////////

void UResPoseHist::addPoseHist(UPoseTVQ poseAtTime)
{
  double d;
  UTime t;

  lock();
    if (posesCnt > 2)
      d = getPose(0).getDistance(poseAtTime);
    else
      d = 0.0;
    poseNew = poseAtTime;
    newest = (newest + 1) % MAX_HIST_POSES;
    if (newest >= posesCnt)
      posesCnt++;
    poses[newest] = poseAtTime;
  unlock();
  updateVarPool(d);
  t.now();
  // log data too
  if (logPose.isOpen())
    fprintf(logPose.getF(), "%lu.%06lu %.3f %.3f %.5f %.3f %g %lu.%06lu\n",
            poseAtTime.t.getSec(), poseAtTime.t.getMicrosec(),
            poseAtTime.x, poseAtTime.y, poseAtTime.h, poseAtTime.vel, poseAtTime.q,
           t.getSec(), t.getMicrosec());
  // flag for potential push events
  setUpdated("");
}

/////////////////////////////////////////////////////////

void UResPoseHist::addPoseHist(UPose pose, UTime atTime, double v, double qual)
{
  UPoseTVQ pt(pose, atTime, v, qual);
  addPoseHist(pt);
}

/////////////////////////////////////////////////////////

bool UResPoseHist::addIfNeeded(UPoseTVQ pose, int source)
{
  bool result = false;
  UPoseTVQ pt;
  double d, dh, dt;
  bool estHead;
  bool estVel;
  double ev, eh; // estimated velocity and heading
  const double kh = 0.25; // constant "kalman" gain for heading
  const double kv = 0.5;  // constant "kalman" gain for velocity
  UPoseTime odoPt, tmp;
  //
  if ((source == SOURCE_REPLAY or not replay) and // replay uses source -3
      (pose.t.getSec() > 0 or fabs(pose.x) > 0.01))
  { // should heading and/or velocity be estimated
    estHead = varEstHead->getValueBool(0);
    estVel = varEstVel->getValueBool(0);
    //
    if (posesCnt < 3)
    { // atta always first three supplied poses
      addPoseHist(pose);
      result = true;
    }
    else
    { // add only if needed
      pt = getPose(0);
      d = pt.getDistance(pose);
      dh = pt.getHeadingDiff(pose);
      dt = pose.t - pt.t;
      if (estVel and dt > 1e-4)
      {
        ev = pt.vel + kv * (d / dt - pt.vel);
        pose.vel = ev;
      }
      if (estHead)
      {
        dh = atan2(pose.y - pt.y, pose.x - pt.x);
        if (posesCnt > 3)
        { // get heading difference (raw)
          dh = limitToPi(dh - pt.h);
          // smooth a bit
          eh = limitToPi(pt.h + kh * dh);
        }
        else
          // no previous heading, so use raw
          eh = dh;
        pose.h = eh;
      }
      if ((d > getMinDist()) or
          (absf(dh) > getMinTheta()) or
          (dt > getMinTime()))
      { // save in history buffer
        addPoseHist(pose);
        result = true;
      }
      else
      { // add always as newest
        poseNew = pose;
        updateVarPool(0.0);
      }
    }
    // update also the position of the odometry origin in this
    // coordinate syustem (if this is not the odometry coordinate system itself)
    if (odoPoseHist != NULL)
    { // update odometry origin
      odoPt = odoPoseHist->getPoseAtTime(pose.t);
      tmp.clear();
      // get odo-origin relative to current pose
      tmp = odoPt.getMapToPosePose(&tmp);
      // get map pose of odo-origin
      odoPoseOrigin = pose.getPoseToMapPose(tmp);
      odoPoseOrigin.t = pose.t;
      varOdoPoseOrigin->setPose(&odoPoseOrigin);
    }
  }
  if (result)
    newestSource = source;
  return result;
}

//////////////////////////////////////////////////////////

void UResPoseHist::updateVarPool(double tripDist)
{
  double dt, ts;
  UPose closePose;
  // update varPool items
  varPose->setPose(&poseNew);
  // deprecated - should be deleted
/*  varPoseX->setPose(&poseNew);
  varPoseY->setValued(poseNew.y, 0);
  varPoseH->setValued(poseNew.h, 0);*/
  // deprecated end
  varVel->setValued(poseNew.vel, 0, false);
  varQ->setValued(poseNew.q, 0, false);
  ts = poseNew.t.getDecSec();
  dt = ts - varTime->getValued(0);
  varTime->setValued(ts, 0, false);
  if (dt < 36000.0)
  { // time difference is valid
    // add time difference to trip time
    varTripTime->add(dt, 0);
    varTripTimeA->add(dt, 0);
    varTripTimeB->add(dt, 0);
  }
  if (tripDist != 0.0)
  { // robot has mooved
    varTrip->add(tripDist, 0);
    varTripA->add(tripDist, 0);
    varTripB->add(tripDist, 0);
  }
  if (varPoseh5mUse->getBool())
  {
    getPoseFitAtDistance(1.0, 5.0,
                         &poseNew, &closePose);
    //h = getHistHeading(1.0, 5.0, &poseNew, NULL);
    varPoseh5m->setPose(&closePose);
  }
}

//////////////////////////////////////////////////////////

UPoseTVQ UResPoseHist::getPoseAtTime(UTime atTime)
{
  UTime t;
  UPoseTVQ pt1(0.0, 0.0, 0.0, t, 0.0, 0.0); // oldest
  UPoseTVQ pt2(0.0, 0.0, 0.0, t, 0.0, 0.0); // newest
  UPoseTVQ result, poo;
  bool isOK;
  bool tooNew = true;
  //
  // debug
  //double d;
  //t = atTime;
  //d = t - poseNew.t;
  // debug end
  // get before and after pose
  isOK = getPoseNearTime(atTime, &pt1, &pt2);
  if (isOK)
  { // make interpolation to get pose at time
    tooNew = ((atTime - pt2.t) > 0.0);
    if (tooNew)
      // ahead of time - use newest avaiable time
      result = pt2;
    else
    { // try interpolation
      result = pt1.getPoseAtTime(pt2, atTime);
      result.t = atTime;
    }
  }
  else
    // not in history buffer (any more)
    result = pt1;
  if ((not isOK or tooNew) and odoPoseHist != NULL)
  { // calculate from
    isOK = odoPoseHist->getPoseNearTime(atTime, &pt1, &pt2);
    if (isOK)
    { // there is a pose on the odopose store, so this and the
      // last known origin of the odometry coordinate system
      poo = pt1.getPoseAtTime(pt2, atTime);
      result = odoPoseOrigin.getPoseToMapPose(poo);
      result.t = poo.t;
      result.vel = pt2.vel;
      result.q = pt2.q;
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////

bool UResPoseHist::getPoseNearTime(UTime atTime,
                                UPoseTVQ * justBefore,
                                UPoseTVQ * justAfter)
{
  UPoseTVQ po1, *pt1; // oldest
  UPoseTVQ po2, *pt2 = NULL; // newest
  int i;
  bool result = false;
  //bool found = false;
  //
  lock();
  po1 = poseNew;
  po2 = poseNew;
  if (posesCnt == 0)
  {
    result = true;
    po1.t = atTime;
  }
  else
  {
    po1 = poses[newest];
    if (atTime >= po1.t)
    {  // newer than newest in history
      po2 = poseNew;
      if (atTime >= po2.t)
        // also newer than poseNew, so best guess is poseNew
        po1 = poseNew;
      else
        result = true;
    }
    else
    { // look back in history
      pt1 = &poses[newest];
      for (i = 0; i < posesCnt; i++)
      {
        pt2 = pt1;
        pt1--;
        if (pt1 < poses)
          pt1 = &poses[posesCnt - 1];
        result =((pt1->t - atTime) < 0.0);
        if (result)
        { // value found
          po1 = *pt1;
          po2 = *pt2;
          break;
        }
      }
      if (not result)
      { // time is too old
        // return oldest available time
        po1 = *pt2;
      }
    }
  }
  // value found
  if (justBefore != NULL)
    *justBefore = po1;
  if (justAfter != NULL)
    *justAfter = po2;
  unlock();
  return result and posesCnt > 0;
}

/////////////////////////////////////////////////

UPoseTime UResPoseHist::getOldest()
{
  UPoseTime result;
  int idx;
  //
  lock();
  if (posesCnt < MAX_HIST_POSES)
    result = poses[0];
  else
  {
    idx = newest + 1 % MAX_HIST_POSES;
    result = poses[idx];
  }
  unlock();
  //
  return result;
}

/////////////////////////////////////////////////

// UPoseTime UResPoseHist::getNewest()
// {
//   UPoseTime result;
//   //
//   lock();
//   if (posesCnt > 0)
//     result = poses[newest];
//   else
//     result.t.setTime(0, 0);
//   unlock();
//   //
//   return result;
// }

/////////////////////////////////////////////////

UTime UResPoseHist::getOldestTime()
{
  UTime result;
  UPoseTime pt;
  if (posesCnt == 0)
    result.Now();
  else
  {
    pt = getOldest();
    result = pt.t;
  }
  return result;
}

/////////////////////////////////////////////////

UTime UResPoseHist::getTimeAtDistance(double away)
{
  UTime result;
  UPoseTVQ pDist;
  UPose ref;
  bool isOK;
  //
  lock();
  isOK = (posesCnt > 0);
  if (isOK)
    ref = poses[newest];
  unlock();
  //
  if (isOK)
  {
    isOK = getPoseNearDistance(away, &ref, NULL, &pDist);
    if (isOK)
      result = pDist.t;
  }
  if (not isOK)
    result.setTime(0, 0);
  return result;
}

/////////////////////////////////////////////////

bool UResPoseHist::getPoseNearDistance(const double away,
                                    const UPose * ref,
                                    UPoseTVQ * closer,
                                    UPoseTVQ * moreDistant)
{
  UPoseTVQ *pt1, *pt2;
  int i;
  bool result = false;
  UPose p;
  double d;
  //
  if (posesCnt > 0)
  {
    lock();
    pt1 = &poses[newest];
    for (i = 0; i < posesCnt; i++)
    {
      pt2 = pt1;
      pt1--;
      if (pt1 < poses)
        pt1 = &poses[posesCnt - 1];
      d = hypot(ref->x - pt1->x, ref->y - pt1->y);
      result = (d > away);
      if (result)
        // value found
        break;
    }
    if (result)
    { // value found
      if (closer != NULL)
        *closer = *pt1;
      if (moreDistant != NULL)
        *moreDistant = *pt2;
    }
    unlock();
  }
  return result;
}

/////////////////////////////////////////////////

double UResPoseHist::getPoseFitAtDistance(double closeBy, double farAway,
                                       const UPose * ref, UPose * closeFitPose)
{
  double result = -1.0; // returns fit variance
  UPoseTVQ *pt1, pNear, pAway;
  int i;
  bool start = false;
  U2Dlined fitLine;
  const int MAX_LINE_PTS = 5000;
  double fx[MAX_LINE_PTS];
  double fy[MAX_LINE_PTS];
  int fxyCnt = 0;
  double xn, yn, xa, ya;
  double d;
  int startI = -1;
  //
  if (posesCnt > 2)
  {
    lock();
    pt1 = &poses[newest];
    for (i = 0; i < posesCnt; i++)
    {
      if (pt1 < poses)
        pt1 = &poses[posesCnt - 1];
      d = hypot(ref->x - pt1->x, ref->y - pt1->y);
      if (not start)
      {
        start = (d > closeBy);
        if (start)
        { // save close by position
          pNear = *pt1;
          startI = i;
        }
      }
      if (start)
      {
        if (d > farAway)
          break;
        fx[fxyCnt] = pt1->x;
        fy[fxyCnt] = pt1->y;
        fxyCnt++;
        pAway = *pt1;
        if (fxyCnt >= MAX_LINE_PTS)
        {
          printf("UResPoseHist::getPoseFitAtDistance: overflow warning "
               "(start idx %d end idx %d of (%d)\n",
               startI, i, posesCnt);
          break;
        }
      }
      pt1--;
    }
    unlock();
    if (fxyCnt > 1)
    { // calculate fit line
      fitLine.fit(fx, fy, fxyCnt, &result);
      // get where closest point is on line
      fitLine.getOnLine(pNear.x, pNear.y, &xn, &yn);
      // get where further-away point is on line
      fitLine.getOnLine(pAway.x, pAway.y, &xa, &ya);
      // set result pose.
      closeFitPose->set(xn, yn, atan2(yn - ya, xn - xa));
    }
  }
  return result;
}

///////////////////////////////////////////////////////////////

double UResPoseHist::getHistHeading(double closeBy, double farAway,
                                       const UPoseTime * ref, double * histAge)
{
  double result; // average heading
  UPoseTVQ *pt1, pNear, pAway;
  int i;
  bool start = false;
  double d;
  //
  if (posesCnt > 2)
  {
    lock();
    pt1 = &poses[newest];
    pNear = *pt1;
    for (i = 0; i < posesCnt; i++)
    { // catch near position and stop at far position
      if (pt1 < poses)
        pt1 = &poses[posesCnt - 1];
      d = hypot(ref->x - pt1->x, ref->y - pt1->y);
      if (not start)
      {
        start = (d > closeBy);
        if (start)
          pNear = *pt1;
      }
      if (start)
      {
        if (d >= farAway)
          break;
      }
      pt1--;
    }
    pAway = *pt1;
    unlock();
    result = atan2(pNear.y - pAway.y, pNear.x - pAway.x);
    if (histAge != NULL)
    {
      if (i < posesCnt)
        // time is OK
        *histAge = pAway.t - ref->t;
      else
        // not enough valid data to determine age
        *histAge = 3600.0;
    }
  }
  else
    result = ref->h;
  return result;
}

/////////////////////////////////////////////////////////

UPoseTVQ UResPoseHist::getPose(int index)
{
  int i;
  UPoseTVQ result;
  //
  i = newest - index;
  if (i < 0)
    i += MAX_HIST_POSES;
  result = poses[i];
  //
  return result;
}

/////////////////////////////////////////////////////////

void UResPoseHist::saveToLog(FILE * logf)
{
  UPoseTime pt; //, ptUTM;
  int i;
  //
  if (logf != NULL)
  { // first newest pose
    pt = poseNew;
    fprintf(logf, "%lu.%06lu %.3f %.3f %.4f\n",
            pt.t.getSec(), pt.t.getMicrosec(),
            pt.x, pt.y, pt.h);
    for (i = 0; i < posesCnt; i++)
    { // all the rest
      pt = getPose(i);
      fprintf(logf, "%lu.%06lu %.3f %.3f %.4f\n",
              pt.t.getSec(), pt.t.getMicrosec(),
              pt.x, pt.y, pt.h);
    }
  }
}

/////////////////////////////////////////////////////////

const char * UResPoseHist::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s holds %d poses\n",
           preString, posesCnt);
  return buff;
}

/////////////////////////////////////////////////////////

bool UResPoseHist::setReplay(bool value)
{
  bool result = true;
  const int MFL = 500;
  char fn[MFL];
  UTime t;
  char * p1;
  //
  if (replayFile != NULL)
  { // stop replay and clear pose history
    fclose(replayFile);
    replayFile = NULL;
  }
  if (value)
  { // start replay
    replayFile = fopen(getReplayFileName(fn, MFL), "r");
    result = (replayFile != NULL);
    replayLogLine = 0;
    if (result)
    { // read first line
      while (not t.valid and not feof(replayFile))
      { // read past lines not starting with a timestamp
        p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
        // normal - timestamp is first entry
        t.setTimeTod(replayLine);
        if (replayTime2 and t.valid)
        { // last entry is (assumed to be) time
          p1 = strrchr(replayLine, ' ');
          if (p1 != NULL)
            t.setTimeTod(++p1);
        }
        replayLogLine++;
      }
      result = t.valid;
      if (result)
        replayTimeNext = t;
      if (feof(replayFile))
      { // no valid data - close file
        fclose(replayFile);
        replayFile = NULL;
        //if (verbose)
        fprintf(stderr, "No valid replay data in '%s'\n", fn);
      }
    }
    else
      //if (verbose)
      fprintf(stderr, "Replay file not found '%s'\n", fn);
  }
  // clear current pose history buffer in all cases
  clear();
  replay = value;
  return result;
}

/////////////////////////////////////////////////////////

// void UResPoseHist::setReplaySubdir(const char * subdir)
// {
//   strncpy(replaySubPath, subdir, MAX_PATH_LENGTH);
// }

/////////////////////////////////////////////////////////

char * UResPoseHist::getReplayFileName(char * fn, const int fnCnt)
{
  snprintf(fn, fnCnt, "%s/%s.log", replayPath, logPose.getLogName());
  return fn;
}

/////////////////////////////////////////////////////////

bool UResPoseHist::replayStep(int steps)
{
  bool result = true;
  int i;
  //
  for (i = 0; i < steps; i++)
  {
    result = replayStep();
    if (not result)
      break;
  }
  if (result)
  { // alert other resources of the advance in replay time
    replayAdvanceTime(replayTimeNow);
  }
  //
  return result;
}

/////////////////////////////////////////////////////////

// bool UResPoseHist::replayTime(UTime untilTime)
// {
//   bool result = true;
//   //
//   while (untilTime > replayTimeNext)
//   {
//     result = replayStep();
//     if (not result)
//       break;
//   }
//   return result;
// }

/////////////////////////////////////////////////////////

bool UResPoseHist::replayStep()
{
  bool result = true;
  UTime t;
  int n;
  unsigned int u1,u2;
  UPoseTVQ pose;
  char * p1;
  const int UPDATE_SOURCE_ID = -3;
  //
  if ((replayFile != NULL) and replay)
  {
    t = replayTimeNext; // t.setTimeTod(replayLine);
    pose.vel = 0.0;
    pose.q = -1.0;
    if (t.valid)
    { // this is a legal line - read pose
      // format - speed and the rest is optional
      // time of day          x         y         th      speed   (dist)   (flags)
      // 1148030115.707652 3.120415 -0.147563 -0.058803 0.789768 3.124594 835.000000
      // short format (used in HAKO)
      //  time (MRC)     x     y     th    [vel ] [ Q ] [  time (logger)  ]
      // 1190628060.266748 0.000 0.000 0.00000 [1.22] [2.0] [1190623427.606278]
      n = sscanf(replayLine, "%u.%u %lg %lg %lg %lg %lg",
                &u1, &u2,
                &pose.x, &pose.y, &pose.h,
                &pose.vel, &pose.q);
      result = (n >= 5);
      if (result)
      { // all values read - speed is optional
        pose.t = t;
        if (pose.vel > 1e7)
          // may be a timestamp
          pose.vel = 0.0;
        if (pose.q > 1e7)
          // is a second timestamp
          pose.q = -1.0;
        addIfNeeded(pose, UPDATE_SOURCE_ID);
        replayTimeNow = t;
      }
    }
    do
    {
      p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
      if (replayTime2)
      { // last data set may be second timestamp
        p1 = strrchr(replayLine, ' ');
        if (p1 != NULL)
          t.setTimeTod(++p1);
      }
      else
        // normal - timestamp is first entry
        t.setTimeTod(replayLine);
      replayLogLine++;
    } while (not t.valid and not feof(replayFile));
    if (t.valid)
      replayTimeNext = t;
    if (feof(replayFile))
    {
      fclose(replayFile);
      replayFile = NULL;
      //if (verbose)
      fprintf(stderr, "No more valid replay data (after %d lines)\n", replayLogLine);
    }
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////////////////

bool UResPoseHist::methodCall(const char * name, const char * paramOrder,
                               char ** strings, const double * pars,
                               double * value,
                               UDataBase ** returnStruct,
                               int * returnStructCnt)
{
  bool result = true;
  UPoseTVQ pose1, pose2, pose;
  UTime t;
  bool found;
  double v = 0.0, d, d1, d2, dt;
  // evaluate standard functions
/*  vp->addMethod(this, "poseAtTime", "d");
  vp->addMethod(this, "poseAtDist", "d");
  vp->addMethod(this, "poseFitAt", "dd");*/
  if ((strcasecmp(name, "poseAtTime") == 0) and (strcmp(paramOrder, "d") == 0))
  {
    t.setTime( pars[0]);
    found = getPoseNearTime( t, &pose1, &pose2);
    if (found)
    {
      d = (t - pose2.t) / (pose1.t - pose2.t);
      pose.x = d * (pose1.x - pose2.x) + pose2.x;
      pose.y = d * (pose1.y - pose2.y) + pose2.y;
      pose.h = limitToPi(d * limitToPi(pose1.h - pose2.h) + pose2.h);
      dt = pose1.t - pose2.t;
      pose.t = pose2.t + d * dt;
      pose.vel = d * (pose1.vel - pose2.vel) + pose2.vel;
      pose.q   = d * (pose1.q - pose2.q) + pose2.q;
    }
    else
      pose = pose1;
    setPoseResult(found, pose, value, returnStruct, returnStructCnt);
  }
  else if ((strcasecmp(name, "poseAtDist") == 0) and (strcmp(paramOrder, "d") == 0))
  {
    t.setTime( pars[0]);
    found = getPoseNearDistance( pars[0], &poseNew, &pose1, &pose2);
    if (found)
    {
      pose = getNewest(NULL);
      d1 = pose.getDistance(pose1);
      d2 = pose.getDistance(pose2);
      d = (pars[0] - d2) / (d1 - d2);
      pose.x = d * (pose1.x - pose2.x) + pose2.x;
      pose.y = d * (pose1.y - pose2.y) + pose2.y;
      pose.h = limitToPi(d * limitToPi(pose1.h - pose2.h) + pose2.h);
      dt = pose1.t - pose2.t;
      pose.t = pose2.t + d * dt;
      pose.vel = d * (pose1.vel - pose2.vel) + pose2.vel;
      pose.q   = d * (pose1.q - pose2.q) + pose2.q;
    }
    else
      pose = pose1;
    setPoseResult(found, pose, value, returnStruct, returnStructCnt);
  }
  else if ((strcasecmp(name, "poseFitAt") == 0) and (strcmp(paramOrder, "dd") == 0))
  {
    t.setTime( pars[0]);
    found = getPoseFitAtDistance( pars[0], pars[1], &poseNew, &pose1);
    setPoseResult(found, pose1, value, returnStruct, returnStructCnt);
  }
  else if ((strcasecmp(name, "histHeading") == 0) and (strcmp(paramOrder, "dd") == 0))
  {
    t.setTime( pars[0]);
    v = getHistHeading( pars[0], pars[1], &poseNew, NULL);
    if (value != NULL)
      *value = v;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////////////////

void UResPoseHist::setPoseResult(bool found, UPoseTVQ pose1,
                   double * value,
                   UDataBase ** returnStruct, int * returnStructCnt)
{
  UPosition pos;
  if (value != NULL)
  {
    *value = found;
    varCalcPose->setPose(&pose1);
  }
  if (returnStruct != NULL and (returnStructCnt != NULL))
  {
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("posetime"))
      {
        *(UPoseTime *)returnStruct[0] = pose1.getPoseTime();
        *returnStructCnt = 1;
      }
      if (returnStruct[0]->isA("posetvq"))
      {
        *(UPoseTime *)returnStruct[0] = pose1;
        *returnStructCnt = 1;
      }
      else if (returnStruct[0]->isA("pose"))
      {
        *(UPose *)returnStruct[0] = pose1.getPose();
        *returnStructCnt = 1;
      }
      else if (returnStruct[0]->isA("posev"))
      {
        *(UPoseV *)returnStruct[0] = pose1.getPoseV();
        *returnStructCnt = 1;
      }
      else if (returnStruct[0]->isA("3d"))
      {
        pos.set(pose1.x, pose1.y, 0.0);
        *(UPosition *)returnStruct[0] = pos;
        *returnStructCnt = 1;
      }
      else
        *returnStructCnt = 0;
    }
    else
      *returnStructCnt = 0;
  }
}

////////////////////////////////////////////////////////////////

// void UResPoseHist::setVarCalcPose(UPose pose)
// {
//   varCalcPose->setPose(&pose);
// //  varCalcY->setValued(pose.y, 0);
// //  varCalcH->setValued(pose.h, 0);
// }

//////////////////////////////////////////////////////

bool UResPoseHist::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  if (not gotCmdExe())
  {
    if (p1 != NULL)
    {
      strncpy(p1, UCmdExe::getResClassID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }
  result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

////////////////////////////

bool UResPoseHist::openLogfile()
{
  //logPose.openLog();
  return logPose.openLog();
}

////////////////////////////

void UResPoseHist::closeLogfile()
{
  {
    lock();
    logPose.closeLog();
    unlock();
  }
}

///////////////////////////////

bool UResPoseHist::toKml(char * aKmlName, const int MRL)
{
  ULogFile srcName;
  FILE * src;
  ULogFile dest;
  bool fullPath, isOK;
  char * p2;
  int n;
  //
  if (strcmp(aKmlName, logPose.getLogName()) == 0)
  { // the current logfile
    logPose.doFlush();
  }
  fullPath = (aKmlName[0] == '/');
  if (fullPath)
  {
    src = fopen(aKmlName, "r");
    p2 = &aKmlName[strlen(aKmlName)];
  }
  else
  {
    if (isReplayFileOpen())
    { // use replay file as source
      getReplayFileName(aKmlName, MRL);
      src = fopen(aKmlName, "r");
    }
    else
    { // assume aKmlname is the name without extension of utm logfile
      srcName.setLogName(aKmlName);
      src = fopen(srcName.getLogFileName(), "r");
      strncpy(aKmlName, srcName.getLogFileName(), MRL);
    }
  }
  // set p2 to end of name for conversion result
  n = strlen(aKmlName);
  p2 = &aKmlName[n];
  // ready to convert
  if (isReplayFileOpen())
    dest.setLogName("utmPose", "kml");
  else
    // replay source
    dest.setLogName("utmPoseReplay", "kml");
  isOK = (src != NULL);
  if (isOK)
  { // open destination
    dest.openLog();
    isOK = (dest.isOpen());
    if (isOK)
      // do the conversion
      processUtmFile(src, dest.getF());
    dest.closeLog();
    fclose(src);
  }
  // make result message
  if (isOK)
    snprintf(p2, MRL - n, " to %s done\n", dest.getLogFileName());
  else
    snprintf(p2, MRL - n, " to %s failed\n", dest.getLogFileName());
  return isOK;
}

/////////////////////////////////////////////////////////

bool UResPoseHist::processUtmFile(FILE * fs, FILE * fd, bool doLine)
{
  const int zone = 32;
  const char * header = "<Document><name>utmPose.kml</name>\n"
      "<Style id=\"sn_donut\">" "<IconStyle><color>ff1effdd</color><scale>0.4</scale>\n"
      "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/donut.png</href></Icon></IconStyle></Style>\n"
      "<Style id=\"sh_donut\"><IconStyle><color>ff1effdd</color><scale>0.52</scale>\n"
      "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/donut.png</href></Icon></IconStyle></Style>\n"
      "<Style id=\"utmLine1\"><LineStyle>\n"
      "<color>7fff0000</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<Style id=\"utmLine2\"><LineStyle>\n"
      "<color>7f9f009f</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<Style id=\"utmLine3\"><LineStyle>\n"
      "<color>7f0000ff</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<Style id=\"utmLine4\"><LineStyle>\n"
      "<color>7f7f7f00</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<StyleMap id=\"msn_donut\"><Pair><key>normal</key><styleUrl>#sn_donut</styleUrl></Pair>"
      "<Pair><key>highlight</key><styleUrl>#sh_donut</styleUrl></Pair></StyleMap>\n";
  const char * footer="</Document>";
  const int MSL = 200;
  char s[MSL];
  int m = 0, l = 0;
  double t, e, n, lat = 0.0, lon = 0.0, lat2, lon2;
  float d1, d2, q, q2 = 0;
//  bool isOK;
  const char * lin0Hdr = "<Placemark><name>RTK-GPS old</name>\n"
      "<styleUrl>#utmLine1</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin1Hdr = "<Placemark><name>RTK-GPS autonomus</name>\n"
      "<styleUrl>#utmLine1</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin2Hdr = "<Placemark><name>RTK-GPS float</name>\n"
      "<styleUrl>#utmLine2</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin3Hdr = "<Placemark><name>RTK-GPS fix</name>\n"
      "<styleUrl>#utmLine3</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin4Hdr = "<Placemark><name>RTK-GPS dgps</name>\n"
      "<styleUrl>#utmLine4</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * linEnd = "</coordinates></LineString></Placemark>\n";
  char * p1;
  //
  fprintf(fd, "%s\n", header);
  if (doLine)
    fprintf(fd, "<Placemark><name>RTK-GPS HAKO Pometet</name>\n"
        "<styleUrl>#utmLine1</styleUrl>\n"
            "<LineString><coordinates>\n");
  while (not feof(fs))
  {
    p1 = fgets(s, MSL, fs);
    if (p1 != NULL)
      m = sscanf(s, "%lf %lf %lf %f %f %f", &t, &e, &n, &d1, &d2, &q);
    if (m == 6)
    {
      lat2 = lat;
      lon2 = lon;
      //isOK = utm2latlon(e, n, zone, &lat, &lon);
      UTMtoLL(23, n, e, zone, &lat,  &lon);

      if (doLine and l > 0)
      {
        if ( q != q2)
        { // change line stype
          if (q < q2)
            // improved quality - add new point to lower quality line
            fprintf(fd, "%.8f,%.8f,0\n", lon, lat);
          //
          fprintf(fd, "%s", linEnd);
          if (q <= 0.5)
            // old position
            fprintf(fd, "%s", lin0Hdr);
          else if (q <= 1.01)
            // autonomous
            fprintf(fd, "%s", lin1Hdr);
          else if (q <= 2.01)
            // floating
            fprintf(fd, "%s", lin2Hdr);
          else if (q <= 3.01)
            // FIX
            fprintf(fd, "%s", lin3Hdr);
          else
            // DGPS
            fprintf(fd, "%s", lin4Hdr);
          // repeat first point
          if (q > q2)
            // worse quality - add last good point to worse line
            fprintf(fd, "%.8f,%.8f,0\n", lon2, lat2);
        }
      }
      q2 = q;
      //bool utm2latlon(double easting, double northing, int zone,
      //                double * latitude, double * longitude)
      //isOK = utm2latlon(e, n, zone, &lat, &lon);
      if (doLine)
      {
        fprintf(fd, "%.8f,%.8f,0\n", lon, lat);
      }
      else
      {
        fprintf(fd, "<Placemark><styleUrl>#msn_donut</styleUrl><Point>"
            "<coordinates>%.8f,%.8f,0.0</coordinates>"
                "</Point></Placemark>\n", lon, lat);
      }
    }
    l++;
  }
  if (doLine)
    fprintf(fd, "%s", linEnd);
  fprintf(fd, "%s\n", footer);
  return m == 6;
}

