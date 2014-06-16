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

#include "uposehist.h"

UPoseHistNotUsed::UPoseHistNotUsed()
{
  posesCnt = 0;
  newest = -1;
  setResID(getResID(), getResVersion());
  resVersion = getResVersion();
  minDistChange = 0.03;
  minHeadingChange = 1.0 * M_PI / 180.0;
  minTimeChange = 10.0; // seconds
  replay = false;
  strncpy(replaySubPath, "replay", MAX_SUBPATH_LENGTH);
  replayFile = NULL;
  replayLine[0] = '\0';
  replayLogLine = 0;
}

/////////////////////////////////////////////////////////

UPoseHistNotUsed::~UPoseHistNotUsed()
{
}

/////////////////////////////////////////////////////////

void UPoseHistNotUsed::clear()
{
  newest = -1;
  posesCnt = 0;
}

/////////////////////////////////////////////////////////

void UPoseHistNotUsed::addPoseHist(UPoseTime poseAtTime)
{
  lock();
  poseNew = poseAtTime;
  newest = (newest + 1) % MAX_HIST_POSES;
  if (newest >= posesCnt)
    posesCnt++;
  poses[newest] = poseAtTime;
  unlock();
}

/////////////////////////////////////////////////////////

void UPoseHistNotUsed::addPoseHist(UPose pose, UTime atTime)
{
  UPoseTime pt(pose, atTime);
  addPoseHist(pt);
}

/////////////////////////////////////////////////////////

bool UPoseHistNotUsed::addIfNeeded(UPoseTime pose, double velocity)
{
  bool result = false;
  UPoseTime pt;
  double d, dh, dt;
/*  const double MIN_DISTANCE_CHANGE = 0.03;
  const double MIN_HEADING_CHANGE = 1.0 * M_PI / 180.0;
  const double MIN_TIME_CHANGE = 10.0; // seconds*/
  //
  if (pose.t.getSec() > 0)
  {
    // debug
    // printf("UPoseHistNotUsed::addIfNeeded: Adding pose to poseHist\n");
    // debug end
    if (posesCnt < 3)
    {
      addPoseHist(pose);
      result = true;
    }
    else
    { // add if needed
      pt = getPose(0);
      d = pt.getDistance(pose);
      dh = pt.getHeadingDiff(pose);
      dt = pose.t - pt.t;
      if ((d > minDistChange) or
           (absf(dh) > minHeadingChange) or
           (dt > minTimeChange))
      { // save in history buffer
        addPoseHist(pose);
        result = true;
      }
    }
    // add always as newest
    poseNew = pose;
    speed = velocity;
  }
  return result;
}

//////////////////////////////////////////////////////////

UPose UPoseHistNotUsed::getPoseAtTime(UTime atTime)
{
  UTime t;
  UPoseTime pt1(0.0, 0.0, 0.0, t); // oldest
  UPoseTime pt2; // newest
  UPose result;
  bool isOK;
  bool tooNew = true;
  //
  // get before and after pose
  isOK = getPoseNearTime(atTime, &pt1, &pt2);
  if (isOK)
  { // make interpolation to get pose at time
    tooNew = ((atTime - pt2.t) > 0.0);
    if (tooNew)
      // ahead of time - use newest avaiable time
      result = pt2.getPose();
    else
      // try interpolation
      result = pt1.getPoseAtTime(pt2, atTime);
  }
  else
    // not in history buffer (any more)
    result = pt1.getPose();
  //
  return result;
}

/////////////////////////////////////////////////

bool UPoseHistNotUsed::getPoseNearTime(UTime atTime,
                                UPoseTime * justBefore,
                                UPoseTime * justAfter)
{
  UPoseTime *pt1; // oldest
  UPoseTime *pt2 = NULL; // newest
  int i;
  bool result = false;
  //
  lock();
  if (posesCnt == 0)
  {
    pt1 = &poseNew;
    pt2 = &poseNew;
    result = true;
  }
  else
  {
    pt1 = &poses[newest];
    if ((atTime - pt1->t) > 0.0)
    {  // newer than newest in history
      pt2 = &poseNew;
      if ((atTime - pt2->t) > 0.0)
        // also newer than poseNew, so best guess is poseNew
        pt1 = &poseNew;
      result = true;
    }
  }
  if (not result)
  { // look back in history
    for (i = 0; i < posesCnt; i++)
    {
      pt2 = pt1;
      pt1--;
      if (pt1 < poses)
        pt1 = &poses[posesCnt - 1];
      result =((pt1->t - atTime) < 0.0);
      if (result)
        // value found
        break;
    }
  }
  if (not result)
  { // time is too old
    // return oldest available time
    pt1 = pt2;
    // debug
    printf("UPoseHistNotUsed::getPoseNearTime: %lu.%06lu not found "
            "(newest %lu.%06lu oldest %lu.%06lu)\n",
            atTime.getSec(), atTime.getMicrosec(),
            poseNew.t.getSec(), poseNew.t.getMicrosec(),
            pt1->t.getSec(), pt1->t.getMicrosec());
    // debug end
  }
  if (result)
  { // value found
    if (justBefore != NULL)
      *justBefore = *pt1;
    if (justAfter != NULL)
      *justAfter = *pt2;
  }
  unlock();
  return result;
}

/////////////////////////////////////////////////

UPoseTime UPoseHistNotUsed::getOldest()
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

// UPoseTime UPoseHistNotUsed::getNewest()
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

UTime UPoseHistNotUsed::getOldestTime()
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

UTime UPoseHistNotUsed::getTimeAtDistance(double away)
{
  UTime result;
  UPoseTime pDist;
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

bool UPoseHistNotUsed::getPoseNearDistance(const double away,
                                    const UPose * ref,
                                    UPoseTime * closer,
                                    UPoseTime * moreDistant)
{
  UPoseTime *pt1, *pt2;
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

double UPoseHistNotUsed::getPoseFitAtDistance(double closeBy, double farAway,
                                       const UPose * ref, UPoseTime * closeFitPose)
{
  float result = -1.0; // returns fit variance
  UPoseTime *pt1, pNear, pAway;
  int i;
  bool start = false;
  U2Dline fitLine;
  const int MAX_LINE_PTS = 5000;
  float fx[MAX_LINE_PTS];
  float fy[MAX_LINE_PTS];
  int fxyCnt = 0;
  float xn, yn, xa, ya;
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
        if (fxyCnt >= MAX_LINE_PTS)
        {
          printf("UPoseHistNotUsed::getPoseFitAtDistance: overflow warning "
               "(start idx %d end idx %d of (%d)\n",
               startI, i, posesCnt);
          break;
        }
      }
      pt1--;
    }
    pAway = *pt1;
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

double UPoseHistNotUsed::getHistHeading(double closeBy, double farAway,
                                       const UPoseTime * ref, double * histAge)
{
  double result; // average heading
  UPoseTime *pt1, pNear, pAway;
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

UPoseTime UPoseHistNotUsed::getPose(int index)
{
  int i;
  UPoseTime result;
  //
  i = newest - index;
  if (i < 0)
    i += MAX_HIST_POSES;
  result = poses[i];
  //
  return result;
}

/////////////////////////////////////////////////////////

void UPoseHistNotUsed::saveToLog(FILE * logf)
{
  UPoseTime pt, ptUTM;
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

const char * UPoseHistNotUsed::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s holds %d poses\n",
           preString, posesCnt);
  return buff;
}

/////////////////////////////////////////////////////////

bool UPoseHistNotUsed::setReplay(bool value)
{
  bool result = true;
  const int MFL = 500;
  char fn[MFL];
  UTime t;
  const char * p1;
  //
  if (value != replay)
  { // stop replay
    if (replay and (replayFile != NULL))
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
          t.setTimeTod(replayLine);
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
  }
  replay = value;
  return result;
}

/////////////////////////////////////////////////////////

void UPoseHistNotUsed::setReplaySubdir(const char * subdir)
{
  strncpy(replaySubPath, subdir, MAX_PATH_LENGTH);
}

/////////////////////////////////////////////////////////

char * UPoseHistNotUsed::getReplayFileName(char * fn, const int fnCnt)
{
  snprintf(fn, fnCnt, "%s/%s/odo.log", imagePath, replaySubPath);
  return fn;
}

/////////////////////////////////////////////////////////

bool UPoseHistNotUsed::replayStep(int steps)
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
  return result;
}

/////////////////////////////////////////////////////////

bool UPoseHistNotUsed::replayTime(UTime untilTime)
{
  bool result = true;
  //
  while (untilTime > replayTimeNext)
  {
    result = replayStep();
    if (not result)
      break;
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UPoseHistNotUsed::replayStep()
{
  bool result = true;
  UTime t;
  int n;
  unsigned int u1,u2;
  UPoseTime pose;
  double vel;
  const char * p1;
  //
  if ((replayFile != NULL) and replay)
  {
    t.setTimeTod(replayLine);
    if (t.valid)
    { // this is a legal line - read pose
      // format - speed and the rest is optional
      // time of day          x         y         th      speed   (dist)   (flags)
      // 1148030115.707652 3.120415 -0.147563 -0.058803 0.789768 3.124594 835.000000
      vel = 0.0;
      n = sscanf(replayLine, "%u.%u %lg %lg %lg %lg",
                  &u1, &u2,
                  &pose.x, &pose.y, &pose.h,
                  &vel);
      result = (n >= 5);
      if (result)
      { // all values read - speed is optional
        pose.t = t;
        addIfNeeded( pose, vel);
        replayTimeNow = t;
      }
    }
    do
    {
      p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
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

