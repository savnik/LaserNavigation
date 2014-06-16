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

#include <urob4/uresposehist.h>

#include "ureplaydevice.h"

UReplayDevice::UReplayDevice()
{
  replay = false;
  strncpy(devName, "laser_0.log", MAX_DEVICE_NAME_LNG);
  strncpy(name, "replay", MAX_NAME_LNG);
  replayFile = NULL;
  replayLine[0] = '\0';
  replayLogLine = 0;
  maxValidRange = 8.1;
}


UReplayDevice::~UReplayDevice()
{
}

/////////////////////////////////////////////////////////

bool UReplayDevice::setReplay(bool value)
{
  bool result = false;
  const int MFL = 500;
  char fn[MFL];
  UTime t;
  char * p1;
  //
  // stop replay
  if (not value and (replayFile != NULL))
  { // stop replay and clear pose history
    fclose(replayFile);
    replayFile = NULL;
    result = true;
  }
  if (value and (replayFile == NULL))
  { // start replay
    replayFile = fopen(getReplayFileName(fn, MFL), "r");
    result = (replayFile != NULL);
    replayLogLine = 0;
    if (result)
    { // read first line
      while (not t.valid and not feof(replayFile))
      { // read past lines not starting with a timestamp
        p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
        if (p1 != NULL)
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
    result = true;
  }
  replay = value;
  return result;
}

/////////////////////////////////////////////////////////

char * UReplayDevice::getReplayFileName(char * fn, const int fnCnt)
{
  snprintf(fn, fnCnt, "%s/%s", replayPath, devName);
  return fn;
}

/////////////////////////////////////////////////////////

bool UReplayDevice::replayStep(int steps, UResBase * lasPool)
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
  if (i > 0)
    lasPool->replayAdvanceTime(replayTimeNow);
  return result;
}

/////////////////////////////////////////////////////////

bool UReplayDevice::replayStepToScan(unsigned int toSerial, UResBase * lasPool)
{
  bool result = false;
  int i = 0;
  //
  while (scan.getSerial() < toSerial)
  {
    result = replayStep();
    if (not result)
      break;
    i++;
  }
  if (i > 0)
    lasPool->replayAdvanceTime(replayTimeNow);
  return result;
}

/////////////////////////////////////////////////////////

bool UReplayDevice::replayToTime(UTime untilTime)
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

bool UReplayDevice::replayStep()
{
  bool result = true;
  UTime t;
  int i;
  unsigned int u1;
  UPoseTime pose;
  double t1; //, t2, px, py;
  char *p1, *p2, *p3;
  int rngCnt;
  double rngRes; // resolution
  double rngStartAng; // start angle
  double rng;
  int * rngI;
  int * rngFlagI;
  double maxRng = 4.096;
  //
  if (replayFile == NULL)
    // start replay if not started already
    setReplay(true);
  //
  if ((replayFile != NULL) and replay)
  {
    t.setTimeTod(replayLine);
    if (t.valid)
    { // this is a legal line - read the rest
      // format - 1 (wpf.log)
      // % laserscan save: time tilt odox odoy odoh planX planY ray1 ... ray181
      // 1148031884.152597 1148031884.264499 1162.350 1200.926 0.82011 0.00 0.00 8.191 8.191 8.191 ...
      // format - 2
      //         time    scan  res start-angle range count range range range ....
      // 1175162437.565549 553237 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.667 1.623 1.682 1.880 ...
      // 1175162438.698901 553322 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.665 1.624 1.682 1.890 ...
      p1 = replayLine;
      t1 = strtod(p1, &p1); // time
      u1 = strtol(p1, &p2, 10); // (time again - or tilt?)
      if (*p2 == '.')
      { // format 1 from wpf.log
        printf("replay line with time %.3f is old format!\n", t1);
        //t2 = strtod(p1, &p1); // (time again - or tilt)
        pose.x = strtod(p1, &p1); // odoX
        pose.y = strtod(p1, &p1); // odoY
        pose.h = strtod(p1, &p1); // odoTh
        //px = strtod(p1, &p1); // plan x
        //py = strtod(p1, &p1); // plan y
        rngRes = 1.0;
        rngStartAng = -90.0;
        rngCnt = 181;
        pose.t = t;
// old format without odometry log is no longer supported
//        if (poseHist != NULL)
//        { /** @todo velocity not available to replay pose history - is this a problem? * /
//          poseHist->addIfNeeded( pose, 0.0, -3);
//        }
        u1 = replayLogLine;
      }
      else
      { // format is just laserscan - i.e. no robot position
        u1 = strtol(p1, &p1, 10); // scan number
        rngRes = strtod(p1, &p1); // angle resolution
        rngStartAng = strtod(p1, &p1); // start angle
        rngCnt = strtol(p1, &p1, 10); // range count
//        if (poseHist != NULL)
//        { /** @todo should be moved to replay master */
//          if (poseHist->isReplay())
//            poseHist->replayTime(t);
//        }
      }
      result = (p2 != NULL);
      if (result)
      { // all non-range values read - speed is optional
        scan.lock();
        // data set serial number
        scan.setSerial(u1);
        // set also device serial number (someone may use this number)
        serial = u1;
        scan.setAngleResAndStart(rngStartAng, rngRes);
        scan.setScanTime(t - var.scanDelay->getValued());
        scan.setDeviceNum(getDeviceNum());
        replayTimeNow = t;
        rngI = scan.getRange(0);
        rngFlagI = scan.getFlags(0);
        scan.setUnit(1); // unit is mm
        for (i = 0; i < rngCnt; i++)
        {
          rng = strtod(p1, &p1);
          if (p1 == NULL)
            // no more data
            break;
          *rngFlagI++ = (rng > maxValidRange);
          *rngI++ = roundi(rng * 1000);
          if (rng > maxRng)
          {
            if (rng < 8.2)
              maxRng = 8.2;
            else
              maxRng = 80.0;
          }
        }
        // valid if right amount of range data
        scan.setValid(i == rngCnt);
        scan.setRangeCnt(i);
        scan.setMaxValidRange(maxRng - 0.1);
        // set actual scan as device mode
        angleResolution = rngRes;
        modeAngleScan = roundi(absf(rngStartAng) * 2.0);
        if (mirrorData)
          scan.setAngleResAndStart(-rngStartAng, -rngRes);
        // execute push commands - if any pending
        gotNewScan(&scan);
        scan.unlock();
      }
    }
    do
    {
      p3 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
      if (p3 != NULL)
        t.setTimeTod(replayLine);
      else
        t.valid = false;
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

////////////////////////////////////////////////////////////////

bool UReplayDevice::getNewestData(ULaserData * dest,
                             unsigned long lastSerial,
                             int fake)
{
  bool result = (dest != NULL);
  //
  if (result)
  {
    if (fake > 0)
      getFakeScan(dest, lastSerial, fake);
    else
    { // replay data
      scan.lock();
      if (scan.getSerial() > lastSerial)
        dest->copy( &scan);
      else
      {
        dest->setValid(false);
        result = false;
      }
      scan.unlock();
    }
  }
  return result;
}

//////////////////////////////////////////////////////////

bool UReplayDevice::getReplayFileExist()
{
  const int MFL = 500;
  char fn[MFL];
  FILE * f;
  //
  getReplayFileName(fn, MFL);
  f = fopen(fn, "r");
  if (f != NULL)
    fclose(f);
  return (f != NULL);
}

