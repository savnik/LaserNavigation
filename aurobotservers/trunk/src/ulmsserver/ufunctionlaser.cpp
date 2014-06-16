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

#include <pthread.h>

#include <urob4/usmltag.h>
#include <urob4/uresposehist.h>

#include "usickdata.h"
#include "ufunctionlaser.h"
#include "usick.h"
#include "uhokuyo.h"
#include "ulasersim.h"
#include "ufakedevice.h"
#include "ureplaydevice.h"
#include "ulms100.h"

UFunctionLaser::UFunctionLaser()
{
  int i;
  //
  setCommand("scanGet scanPush scanSet", "laserdevs", "handles scannder device settings (by jca" __DATE__ " " __TIME__ ")");
  lasPool = NULL;
  for (i = 0; i < (MAX_LASER_DEVS + 1); i++)
    lastSerial[i] = 0;
  lasPoolLocal = false;
  v360 = NULL;
  poseHist = NULL;
}

/////////////////////////////////////////////////////////////

UFunctionLaser::~UFunctionLaser()
{ // dispose local ressource if mine
  if (lasPool != NULL)
    delete lasPool;
}

////////////////////////////////////////////

// const char * UFunctionLaser::resourceList()
// {
//   return "lasPool vl360";
// }

///////////////////////////////////////////////////

void UFunctionLaser::createResources()
{
  if (lasPool == NULL)
  { // no pool - so (try to) create one
    lasPool = new ULaserPool();
    addResource(lasPool, this);
  }
}

///////////////////////////////////////////////////

// UResBase * UFunctionLaser::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, ULaserPool::getResClassID()) == 0)
//   { // my ressource is requested
//     if (lasPool == NULL)
//     { // no pool - so (try to) create one
//       lasPool = new ULaserPool();
//       lasPoolLocal = (lasPool != NULL);
//     }
//     result = lasPool;
//   }
//   else if (strcmp(resID, UResV360::getResClassID()) == 0)
//   { // my ressource is requested
//     if (v360 == NULL)
//     { // no pool - so (try to) create one
//       v360 = new UResV360();
//       v360Local = (v360 != NULL);
//     }
//     result = v360;
//   }
//   //
//   if (result == NULL)
//     // may need a different ressource
//     result = UFunctionBase::getResource(resID);
//   return result;
// }

//////////////////////////////////////////////////////////////////

bool UFunctionLaser::setResource(UResBase * res, bool remove)
{
  bool result = true;
  //
  if (res->isA(ULaserPool::getResClassID()))
  { // is a laser pool ressource
    if (lasPoolLocal)
      result = false; // no change
    else
    {
      if (remove)
        lasPool = NULL;
      else if (lasPool != res)
        lasPool = (ULaserPool *) res;
      else
        result = false;
    }
  }
  else if (res->isA(UResV360::getResClassID()))
  { // is a virtual 360 deg laser scanner
    if (remove)
      v360 = NULL;
    else if (v360 != res)
      v360 = (UResV360 *) res;
    else
      result = false;
  }
  else if (res->isA(UResPoseHist::getOdoPoseID()))
  { // is a virtual 360 deg laser scanner
    if (remove)
      poseHist = NULL;
    else if (poseHist != res)
      poseHist = (UResPoseHist *) res;
    else
      result = false;
    fakeMap.resOdo = poseHist;
  }
  else if (res->isA(UResPoseHist::getUtmPoseID()))
  { // is a virtual 360 deg laser scanner
    if (remove)
      fakeMap.resUTM = NULL;
    else if (fakeMap.resUTM != res)
      fakeMap.resUTM = (UResPoseHist *) res;
    else
      result = false;
  }
  else
    result = UFunctionBase::setResource(res, remove);
  //
  return result;
}

/////////////////////////////////////////////////////////////

// bool UFunctionLaser::gotAllResources(char * missingThese, int missingTheseCnt)
// {
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = lasPool != NULL;
//   if ((not isOK) and (p1 != NULL))
//   {
//     snprintf(p1, missingTheseCnt, " %s", ULaserPool::getResClassID());
//     n += strlen(p1);
//     p1 = &missingThese[n];
//     result = false;
//   }
//   isOK = poseHist != NULL;
//   if ((not isOK) and (p1 != NULL))
//   {
//     snprintf(p1, missingTheseCnt, " %s", UResPoseHist::getOdoPoseID());
//     n += strlen(p1);
//     p1 = &missingThese[n];
//     result = false;
//   }
//   isOK = UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result and isOK;
// }

/////////////////////////////////////////////////////////////

bool UFunctionLaser::handleCommand(UServerInMsg * msg, void * extra)
{
  bool result = false;
  //
  if (msg->tag.isTagA("scanGet"))
    result = handleScanGetCommand(msg, extra);
  else if (msg->tag.isTagA("scanPush"))
    result = handleScanPushCommand(msg);
  else if (msg->tag.isTagA("scanset"))
    result = handleSetCommand(msg);
/*  else if (msg->tag.isTagA("get"))
    result = handleGetCommand(msg);*/
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;

}

/////////////////////////////////////////////////////////////

bool UFunctionLaser::handleScanGetCommand(UServerInMsg * msg, void * extra)
{ // get parameters
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 50;
  char attValue[VAL_BUFF_LNG];
  int n;
  unsigned int dataInterval = 1;
  ULaserDevice * sick = NULL;
  unsigned long * last = 0;
  ULaserData * laserData = (ULaserData *)extra;
  ULaserData laserDataBuff;
  UTime t;
  Dataformat dataCodex = HEX;
  int modeFake = 0;
  bool devV360 = false;
  bool ask4help = false;
  bool getScan = true;
  bool getPose = true;
  bool sendPose = false;
  bool unusedParams = false;
  const int MRL = 1000;
  char reply[MRL];
  char unused[MRL] = "";
  int unusedCnt=0;
  bool doLog = false;
  bool gotScan = false;
  bool useFakePose = false;
  UPoseTVQ poseFake;
  //
  if (lasPool != NULL)
  { // get default device
    sick = lasPool->getDefDevice();
    last = &lastSerial[lasPool->getDefDeviceNumber()];
  }
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strncasecmp(attName, "interval", 3) == 0)
      n = sscanf(attValue, "%u", &dataInterval); // interval between measurements
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "any") == 0)
      *last = 0;
    else if (strcasecmp(attName, "pose") == 0)
    {
      if (strlen(attValue) == 0)
        getPose = true;
      else
        getPose = str2bool(attValue);
      // explicit pose - send to odoPose if fake
      useFakePose = getPose;
    }
    else if (strcasecmp(attName, "log") == 0)
    {
      if (strlen(attValue) > 0)
        doLog = str2bool(attValue);
      else
        doLog=true;
    }
    else if (strcasecmp(attName, "device") == 0)
    {
      if (strcasecmp(attValue, "V360") == 0)
      { // request of data from virtual laser device
        devV360 = true;
        n = MAX_LASER_DEVS;
      }
      else
      {
        n = strtol(attValue, NULL, 0); // device
        if (n < 0)
          n = 0;
        if (lasPool != NULL)
        {
          lasPool->setDefDevice(n);
          sick = lasPool->getDefDevice();
        }
      }
      last = &lastSerial[n];
    }
    else if (strncasecmp(attName, "fake", 4) == 0)
    {
      if (strcasecmp(attValue, "false") == 0)
        modeFake = 0;
      else if (strlen(attValue) == 0)
        modeFake = 1; // no value, just fake
      else
        modeFake = strtol(attValue, NULL, 10);
    }
    else if  (strncasecmp(attName, "codex", 4) == 0)
    {
      if (strncasecmp(attValue, "bin", 3) == 0)
        dataCodex = BIN;
      else if (strncasecmp(attValue, "hex", 3) == 0)
        dataCodex = HEX;
      else if (strncasecmp(attValue, "TAG", 3) == 0)
        dataCodex = TAG;
    }
    else if (strcasecmp(attName, "silent") == 0)
      silent = str2bool2(attValue, true);
    else
    { // ignore unknown attributes
      unusedParams = true;
      snprintf(&unused[unusedCnt], MRL - unusedCnt - 1,
                " %s='%s'", attName, attValue);
      unusedCnt=strlen(unused);
    }
  }
  //
  if (ask4help)
  {
    sendMsg(msg, "<help subject='SCANGET'>\n");
    sendText(msg, "--------------Available SCANGET options:\n");
    sendText(msg, "fake [=N]     Get fake set of values (N=1..4 def=1)\n");
    sendText(msg, "              N=1 random data\n");
    sendText(msg, "              N=2,3 fake corridor\n");
    sendText(msg, "codec=BIN | HEX | TAG  Format (def=HEX)\n");
    sendText(msg, "device=N      Device N becomes new default device\n");
    sendText(msg, "device='V360' Get data from virtual 360 deg device (source=default).\n");
    sendText(msg, "interval=N    Send every N measurements only (def=1)\n");
    sendText(msg, "pose[=false]  Send robots odo-pose with scan (requires odoPose)\n");
    sendText(msg, "              and pose of device on robot (default=true)\n");
    sendText(msg, "              When combined with fake, then odometry pose is updated\n");
    sendText(msg, "any           Use any available scan (ignore scan number)\n");
    sendText(msg, "silent        do not send any reply (just activate any push commands)\n");
    sendText(msg, "log           Save this scan into log (use scanset to open log)\n");
    sendText(msg, "----\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if ((lasPool!= NULL) and (sick != NULL)) // laserpool is a must (v360 an option)
  {
    //sick->setSimulatedMode(modeFake);
    if (unusedParams)
    {
      snprintf(reply, MRL,
               "Some parameters to scanGet are ignored (%s)", unused);
      sendWarning(msg, reply);
    }
    if (getScan)
    { // get data and data source
      if (laserData != NULL)
        // from a push command - get pointer to source device (for sensor pose)
        sick = lasPool->getDevice(laserData->getDeviceNum());
      else
      {
        // get fresh data from sensor
        laserData = &laserDataBuff;
        // either virtual scanner or device direct
        if (devV360)
        { // request from virtual 360 deg laser scanner
          if (v360 != NULL)
          { // get a copy of data from virtual device
            v360->update(NULL, NULL, modeFake);
            gotScan = v360->getNewestData(laserData, *last, modeFake);
          }
        }
      }
      if (not (laserData->isValid() or devV360))
      { // get a copy of newest data - if not pushed data
        gotScan = sick->getNewestData(laserData, *last, modeFake);
      }
      // send reply - if available
      if (laserData->isValid())
      { // ensure pose is in laser data structure
        if (getPose)
        { // pose is requested
          if (laserData->isFake() and useFakePose and poseHist != NULL)
          { // update odoPose with fake pose
            poseFake = laserData->getFakePoseTime();
            printf("Data is fake and pose is %gx %gy %gm/s\n", poseFake.x, poseFake.y, poseFake.vel);
            poseHist->addIfNeeded(poseFake, -16);
          }
          sendPose = true;
          if (modeFake == 0)
          {
            if (poseHist != NULL)
              laserData->setFakePose(poseHist->getPoseAtTime(laserData->getScanTime()));
            else
              sendPose = false;
          }
        }
        if (doLog)
          sick->logThisScan(laserData);
        // send scan as requested
        if (not silent)
          sendScan(msg, laserData, dataInterval, dataCodex, sick, sendPose);
      }
      else
      { // no valid data
        if (devV360)
          sendWarning(msg, "No virtual (v360) scandata available");
        else
        { // device pool error -- explain
          if (not sick->isPortOpen())
          {
            snprintf(reply, MRL, "Can not open %s", sick->getDeviceName());
            sendWarning(msg, reply);
          }
          else
          {
            snprintf(reply, MRL, "No data (gotData=%s dataValid=%s serial=%lu lastSerial=%lu)\n",
                     bool2str(gotScan), bool2str(laserData->isValid()),
                     laserData->getSerial(), *last);
            sendWarning(msg, reply);
          }
        } 
      }
      if (gotScan)
        *last = laserData->getSerial();
    }
  }
  else
  {
    if (lasPool == NULL)
    {
      snprintf(reply, MRL, "Missing laserPool resource");
      sendWarning(msg, reply);
    }
    else
      sendWarning(msg, "no such device");
  }
  silent = false;
  return result;
}

///////////////////////////////////////////////////////////////////////

bool UFunctionLaser::sendScan(UServerInMsg * msg, ULaserData * laserData,
                           int dataInterval,
                           Dataformat dataCodex,
                           ULaserDevice * sick,
                             bool andPose)
{
  int mv;
  int dataFirst = 0;
  int dataCount = 0;
  double scaleFac;
  const int RL = MAX_RANGE_VALUES * 4; // for hex format
  char reply[RL];
  int n, i, f;
  int * pr; // pointer to range integer
  int * pf; // pointer to flags
  bool result = laserData != NULL and msg != NULL;
  UTime t;
  const int MSL = 20;
  char s[MSL], s2[MSL];
  unsigned char * pd;
  UPose pose;
  USmlTag tag;
  UPosition pos;
  URotation rot;
  //
  if (result)
  { // get measured values
    mv = laserData->getRangeCnt();
    //
    dataInterval = maxi(1, mini(dataInterval, mv));
    // number of ranges to be send
    dataCount = mv / dataInterval;
    if (laserData->isMirror())
    {
      dataFirst = mv - 1;
      dataInterval *= -1;
    }
    else
      dataFirst = 0;
  }
  //
  if (result)
  { // send requested data
    t = laserData->getScanTime();
    switch (laserData->getUnit())
    {
      case 0:
        strcpy(s, "cm");
        scaleFac = 0.01;
        break;
      case 1:
        strcpy(s, "mm");
        scaleFac = 0.001;
        break;
      case 2:
        strcpy(s, "10cm");
        scaleFac = 0.1;
        break;
      default:
        strcpy(s, "invalid");
        scaleFac = 0.001;
        break;
    }
    switch (dataCodex)
    { // make format to string for reply
      case BIN: strcpy(s2, "BIN");    break;
      case HEX: strcpy(s2, "HEX");    break;
      case TAG: strcpy(s2, "TAG");    break;
      default: strcpy(s2, "invalid"); break;
    }
    snprintf(reply, RL, "<%s serial=\"%lu\" interval=\"%g\" "
          "count=\"%d\" tod=\"%ld.%06ld\" unit=\"%s\" "
          "min=\"%4.2f\" max=\"%4.2f\" codec=\"%s\" maxValidRange=\"%.2g\">\n",
          msg->tag.getTagName(), laserData->getSerial(),
          laserData->getAngleResolutionDeg() * dataInterval,
          dataCount,
          t.GetSec(), t.getMicrosec(), s,
          laserData->getAngleDeg(dataFirst),
          laserData->getAngleDeg(dataFirst + (dataCount - 1) * dataInterval),
          s2, laserData->getMaxValidRange());
    sendMsg(msg, reply);
    if (andPose)
    { // send also robot and sensor pose
      // robot pose (x,y,Th)
      sendMsg(msg, tag.codePose(laserData->getFakePose(), reply, RL, "robot"));
      // sensor position (x,y,z) relative to robot origin
      sendMsg(msg, tag.codePosition(sick->getDevicePos(), reply, RL, "device"));
      // sensor roattion (Omega, Phi, Kappa) relative to robot
      sendMsg(msg, tag.codeRotation(sick->getDeviceRot(), reply, RL, "device"));
    }
    //
    if (dataCount > 0)
    { // send data as one binary pack
      if (dataCodex != TAG)
      { // send in bin tag if format is HEX or binary
        if (dataCodex == BIN)
          n = dataCount * 2;
        else
          n = dataCount * 4;
        snprintf(reply, RL, "<bin size=\"%d\" codec=\"%s\">", n, s2);
        sendMsg(msg, reply);
      }
      // send data
      pd = (unsigned char *)reply;
      pr = laserData->getRange(dataFirst);
      pf = laserData->getFlags(dataFirst);
      n = 0;
      switch (dataCodex)
      {
        case BIN:
          for (i = 0; i < dataCount; i++)
          { // pack binary
            // pack flag as first bit - changed with lms100 to allow 15 bit range
            if ((*pf <= 7) and (*pf > 0))
              f = 0x80;
            else
              f = 0;
            // pack as little endian binary
            *pd++ = (*pr) & 0xff;
            *pd++ = ((*pr) >> 8) + f;
            pr += dataInterval;
            pf += dataInterval;
            n += 2;
          }
          sendMsg(msg, reply, n);
          break;
        case HEX:
          for (i = 0; i < dataCount; i++)
          { // format in hex
            // pack flag as first bit - changed with lms100 to allow 15 bit range
            if ((*pf <= 7) and (*pf > 0))
              f = 0x80;
            else
              f = 0;
            // hex format (little endian - LSB first)
            sprintf((char *)pd, "%02x%02x", *pr & 0xff, (*pr >> 8) + f);
            n += 4;
            pd += 4;
            pr += dataInterval;
            pf += dataInterval;
          }
          sendMsg(msg->client, reply, n);
          break;
        case TAG:
          for (i = 0; i < dataCount; i++)
          { // send count number of messages - decode values
            snprintf(reply, RL, "<lval f=\"%d\" ang=\"%4.2f\" dist=\"%1.3f\"/>\n",
                        *pf,
                        laserData->getAngleDeg(dataFirst + i * dataInterval),
                        double(*pr * scaleFac));
            sendMsg(msg, reply);
            pr += dataInterval;
            pf += dataInterval;
          }
          break;
        default:
          break;
      }
      if (dataCodex != TAG)
        sendMsg(msg, "</bin>\n");
      snprintf(reply, RL, "</%s>\n", msg->tag.getTagName());
      result = sendMsg(msg, reply);
    }
  }
  return result;
}

///////////////////////////////////

bool UFunctionLaser::handleScanPushCommand(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  int n;
  ULaserDevice * sick = NULL;
  //unsigned long * last = 0;
  bool ask4help = false;
  bool ask4list = false;
  const int MRL = 1000;
  char reply[MRL];
  //
  if (lasPool != NULL)
  { // get default device
    sick = lasPool->getDefDevice();
    //last = &lastSerial[lasPool->getDefDeviceNumber()];
  }
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4list = msg->tag.getAttValue("list", val, VBL);
  if (msg->tag.getAttValue("device", val, VBL))
  { // get (non default) device
    n = strtol(val, NULL, 0); // device number
    if (n < 0)
      n = 0;
    if (lasPool != NULL)
      sick = lasPool->getDevice(n);
    //last = &lastSerial[n];
  }
  // ignore all other attributes
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"SCANPUSH\">\n");
    sendText(msg, "----------------Available SCANPUSH settings:\n");
    sendText(msg, "device=N        Push from device 'N' (else default device)\n");
    sendText(msg, "good=k or g=k   Stop push after k good scans (def: no stop).\n");
    sendText(msg, "total=k or n=k  Stop push after k scans (def: no stop)\n");
    sendText(msg, "interval=k or i=k Push with this interval (in scans, def = 1)\n");
    sendText(msg, "flush=cmd       Remove all push commands from client (default all)\n");
    sendText(msg, "cmd=\"cmd\"       Do execute 'cmd' command for every available scan\n");
    sendText(msg, "list            List all active push commands\n");
    sendText(msg, "See also PUSH (timed push), SCANGET, SCANSET\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (sick != NULL)
  { // push command
    if (ask4list)
    {
      sendMsg(msg, "<help subject=\"scanPush command list\">\n");
      sick->UServerPush::print("scanPush", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    {
      result = sick->addPushCommand(msg);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d",
                sick->getPushCmdCnt(NULL, NULL));
        sendInfo(msg, val);
      }
      else
        sendWarning(msg, "push command failed");
    }
  }
  else
    sendWarning(msg, "No such device");

  return result;
}

////////////////////////////////////////////////////////

bool UFunctionLaser::handleSetCommand(UServerInMsg * msg)
{
  bool result = true;
  char attName[MAX_SML_NAME_LENGTH];
  const int VBL = 50;
  char attValue[VBL];
  int n;
  const int MRL = 5000;
  char reply[MRL];
  const int MSL = 300;
  char s[MSL];
  char sendStr[MSL] = "V";
  bool doClose = false;
  ULaserDevice * sick = NULL;
  unsigned int scanWidth = 0;
  double scanRes = -1.0;
  bool ask4help = false;
  bool doSend = false;
  bool doScanRes = false;
  bool doLogOpen = false;
  bool doVerbose = false;
  bool doVerboseValue = false;
  bool newDevId = false;
  bool newDevName = false;
  bool newMirror = false;
  bool mirrorValue = false;
  bool doLogClose = false;
  bool doX = false;
  bool doY = false;
  bool doZ = false;
  bool doOmega = false;
  bool doPhi   = false;
  bool doKappa = false;
  bool getPose = false;
  bool doLogM = false;
  int logM = 0;
  bool logUsed = false;
  char newDevIdStr[VBL];
  char newDevNameStr[VBL];
  bool doSetDef = false;
  const int MDL = 30;
  char setDef[MDL] = "";
  char * p1;
  int simPort;
  int defDev = 0;
  const int SimBasePortNumberLaser  = 19000;
  const int SimBasePortNumberServer = 20000;
  UPosition pos, *ppos;
  URotation rot, *prot;
  UPosRot pr;
  UPosRot sensorPose;
  USmlTag tag;
  double scanWIs = 0.0;
  double scanResIs = 0.0;
  const char * devType = "-";
  const char * devName = "-";
  bool isVerbose = false;
  bool isOpen = false;
  bool aReplay = false; // on off
  bool aReplayTime = false; // get or set time
//  bool aReplayDir = false; // get or set subdir
  bool aReplayStep = false; // step one entry in logfile
  bool replay = false;
  bool replayDev = false;
  bool replayOpen = false;
  bool replayFileExist = false;
  int replaySteps = 1;
  bool aReplayToScan = false;
  unsigned int replayToScan = 0;
//  char replaySubdir[VBL];
  char * replayLogFile = NULL;
  UTime replayTimeLast, replayTimeNext, t;
  const int MFL = 100;
  char replayFilename[MFL] = "";
  bool ignoresOptions = false;
  const int MIL = 100;
  char ignores[MIL] = "";
  UReplayDevice * rDev = NULL;
  bool replyOK = false;
//  const char * replayDir = NULL;
  int devCnt = 0;
  bool aDevList = false;
  bool silent = false;
  //
  if (lasPool != NULL)
    // get default device
    sick = lasPool->getDefDevice();
  //
  while (msg->tag.getNextAttribute(attName, attValue, VBL))
  {
    if (strcasecmp(attName, "close") == 0)
      doClose = true;
    else if (strcasecmp(attName, "width") == 0)
    {
      n = sscanf(attValue, "%u", &scanWidth); // width 180 or 100
      doScanRes = true;
    }
    else if (strcasecmp(attName, "res") == 0)
    {
      n = sscanf(attValue, "%lf", &scanRes); // number of values
      doScanRes = true;
    }
    else if (strcasecmp(attName, "send") == 0)
    {
      doSend = true;
      strncpy(sendStr, attValue, MSL);
    }
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "logOpen") == 0)
    { // allow to set logOpen=false to close
      if (str2bool2(attValue, true))
        doLogOpen = true;
      else
        doLogClose = true;
    }
    else if (strcasecmp(attName, "logClose") == 0)
      doLogClose = true;
    else if (strcasecmp(attName, "def") == 0)
    {
      doSetDef = true;
      strncpy(setDef, attValue, MDL);
    }
    else if (strcasecmp(attName, "device") == 0)
    {
      if (lasPool != NULL)
        sick = lasPool->getDevice(strtol(attValue, NULL, 0));
    }
    else if (strcasecmp(attName, "devType") == 0)
    {
      newDevId = true;
      strncpy(newDevIdStr, attValue, VBL);
    }
    else if (strcasecmp(attName, "devName") == 0)
    {
      newDevName = true;
      strncpy(newDevNameStr, attValue, VBL);
    }
    else if (strcasecmp(attName, "mirror") == 0)
    {
      newMirror = true;
      mirrorValue = true;
      if (strlen(attValue) > 0)
        mirrorValue = str2bool(attValue);
    }
    else if (strcasecmp(attName, "pose") == 0)
      getPose = true;
    else if (strcasecmp(attName, "x") == 0)
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doX = true;
        pos.x = strtod(attValue, NULL);
      }
    }
    else if (strcasecmp(attName, "y") == 0)
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doY = true;
        pos.y = strtod(attValue, NULL);
      }
    }
    else if (strcasecmp(attName, "z") == 0)
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doZ = true;
        pos.z = strtod(attValue, NULL);
      }
    }
    else if ((strcasecmp(attName, "roll") == 0) or
              (strcasecmp(attName, "omega") == 0))
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doOmega = true;
        rot.Omega = strtod(attName, NULL);
      }
    }
    else if ((strcasecmp(attName, "tilt") == 0) or
              (strcasecmp(attName, "pitch") == 0) or
              (strcasecmp(attName, "phi") == 0))
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doPhi = true;
        rot.Phi = strtod(attValue, NULL);
      }
    }
    else if ((strcasecmp(attName, "turn") == 0) or
              (strcasecmp(attName, "yaw") == 0) or
              (strcasecmp(attName, "kappa") == 0))
    {
      if (strlen(attValue)==0)
        getPose=true;
      {
        doKappa = true;
        rot.Kappa = strtod(attValue, NULL);
      }
    }
    else if (strcasecmp(attName, "verbose") == 0)
    {
      doVerbose = true;
      doVerboseValue = true;
      if (strlen(attValue) > 0)
        doVerboseValue = str2bool(attValue);
    }
    else if (strcasecmp(attName, "log") == 0)
    {
      doLogM = true;
      if (strlen(attValue) > 0)
      {
        if (strcasecmp(attValue, "used") == 0)
          logUsed = true;
        else
          logM = strtol(attValue, NULL, 0);
      }
    }
    else if (strcasecmp(attName, "replay") == 0)
    {
      aReplay = true;
      if (strlen(attValue) == 0)
        replay = true;
      else if (strcasecmp(attValue, "false") == 0)
        replay = false;
      else if (strcasecmp(attValue, "true") == 0)
        replay = true;
      else
      {
        replay = true;
        strncpy(replayFilename, attValue, MFL);
      }
    }
/*    else if (strcasecmp(attName, "replaySubdir") == 0)
    {
      aReplayDir = true;
      strcpy(replaySubdir, attValue);
    }*/
    else if (strcasecmp(attName, "replayTime") == 0)
    {
      aReplayTime = true;
      replayTimeNext.setTimeTod(attValue);
    }
    else if (strcasecmp(attName, "step") == 0)
    {
      aReplayStep = true;
      if (strlen(attValue) > 0)
        replaySteps = strtol(attValue, NULL, 0);
    }
    else if (strcasecmp(attName, "toscan") == 0)
    {
      aReplayToScan = true;
      if (strlen(attValue) > 0)
        replayToScan = strtol(attValue, NULL, 0);
    }
    else if (strcasecmp(attName, "list") == 0)
      aDevList = true;
    else if (strcasecmp(attName, "silent") == 0)
      silent = true;
    else
    { // else ignore attribute
      ignoresOptions = true;
      n = strlen(ignores);
      if ((n > 0) and (n < MIL))
        ignores[n++] = ' ';
      strncpy(&ignores[n], attName, MIL - n);
    }
  }
  //
  if ((sick == NULL) and (lasPool != NULL) and not ask4help)
    sick = lasPool->getDefDevice();
  //
  if (ask4help)
  {
    defDev = -1;
    if (sick != NULL)
    {
      logM = sick->getLogInterval();
      defDev = sick->getDeviceNum();
      scanWIs = sick->getScanAngle();
      scanResIs = sick->getScanResolution();
      devType = sick->getName();
      devName = sick->getDeviceName();
      sensorPose = sick->getDevicePose();
      isVerbose = sick->isVerbose();
      isOpen = sick->isPortOpen();
      replayDev = sick->isReplayDevice();
      devCnt = lasPool->getDeviceCnt();
      logUsed = sick->getLogUsedScans();
      logM = sick->getLogInterval();
      if (replayDev)
      {
        rDev = (UReplayDevice *) sick;
        replay = rDev->isReplayDevice();
//        replayDir = rDev->getReplaySubdir();
        replayTimeLast = rDev->getReplayTimeNow();
        replayTimeNext = rDev->getReplayTimeNext();
        replayOpen = rDev->isReplayFileOpen();
        replayLogFile = rDev->getDeviceName();
        replayFileExist = rDev->getReplayFileExist();
      }
    }
    sendMsg(msg, "<help subject=\"SCANSET\">\n");
    sendText(msg, "----------available SCANSET options:\n");
    snprintf(reply, MRL, "close            Close device port (open=%s)\n", bool2str(isOpen));
    sendText(msg, reply);
    snprintf(reply, MRL, "def=N | devType  Set default device (is %d, a '%s' on '%s')\n", defDev, devType, devName);
    sendText(msg, reply);
    sendText(msg,        "device=N         Specify device N\n");
    snprintf(reply, MRL, "width=W   W=100 | 180 | 240  Set scan width (now W=%.0f deg)\n", scanWIs);
    sendText(msg, reply);
    snprintf(reply, MRL, "res=A            Set scan resolution (now A=%g deg)\n", scanResIs);
    sendText(msg, reply);
    if (sick != NULL)
      snprintf(reply, MRL, "logOpen          Start logging to 'laser_N.log' (open=%s)\n",
               bool2str(sick->isLogFileOpen()));
    else
      snprintf(reply, MRL, "logOpen          Start logging to 'laser_N.log'\n");
    sendText(msg, reply);
    snprintf(reply, MRL,   "log=M            Log every M'th scan (0 is off) now M=%d\n", logM);
    sendText(msg, reply);
    snprintf(reply, MRL,   "log=used         Log used scans only (is %s)\n", bool2str(logUsed));
    sendText(msg, reply);
    sendText(msg,          "logClose         close logfile\n");
    snprintf(reply, MRL,   "send=str         Send this string to device (not all)\n");
    sendText(msg, reply);
    sendText(msg, "devType='type' devName='source'\n");
    sendText(msg, "          Make new device, typically:\n");
    sendText(msg, "          devType='sick'   devName='/dev/ttyUSB0'\n");
    sendText(msg, "          devType='urg'    devName='/dev/ttyACM0'\n");
    sendText(msg, "          devType='lms100' devName='lms100:2111'\n");
    sendText(msg, "          devType='sim'    devName='host:port'\n");
    sendText(msg, "          devType='sim'    devName='host:4' (for smr4)\n");
    sendText(msg, "          devType='replay' devName='0' (using laser_0.log)\n");
    sendText(msg, "          devType='fake'   devName='sick' or 'urg' or 'rnd'\n");
    snprintf(reply, MRL,"list               List loaded (%d) devices\n", devCnt);
    sendText( msg, reply);
    sendText(msg, "mirror[=false] Device mounted upside down (roll=pi)\n");
    sendText(msg, "pose         Get sensor pose [m]\n");
    snprintf(reply, MRL, "x=X y=Y z=Z  Set position of device relative to robot [m] is (%gx, %gy, %gz)\n",
             sensorPose.getX(), sensorPose.getY(), sensorPose.getZ());
    sendText(msg, reply);
    snprintf(reply, MRL, "Roll=rad     Set Roll (Omega) (x-axis) right is positive       is %grad\n", sensorPose.getOmega());
    sendText(msg, reply);
    snprintf(reply, MRL, "Tilt=rad     Set Tilt (pitch or Phi) (y-axis) down is positive is %grad\n", sensorPose.getPhi());
    sendText(msg, reply);
    snprintf(reply, MRL, "Turn=rad     Set turn (yaw or Kappa) (z-axis) left is positive is %grad\n", sensorPose.getKappa());
    sendText(msg, reply);
    snprintf(reply, MRL, "verbose[=false] More print to server console (for device) now verbose=%s\n", bool2str(isVerbose));
    sendText(msg, reply);
    snprintf(reply, MRL, "---- replay device (%s) options:\n", bool2str(replayDev));
    sendText( msg, reply);
    snprintf(reply, MRL, "replay[=false]  Set replay (file=%s Open=%s found=%s)\n",
             replayLogFile, bool2str(replayOpen), bool2str(replayFileExist));
    sendText( msg, reply);
/*    snprintf(reply, MRL,"replaySubdir='dir' Set subdir to logfile (is '%s') from log path\n",
             replayDir);
    sendText( msg, reply);*/
/*    snprintf(reply, MRL, "                   log path is 'dataPath' (is %s)\n", dataPath);
    sendText( msg, reply);*/
    sendText( msg,      "step=N             Advance replay by N scanlines\n");
    sendText( msg,      "toScan=S           Advance replay to scan S\n");
    snprintf(reply, MRL,"replayTime[=tod]   Get or set replay to tod (now %lu.%06lu)\n",
             replayTimeLast.getSec(), replayTimeLast.getMicrosec());
    sendText( msg, reply);
    sendText(msg, "----\n");
    sendText(msg, "Se also 'MODULE RESLIST' for laser devices, and 'server help' for path settings\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (ignoresOptions)
  {
    snprintf(reply, MRL, "unknown SCANSET option(s): %s", ignores);
    sendWarning(msg, reply);
  }
  else if (newDevId and newDevName)
  { // rewuest for new device
    if (lasPool == NULL)
    { // no laser pool
      snprintf(reply, MRL, "No laser scanner device pool (lasPool)");
      result = false;
    }
    if (strcasecmp(newDevIdStr, "sick") == 0)
    {
      sick = new USick();
      sick->setDeviceName(newDevNameStr); // sick device name - like '/dev/ttyUSB1'
      sick->changeMode(180, 1.0); // 180 deg 1.0 deg default resolution
      lasPool->addDevice(sick);
      snprintf(reply, MRL, "Made device=%d a %s scanner on %s (not open)",
             lasPool->getDeviceCnt()-1,
             sick->getName(),
             sick->getDeviceName());
    }
    else if (strcasecmp(newDevIdStr, "urg") == 0)
    {
      sick = new UHokuyo();
      sick->setDeviceName(newDevNameStr); // serial device like: /dev/ttyACM0
      sick->changeMode(240, 360.0/1024.0); // like 240 deg 0.351 deg resolution
      lasPool->addDevice(sick);
      snprintf(reply, MRL, "Made device=%d an %s scanner on %s (not open)",
             lasPool->getDeviceCnt()-1,
             sick->getName(),
             sick->getDeviceName());
    }
    else if (strcasecmp(newDevIdStr, "lms100") == 0)
    {
      sick = new ULms100();
      sick->setDeviceName(newDevNameStr); //
      sick->changeMode(270, 0.5); // e.g. 270 deg 0.5 deg resolution
      lasPool->addDevice(sick);
      snprintf(reply, MRL, "Made device=%d an %s scanner on %s (not open)",
               lasPool->getDeviceCnt()-1,
                                     sick->getName(),
                                         sick->getDeviceName());
    }
    else if (strcasecmp(newDevIdStr, "sim") == 0)
    { // find requested port number
      p1 = strchr(newDevNameStr, ':');
      // there must be a : in the host:port device name
      result = (p1 != NULL);
      if (result)
      { // get host name and port number from device name
        // format: host:port, e.g.: nyquist:19000
        p1++; // skip ':'
        simPort = strtol(p1, NULL, 0);
        if (simPort < 1000)
        { // change laser server port as well
          serverPort = SimBasePortNumberServer + simPort;
          // write new simport number
          simPort = SimBasePortNumberLaser + simPort;
          sprintf(p1,"%d",simPort);
        }
      }
      else
      {
        snprintf(reply, MRL, "Unknown name. '%s' is not a valid device name - see 'set help'", newDevNameStr);
      }
      if (result)
      {
        sick = new ULaserSim();
        sick->setDeviceName(newDevNameStr); // (host:port)
        //sick->changeMode(180, 1.0); // 180 deg 1.0 deg resolution
        lasPool->addDevice(sick);
        snprintf(reply, MRL, "Made device=%d a %s scanner on %s (not open)",
              lasPool->getDeviceCnt()-1,
              sick->getName(),
              sick->getDeviceName());
      }
    }
    else if (strcasecmp(newDevIdStr, "fake") == 0)
    {
      sick = new UFakeDevice();
      sick->setDeviceName(newDevNameStr); // serial dvice - smr16: ttyS4
      lasPool->addDevice(sick);
      snprintf(reply, MRL, "Made device=%d a %s scanner name %s (open)",
               lasPool->getDeviceCnt()-1,
               sick->getName(),
               sick->getDeviceName());
    }
    else if (strcasecmp(newDevIdStr, "replay") == 0)
    {
      sick = new UReplayDevice();
      // logfile should end in a .log
      if (strcasestr(newDevNameStr, ".log") == NULL)
        strncat(newDevNameStr, ".log", MAX_DEVICE_NAME_LNG);
      sick->setDeviceName(newDevNameStr); // filename
      lasPool->addDevice(sick);
      snprintf(reply, MRL, "Made replay device=%d using %s",
               lasPool->getDeviceCnt()-1,
               newDevNameStr);
    }
    else
    {
      result = false;
      snprintf(reply, MRL, "Unknown scanner type '%s'", newDevIdStr);
    }
    // send reply
    if (msg->client >= 0 and not silent)
    { // real client
      if (result)
        sendInfo(msg, reply);
      else
        // unknown device type
        sendWarning(msg, reply);
    }
    // print on console too
    printf("%s\n", reply);
    result = true;
  }
  else if (newDevId or newDevName)
  {
    sendWarning(msg, "DevType and DevName must come as a pair");
  }
  else if (sick != NULL)
  { // set all the remaining options
    if (doSetDef)
    { // set new default scanner
      if (strlen(setDef) > 0)
      { // is it a number
        n = strtol(setDef, &p1, 10);
        if (p1 > setDef)
          // parameter is a number
          lasPool->setDefDevice(n);
        else
          // a device type string
          lasPool->setDefDevice(setDef);
        sick = lasPool->getDefDevice();
      }
      snprintf(reply, MRL, "<%s def=\"%d\" devType=\"%s\" devName=\"%s\"/>\n",
                 msg->tag.getTagName(), sick->getDeviceNum(),
                 sick->getName(), sick->getDeviceName());
      sendMsg(msg, reply);
      replyOK = true;
    }
    // get default values
    if (scanWidth == 0)
      scanWidth = sick->getScanAngle();
    if (scanRes < 0.0)
      scanRes = sick->getScanResolution();
    if (doLogOpen)
      sick->logFileOpen();
    if (doLogClose)
      sick->logFileClose();
    if (newMirror)
    { // set mirror flag
      sick->setMirror(mirrorValue);
      // and set as rotation on Omega (180 deg)
/*      doOmega = true;
      if (mirrorValue)
        rot.Omega = M_PI;
      else
        rot.Omega = 0.0;*/
    }
    pr = sick->getDevicePose();
    if (doX)
      pr.setX(pos.x);
    if (doY)
      pr.setY(pos.y);
    if (doZ)
      pr.setZ(pos.z);
    if (doOmega)
      pr.setOmega(rot.Omega);
    if (doPhi)
      pr.setPhi(rot.Phi);
    if (doKappa)
      pr.setKappa(rot.Kappa);
    sick->setDevicePose(&pr);
    if (doVerbose)
      sick->setVerbose(doVerboseValue);
    if (doLogM)
    {
      sick->setLogInterval(logM);
      sick->setLogUsedScans(logUsed);
    }
    // need a full class spec to handle replay commands
    if (sick->isReplayDevice())
      rDev = (UReplayDevice *) sick;
    //
    if (aReplay and (rDev != NULL))
    {
      if (strlen(replayFilename) > 0)
        rDev->setDeviceName(replayFilename);
      result = rDev->setReplay( replay);
      if (result)
      {
        t = rDev->getReplayTimeNow();
        t.getTimeAsString(attValue, VBL);
        snprintf(reply, MRL, "<%s fileopen=\"%s\" tod=\"%lu.%06lu\" "
            "time=\"%s\" logLine=\"%d\"/>\n",
            msg->tag.getTagName(), bool2str(rDev->isReplayFileOpen()),
            t.getSec(), t.getMicrosec(), attValue, rDev->getReplayLogLine());
        sendMsg( msg, reply);
      }
      else
      {
        rDev->getReplayFileName(s, MSL);
        snprintf(reply, MRL, "No file or no data in file: %s", s);
        sendHelp(msg, reply);
      }
      replyOK = true;
    }
    if ((aReplayStep or aReplayToScan) and (rDev != NULL))
    {
      if (aReplayToScan)
        rDev->replayStepToScan(replayToScan, lasPool);
      else
        rDev->replayStep(replaySteps, lasPool);
      t = rDev->getReplayTimeNow();
      // advance robot pose history to the same time
      // NB! this may be implicit if the logfile is a wpf.log, but
      //     no harm is done if this is the case
/*      if (poseHist != NULL)
        poseHist->replayTime(t);
      //
      utmPose = (UResPoseHist*)getStaticResource("utmPose", false);
      if (utmPose != NULL)
        utmPose->replayTime(t);*/
      // print status
      t.getTimeAsString(attValue, VBL);
      if (not silent)
      {
        snprintf(reply, MRL, "<%s tod=\"%lu.%06lu\" time=\"%s\" logLine=\"%d\"/>\n",
                msg->tag.getTagName(), t.getSec(), t.getMicrosec(),
                attValue,  rDev->getReplayLogLine());
        sendMsg( msg, reply);
      }
      replyOK = true;
    }
    if (aReplayTime and (rDev != NULL))
    {
      if (replayTimeNext.valid)
        lasPool->replayAdvanceTime(replayTimeNext);
      t = rDev->getReplayTimeNow();
      //
/*      utmPose = (UResPoseHist*)getStaticResource("utmPose", false);
      if (utmPose != NULL)
        utmPose->replayTime(t);*/
      //
      t.getTimeAsString(attValue, VBL);
      if (not silent)
      {
        snprintf(reply, MRL, "<%s tod=\"%lu.%06lu\" time=\"%s\" logLine=\"%d\"/>\n",
                msg->tag.getTagName(), t.getSec(), t.getMicrosec(),
                attValue, rDev->getReplayLogLine());
        sendMsg( msg, reply);
      }
      replyOK = true;
    }
    // close command
    if (doClose)
    { // close port and stop continious mode
      if (sick->isPortOpen())
      {
        sick->stop(true);
        sendInfo(msg, "Scanner connection closed");
      }
      else
        sendWarning(msg, "Port not open");
      replyOK = true;
    }
    if (doScanRes or doSend) // or doRepeat)
    { // change setting
      if (doScanRes)
      {
        if (((int(scanWidth) != sick->getScanAngle()) or
          (absd(scanRes - sick->getScanResolution()) > 0.01)))
        { // change resolution
          result = sick->changeMode(scanWidth, scanRes);
          if (result)
          {
            snprintf(reply, MRL, "requested: width=%d res=%g",
                    sick->getScanAngle(),
                    sick->getScanResolution());
            sendInfo(msg, reply);
          }
          else
            sendWarning(msg, "Failed");
        }
        else
        {
          snprintf(reply, MRL, "already: width=%d res=%g",
                   sick->getScanAngle(),
                   sick->getScanResolution());
          sendInfo(msg, reply);
        }
      }
      if (doSend)
      {
        printf("Sending '%s'(+LF) to %s -- last str is send (%s)\n", sendStr,
               sick->getDeviceName(), bool2str(sick->isSend()));
        strcat(sendStr, "\n");
        snprintf(reply, MRL, "done -- last str is send (%s)",
                 bool2str(sick->isSend()));
        sick->send(sendStr);
        sendInfo(msg, reply);
      }
      replyOK = true;
    }
    if (doLogOpen or doLogM)
    {
      sick->getLogFileName(s, MSL);
      snprintf(reply, MRL, "Logging open=%s on %s every M=%d scan or every used scan (%s)",
                 bool2str(sick->isLogFileOpen()), s, sick->getLogInterval(),
                bool2str(sick->getLogUsedScans()));
      sendInfo(msg, reply);
      replyOK = true;
    }
    if (doX or doY or doZ or doOmega or doPhi or doKappa)
    {
      snprintf(reply, MRL, "sensor pose x=%g y=%g z=%g roll=%g tilt=%g turn=%g",
               pr.getX(), pr.getY(), pr.getZ(), pr.getOmega(), pr.getPhi(), pr.getKappa());
      sendInfo(msg, reply);
      replyOK = true;
    }
    if (getPose)
    { // send start tag
      n = sick->getDeviceNum();
      snprintf(reply, MRL, "<%s device=%d>\n", msg->tag.getTagName(), n);
      sendMsg(msg, reply);
      // send pose
      pr = sick->getDevicePose();
      ppos = pr.getPos();
      prot = pr.getRot();
      // tag.codePosition((const UPosition)ppos, reply, MRL, "device");
      ppos->codeXml("device", reply, MRL, NULL);
      sendMsg(msg, reply);
      prot->codeXml("device", reply, MRL, NULL);
      sendMsg(msg, reply);
      // send end tag
      snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
      sendMsg(msg, reply);
      replyOK = true;
    }
    if (not replyOK and not silent)
    { // just send status
      if (doLogClose or newMirror or doVerbose)
        sendInfo(msg, "Done");
      else
      {
        n = sick->getDeviceNum();
        snprintf(reply, MRL, "<%s device=%d running=\"%s\" "
            "width=\"%d\" res=\"%4.2f\" good=\"%u\" "
                "bad=\"%u\" rate=\"%4.2f\" dev=\"%s\" mirror=\"%s\"/>\n",
            msg->tag.getTagName(), n,
            bool2str(sick->isRunning()),
            sick->getScanAngle(),
            double(sick->getScanResolution()),
            sick->getGood(), sick->getBad(), sick->getMsgRate(),
            sick->getDeviceName(),
            bool2str(sick->getMirror()));
        sendMsg(msg, reply);
      }
    }
    if (aDevList)
    {
      sendMsg(msg, "<help subject=\"Device list\">\n");
      sendText(msg, "----available laserPool devices----\n");
      lasPool->print("Devices: ", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
  }
  else
    sendWarning(msg, "No such device");
  return result;
}

//////////////////////////////////////////////////

// bool UFunctionLaser::handleGetCommand(UServerInMsg * msg)
// {
//   bool result = true;
//   char attName[MAX_SML_NAME_LENGTH];
//   const int VAL_BUFF_LNG = 50;
//   char attValue[VAL_BUFF_LNG];
//   const int MRL = 1300;
//   char reply[MRL];
//   bool getName = false;
//   bool getStatus = false;
//   bool getMirror = false;
//   bool getPose = false;
//   ULaserDevice * sick = NULL;
//   bool ask4help = false;
//   const int MXL = 1000;
//   char xml[MXL];
//   char * ns;
//   int n = 0;
//   USmlTag tag;
//   UPosRot * pr;
//   UPosition * pos;
//   URotation * rot;
//   //
//   if (lasPool != NULL)
//   {
//     sick = lasPool->getDefDevice();
//     n = lasPool->getDefDeviceNumber();
//   }
//   //
//   while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
//   {
//     if (strcasecmp(attName, "name") == 0)
//       getName = true;
//     else if (strcasecmp(attName, "status") == 0)
//       getStatus = true;
//     else if (strcasecmp(attName, "help") == 0)
//       ask4help = true;
//     else if (strcasecmp(attName, "mirror") == 0)
//       getMirror = true;
//     else if (strcasecmp(attName, "pose") == 0)
//       getPose = true;
//     else if (strcasecmp(attName, "device") == 0)
//     {
//       n = strtol(attValue, NULL, 0);
//       if (lasPool != NULL)
//         sick = lasPool->getDevice(n);
//     }
//     // else ignore attribute
//   }
//   if (ask4help)
//   {
//     sendMsg(msg, "<help subject=\"GET\">\n");
//     sendText(msg, "----------------Available GET options:\n");
//     sendText(msg, "name            Get device port and name\n");
//     sendText(msg, "status          Get status\n");
//     sendText(msg, "device=N        Get from device (see module list)\n");
//     sendText(msg, "mirror          Get mirror status\n");
//     sendText(msg, "pose            Get position of laser device on robot\n");
//     sendText(msg, "----\n");
//     sendMsg(msg, "</help>\n");
//     sendInfo(msg, "done");
//     result=true;
//   }
//   else if (sick != NULL)
//   {
//     if (getName)
//     {
//       ns = sick->getName();
//       if (strlen(ns) < 10)
//         ns = sick->getNameFromDevice();
//       xml2str(xml, MXL, ns, strlen(ns));
//       snprintf(reply, MRL, "<%s device=%d type=\"%s\" name=\"%s\"/>\n",
//                msg->tag.getTagName(),
//                n,
//                sick->getDeviceName(), xml);
//       sendMsg(msg, reply);
//     }
//     else if (getStatus or getMirror)
//     { // just send status
//       snprintf(reply, MRL, "<%s device=%d running=\"%s\" "
//           "width=\"%d\" res=\"%4.2f\" good=\"%u\" "
//               "bad=\"%u\" rate=\"%4.2f\" dev=\"%s\" mirror=\"%s\"/>\n",
//           msg->tag.getTagName(), n,
//           bool2str(sick->isRunning()),
//           sick->getScanAngle(),
//           double(sick->getScanResolution()),
//           sick->getGood(), sick->getBad(), sick->getMsgRate(),
//           sick->getDeviceName(),
//           bool2str(sick->getMirror()));
//       sendMsg(msg, reply);
//     }
//     else if (getPose)
//     { // send start tag
//       snprintf(reply, MRL, "<%s device=%d>\n", msg->tag.getTagName(), n);
//       sendMsg(msg, reply);
//       // send pose
//       pr = sick->getDevicePose();
//       pos = pr->getPos();
//       rot = pr->getRot();
//       tag.codePosition(pos, reply, MRL, "device");
//       sendMsg(msg, reply);
//       tag.codeRotation(rot, reply, MRL, "device");
//       sendMsg(msg, reply);
//       // send end tag
//       snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
//       sendMsg(msg, reply);
//     }
//     else
//     { // send just device number - no specific data requested
//       snprintf(reply, MRL, "<%s device=%d/>\n", msg->tag.getTagName(), n);
//       sendMsg(msg, reply);
//     }
//   }
//   else
//     sendWarning(msg, "No scanner device");
//
//   return result;
// }

///////////////////////////////////////////////////////

// void UFunctionLaser::setDevicePool(ULaserPool * pool, bool remove)
// {
//   if (remove)
//     lasPool = NULL;
//   else
//     lasPool = pool;
// }


