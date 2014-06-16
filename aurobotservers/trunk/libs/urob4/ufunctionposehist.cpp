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

#include "usmltag.h"

#include "ufunctionposehist.h"

UFunctionPoseHist::UFunctionPoseHist()
 : UFunctionBase()
{
  setCommand("odoPose odoPosePush", "posehist", "poseHist (2.1513 " __DATE__ " jca@oersted.dtu.dk)");
  poseHist = NULL;
  poseHistLocal = false;
}

/////////////////////////////////////////////////////////

UFunctionPoseHist::~UFunctionPoseHist()
{
  if ((poseHist != NULL) and (poseHistLocal))
    delete poseHist;
}

/////////////////////////////////////////////////////////
// const char * UFunctionPoseHist::name()
// {
//    return "poseHist (2.1514 " __DATE__ " jca@oersted.dtu.dk)";
// }

/////////////////////////////////////////////////////////

void UFunctionPoseHist::setAliasName(const char * name)
{ // default alias is odometry pose hist (odoPose)
  const char * defAlias = UResPoseHist::getOdoPoseID();
  const char * a = name;
  //
  if (strlen(a) == 0)
    a = defAlias;
  UFunctionBase::setAliasName(a);
  strncpy(aliasResList, a, MAX_RESOURCE_LIST_SIZE);
  strncpy(aliasName, a, MAX_ID_LENGTH);
  snprintf(aliasNamePush, MAX_ID_LENGTH, "%sPush", a);
  snprintf(aliasCommandList, 2 * MAX_ID_LENGTH, "%s %s", aliasName, aliasNamePush);
}

/////////////////////////////////////////////////////////

void UFunctionPoseHist::createResources()
{
  if (poseHist == NULL)
  { // no cam pool loaded - create one now
    poseHist = new UResPoseHist();
    if (poseHist != NULL)
    { // mark as locally created (to delete when appropriate)
      poseHist->setResID(aliasName);
      poseHistLocal = true;
      addResource(poseHist, this);
    }
  }  
}

// const char * UFunctionPoseHist::commandList()
// {
//   return aliasCommandList;
// }

/////////////////////////////////////////////////////////

// const char * UFunctionPoseHist::resourceList()
// {
//   return aliasResList;
// }

//////////////////////////////////////////////////////////////////

bool UFunctionPoseHist::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(aliasName))
  {
    if (poseHistLocal)
      // do not set locally owned resources
      result = false;
    else if (remove)
      // resource module unloaded
      poseHist = NULL;
    else if (poseHist != (UResPoseHist *)resource)
      // resource changed or new
      poseHist = (UResPoseHist *)resource;
    else
      // no change - i.e. not used
      result = false;
  }
  else
    // may be needed by base class (or not)
    result = UFunctionBase::setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////////////////

// bool UFunctionPoseHist::gotAllResources(char * missingThese, int missingTheseCnt)
// {
//   bool result;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   result = (poseHist != NULL);
//   if ((not result) and (p1 != NULL))
//   {
//     snprintf(p1, missingTheseCnt, " %s", aliasName);
//     n = strlen(missingThese);
//     p1 = &missingThese[n];
//   }
//   if (result)
//   {
//     result = poseHist->gotAllResources(p1, missingTheseCnt - n);
//     if (not result and (p1 != NULL))
//     n += strlen(p1);
//     p1 = &missingThese[n];
//   }
//   result &= UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result;
// }

////////////////////////////////////////////////////

// UResBase * UFunctionPoseHist::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, aliasName) == 0)
//   {
//     if (poseHist == NULL)
//     { // no cam pool loaded - create one now
//       poseHist = new UResPoseHist();
//       if (poseHist != NULL)
//       { // mark as locally created (to delete when appropriate)
//         poseHist->setResID(aliasName);
//         poseHistLocal = true;
//       }
//     }
//     result = poseHist;
//   }
//   if (result == NULL)
//     // may need a diffrerene resource
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

bool UFunctionPoseHist::handleCommand(UServerInMsg * msg, void * extra)
{ // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA(aliasName))
    result = handlePoseHistCommand(msg);
  else if (msg->tag.isTagA(aliasNamePush))
    result = handlePoseHistPush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionPoseHist::handlePoseHistCommand(UServerInMsg * msg)
{
  bool result = true;
  char att[MAX_SML_NAME_LENGTH];
  const int VBL = 100;
  char val[VBL];
  char tstr[30];
  char tstrd[30];
  bool ask4help = false;
  bool aPose = false;
  bool aPoseDist = false;
  bool aPoseTime = false;
  bool aPoseOld = false;
  bool aMinDist = false;
  bool aMinTheta = false;
  bool aMinTime = false;
  bool aAdd = false;
  bool aReplay = false; // on off
  bool aReplayTime = false; // get or set time
//  bool aReplayDir = false; // get or set subdir
  bool aReplayStep = false; // step one entry in logfile
  bool aSource = false;
  bool aReplayTime2 = false;
  bool replayTime2 = false;
  int source = -1;
  bool replay = false;
  bool replayOpen = false;
  int replaySteps = 1;
//  char replaySubdir[VBL];
  UTime replayTimeLast, replayTimeNext;
  double poseDist = 0.0;
  double poseTime = 0.0;
  double minDistVal = 0.0;
  double minThetaVal = 0.0;
  double minTimeVal = 0.0;
  UPoseTime pt, pt1, pt2;
  UPoseTVQ ptvq;
  UPose po;
  double dt;
  bool ignoresOptions = false;
  const int MIL = 100;
  char ignores[MIL] = "";
  const int MSL = 500;
  char s[MSL];
  int n, posesCnt = 0;
  const int MRL = 600;
  char reply[MRL];
  USmlTag tag;
  bool replyOK = false;
  bool isOK;
  UTime t;
//  const char * replayDir = NULL;
  bool aLogOpen = false;
  bool logOpen = false;
  const char * logName = NULL;
  bool aKml = false;
  char aKmlName[MRL] = "utmPose";
  bool aList = false;
  int aListCnt = 10;
  //
  ptvq.t.Now();
  while (msg->tag.getNextAttribute(att, val, VBL))
  {
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "pose") == 0)
      aPose = true;
    else if (strcasecmp(att, "poseOldest") == 0)
      aPoseOld = true;
    else if (strcasecmp(att, "poseAtDist") == 0)
    {
      aPoseDist = true;
      poseDist = strtod(val, NULL);
    }
    else if (strcasecmp(att, "poseAtTime") == 0)
    {
      aPoseTime = true;
      poseTime = strtod(val, NULL);
    }
    else if (strcasecmp(att, "pose") == 0)
      aPose = true;
    else if (strcasecmp(att, "add") == 0)
    { // add default time to pose
      aAdd = true;
    }
    else if (strcasecmp(att, "minDist") == 0)
    { // add default time to pose
      aMinDist = true;
      minDistVal = strtod(val, NULL);;
    }
    else if (strcasecmp(att, "minTheta") == 0)
    { // add default time to pose
      aMinTheta = true;
      minThetaVal = strtod(val, NULL);;
    }
    else if (strcasecmp(att, "minTime") == 0)
    { // add default time to pose
      aMinTime = true;
      minTimeVal = strtod(val, NULL);;
    }
    else if (strcasecmp(att, "x") == 0)
    {
      ptvq.x = strtod(val, NULL);
      aAdd = true;
    }
    else if (strcasecmp(att, "y") == 0)
    {
      ptvq.y = strtod(val, NULL);
      aAdd = true;
    }
    else if ((strcasecmp(att, "th") == 0) or
             (strcasecmp(att, "h") == 0))
    {
      ptvq.h = strtod(val, NULL);
      aAdd = true;
    }
    else if (strcasecmp(att, "vel") == 0)
      ptvq.vel = strtod(val, NULL);
    else if (strcasecmp(att, "q") == 0)
      ptvq.q = strtod(val, NULL);
    else if (strcasecmp(att, "tod") == 0)
      ptvq.t.setTime(strtod(val, NULL));
    else if (strcasecmp(att, "replay") == 0)
    {
      aReplay = true;
      if (strlen(val) == 0)
        replay = true;
      else
        replay = str2bool(val);
    }
/*    else if (strcasecmp(att, "replaySubDir") == 0)
    {
      aReplayDir = true;
      strcpy(replaySubdir, val);
    }*/
    else if (strcasecmp(att, "replayTime") == 0)
    {
      aReplayTime = true;
      replayTimeNext.setTimeTod(val);
    }
    else if (strcasecmp(att, "replayTime2") == 0)
    {
      aReplayTime2 = true;
      replayTime2 = str2bool2(val, true);
    }
    else if (strcasecmp(att, "step") == 0)
    {
      aReplayStep = true;
      if (strlen(val) > 0)
        replaySteps = strtol(val, NULL, 0);
    }
    else if (strcasecmp(att, "log") == 0)
    {
      aLogOpen = true;
      logOpen = str2bool2(val, true);
    }
    else if (strcasecmp(att, "source") == 0)
      aSource = true;
    else if (strcasecmp(att, "tokml") == 0)
    {
      aKml = true;
      if (strlen(val) > 0)
        strncpy(aKmlName, val, MRL);
    }
    else if (strcasecmp(att, "list") == 0)
    {
      aList = true;
      if (strlen(val) > 0)
        aListCnt = strtol(val, NULL, 0);
    }
    else
    {
      ignoresOptions = true;
      n = strlen(ignores);
      if ((n > 0) and (n < MIL))
        ignores[n++] = ' ';
      strncpy(&ignores[n], att, MIL - n);
    }
  }
  //
  if (ask4help)
  {
    minDistVal = 0.03;
    minThetaVal = M_PI/180.0;
    minTimeVal = 10.0;
    if (poseHist != NULL)
    {
      minDistVal = poseHist->getMinDist();
      minThetaVal = poseHist->getMinTheta();
      minTimeVal = poseHist->getMinTime();
      replay = poseHist->isReplay();
//      replayDir = poseHist->getReplayPath();
      replayTimeLast = poseHist->getReplayTimeNow();
      replayTimeNext = poseHist->getReplayTimeNext();
      replayOpen = poseHist->isReplayFileOpen();
      posesCnt = poseHist->getPosesCnt();
      pt1 = poseHist->getNewest(NULL);
      pt1.t.getTimeAsString(tstr, true);
      pt1.t.getDateString(tstrd, true);
      source = poseHist->getNewestSource();
      pt2 = poseHist->getOldest();
      pt2.t.getTimeAsString(val, true);
      logOpen = poseHist->isLogfileOpen();
      logName = poseHist->getLogfileName();
      replayTime2 = poseHist->getReplayTimestamp2();
    }
    snprintf(reply, MRL, "<help subject='%s'>\n", aliasName);
    sendMsg(msg, reply);
    snprintf(reply, MRL, "---------- Options to %s command\n", aliasName);
    sendText(msg, reply);
    snprintf(reply, MRL, "help               This help list (currently %d history entries)\n", posesCnt);
    sendText( msg, reply);
    snprintf(reply, MRL, "source             Source of most recent update (is %d from %.3f sec ago)\n",
             source, pt1.t.getTimePassed());
    sendText( msg, reply);

    snprintf(reply, MRL, "pose               Get newest pose and time (%.2fx %.2fy %.2frad at %s %s)\n",
            pt1.x, pt1.y, pt1.h, tstr, tstrd);
    sendText( msg, reply);
    sendText( msg,       "poseAtDist=meter   Get pose at this distance (in meter)\n");
    sendText( msg,       "poseAtTime=sec     Get pose this number of seconds back from newest\n");
    snprintf(reply, MRL, "poseOldest         Get oldest pose in store (%.2fx,%.2fy at %s)\n",
            pt2.x, pt2.y, val);
    sendText( msg, reply);
    sendText( msg,       "list=N             List N poses from saved history (first is newest)\n");
    snprintf(reply, MRL, "minDist[=value]    Get/set minimum distance change"
                         " to be history (is %gm)\n", minDistVal);
    sendText( msg, reply);
    snprintf(reply, MRL, "minTheta[=value]   Get/set minimum heading change "
                         " to be history (is %grad)\n", minThetaVal);
    sendText( msg, reply);
    snprintf(reply, MRL, "minTime[=value]    Get/set minimum time change"
                         " to be history (is %gs)\n", minTimeVal);
    sendText( msg, reply);
    sendText( msg,       "x=m y=m th=rad [tod=ssss.mmm] [val=m/s] [qual=q] Add a pose history value manually\n");
    snprintf(reply, MRL, "log[=false]        Open or close logfile (%s) open=%s)\n",
             logName, bool2str(logOpen));
    sendText( msg, reply);
    snprintf(reply, MRL, "replay[=false]     Start or stop replay mode (replay=%s fileOpen=%s) file in replayPath\n",
             bool2str(replay), bool2str(replayOpen));
    sendText( msg, reply);
/*    snprintf(reply, MRL, "replaySubdir='dir' Set subdir to %s.log file (is '%s') from log path\n",
             aliasName, replayDir);
    sendText( msg, reply);*/
/*    snprintf(reply, MRL, "                   log path is 'dataPath' (is %s)\n", dataPath);
    sendText( msg, reply);*/
    sendText( msg,      "step=N             Replay N entries from replay logfile\n");
    snprintf(reply, MRL,"replayTime[=tod]   Get or set replay to tod (now %lu.%06lu)\n",
             replayTimeLast.getSec(), replayTimeLast.getMicrosec());
    sendText( msg, reply);
    snprintf(reply, MRL,"replayTime2[=false] Use second timestamp in logfile (is %s)\n",
             bool2str(replayTime2));
    sendText( msg, reply);
    if (strcmp(aliasName, "utmPose") == 0)
      // offer to convert to kml for google earth 
      sendText( msg,      "toKml[=utmPose]    Konvert utm logfile (utmPose) to kml file for google earth\n");
    sendText( msg, "-------\n");
    sendText( msg, "see also 'server help' for path settings\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (ignoresOptions)
  {
    snprintf(reply, MRL, "Unknown option: %s", ignores);
    sendWarning(msg, reply);
  }
  else if (not gotAllResources(s, MIL))
  { //
    snprintf(reply, MRL, "No resources, missing %s", s);
    sendWarning(msg, reply);
  }
  else
  {
    if (aLogOpen)
    {
      if (logOpen)
      {
        logOpen = poseHist->openLogfile();
        snprintf(reply, MRL, "Open logfile (%s) filename %s",
                 bool2str(logOpen), poseHist->getLogfileName());
        sendInfo(msg, reply);
      }
      else
      {
        poseHist->closeLogfile();
        sendInfo(msg, "logfile closed");
      }
      replyOK = true;
    }
    if (aPose or aPoseDist or aPoseTime or aPoseOld)
    { // get newest pose
      // code and send open tag
      snprintf(reply, MRL, "<%s name=\"poses\">\n", msg->tag.getTagName());
      sendMsg(msg, reply);
      // code and send pose
      if (aPose)
      {
        ptvq = poseHist->getNewest();
        snprintf(s, MSL, "vel=\"%.3f\" q=\"%g\"", ptvq.vel, ptvq.q);
        tag.codePoseTime1(ptvq, reply, MRL, "newest", s);
        sendMsg(msg, reply);
      }
      if (aPoseDist)
      {
        ptvq = poseHist->getNewest();
        po = ptvq.getPose();
        isOK = poseHist->getPoseNearDistance(poseDist, &po, &ptvq, NULL);
        snprintf(s, MSL, "dist=\"%g\" valid=\"%s\"", poseDist, bool2str(isOK));
        tag.codePoseTime1(ptvq, reply, MRL, "poseDist", s);
        sendMsg(msg, reply);
      }
      if (aPoseTime)
      {
        ptvq = poseHist->getNewest();
        t = ptvq.t - poseTime;
        isOK = poseHist->getPoseNearTime(t, &ptvq, NULL);
        dt = ptvq.t - pt.t;
        snprintf(s, MSL, "time=\"%g\" valid=\"%s\"", dt, bool2str(isOK));
        tag.codePoseTime1(ptvq, reply, MRL, "poseAtTime", s);
        sendMsg(msg, reply);
      }
      if (aPoseOld)
      {
        pt = poseHist->getOldest();
        tag.codePoseTime1(pt, reply, MRL, "oldest");
        sendMsg(msg, reply);
      }
      // code and send close tag
      snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
      sendMsg(msg, reply);
      replyOK = true;
    }
    /*bool aMinDist = false;
    bool aMinTheta = false;
    bool aMinTime = false;
    bool aAdd = false;*/
    if (aMinDist or aMinTheta or aMinTime)
    {
      if (minDistVal > 1e-10)
        poseHist->setMinDist(minDistVal);
      if (minThetaVal > 1e-10)
        poseHist->setMinTheta(minThetaVal);
      if (minTimeVal > 1e-10)
        poseHist->setMinTime(minTimeVal);
      snprintf(reply, MRL, "<%s name=\"limits\" minDist=\"%g\" minTheta= \"%g\" minTime=\"%g\"/>\n",
               msg->tag.getTagName(), poseHist->getMinDist(),
               poseHist->getMinTheta(), poseHist->getMinTime());
      sendMsg(msg, reply);
      replyOK = true;
    }
    if (aAdd)
    { // add a pose hist value
      isOK = poseHist->addIfNeeded(ptvq, msg->client);
      snprintf(reply, MRL, "added %s", bool2str(isOK));
      sendInfo(msg, reply);
      replyOK = true;
    }
    if (aSource)
    {
      pt1 = poseHist->getNewest(NULL);
      pt1.t.getTimeAsString(tstr, true);
      source = poseHist->getNewestSource();
      if (source >= 0)
        strncpy(val, "(see 'server clients' for detail)", VBL);
      else switch (source)
      {
        case -1: strncpy(val, "server console", VBL); break;
        case -2: strncpy(val, "relay (server)", VBL); break;
        case -3: strncpy(val, "replay (pose hist)", VBL); break;
        case -4: strncpy(val, "smr stream", VBL); break;
        case -6: strncpy(val, "laser (scan data)", VBL); break;
        case -7: strncpy(val, "laser (sf)", VBL); break;
        case -8: strncpy(val, "gps Lat-Long", VBL); break;
        case -9: strncpy(val, "gps UTM", VBL); break;
        default: strncpy(val, "unknown", VBL); break;
      }
      snprintf(reply, MRL, "pose source %d is %s from %s", source, val, tstr);
      sendInfo(msg, reply);
      replyOK = true;
    }
    if (aReplay)
    {
      result = poseHist->setReplay( replay);
      if (result)
      {
        t = poseHist->getReplayTimeNow();
        t.getTimeAsString(val, VBL);
        snprintf(reply, MRL, "<%s fileopen=\"%s\" tod=\"%lu.%06lu\" "
            "time=\"%s\" logLine=\"%d\"/>\n",
            msg->tag.getTagName(), bool2str(poseHist->isReplayFileOpen()),
            t.getSec(), t.getMicrosec(), val, poseHist->getReplayLogLine());
        sendMsg( msg, reply);
      }
      else
      {
        poseHist->getReplayFileName(s, MSL);
        snprintf(reply, MRL, "No file or no data in file: %s", s);
        sendHelp(msg, reply);
      }
      replyOK = true;
    }
    if (aReplayStep)
    {
      poseHist->replayStep(replaySteps);
      if (not poseHist->isReplayFileOpen())
      {
        snprintf(reply, MRL, "replay file not found (at %s)", poseHist->getReplayFileName(s, MSL));
        sendWarning(reply);
      }
      else
      {
        t = poseHist->getReplayTimeNow();
        t.getTimeAsString(val, VBL);
        snprintf(reply, MRL, "<%s tod=\"%lu.%06lu\" time=\"%s\" logLine=\"%d\"/>\n",
                msg->tag.getTagName(), t.getSec(), t.getMicrosec(), val, poseHist->getReplayLogLine());
        sendMsg( msg, reply);
      }
      replyOK = true;
    }
    if (aReplayTime)
    {
      if (replayTimeNext.valid)
        poseHist->replayToTime(replayTimeNext);
      t = poseHist->getReplayTimeNow();
      t.getTimeAsString(val, VBL);
      snprintf(reply, MRL, "<%s tod=\"%lu.%06lu\" time=\"%s\" logLine=\"%d\"/>\n",
               msg->tag.getTagName(), t.getSec(), t.getMicrosec(), val, poseHist->getReplayLogLine());
      sendMsg( msg, reply);
      replyOK = true;
    }
    if (aReplayTime2)
    {
      poseHist->setReplayTimestamp2(replayTime2);
      sendInfo( msg, "done");
      replyOK = true;
    }
    if (aKml)
    {
      poseHist->toKml(aKmlName, MRL);
      replyOK = sendInfo(msg, aKmlName);
    }
    if (aList)
    {
      replyOK = listPoses(aListCnt);
    }
    if (not replyOK)
    { // send a reply
      sendInfo(msg, "done");
    }
  }
  //
  return result;
}

/////////////////////////////////////////////

bool UFunctionPoseHist::listPoses(int cnt)
{
  int i, n;
  UPoseTVQ pose;
  const int MSL = 200;
  char s[MSL];
  //
  n = poseHist->getPosesCnt();
  if (cnt < n)
    n = cnt;
  snprintf(s, MSL, "Listing %d poses of %d available in %s", n,
           poseHist->getPosesCnt(), aliasName);
  sendHelpStart(s);
  if (n <= 0)
    sendText("No poses\n");
  else
  {
    pose = poseHist->getNewest();
    sendText("time  x  y  h  vel  q\n");
    snprintf(s, MSL, "%lu.%06lu %6.3f %6.3f %8.5f %5.2f %g\n",
             pose.t.getSec(), pose.t.getMicrosec(), pose.x, pose.y, pose.h, pose.vel, pose.q);
    sendText(s);
    for (i = 0; i < n; i++)
    {
      pose = poseHist->getPose(i);
      snprintf(s, MSL, "%lu.%06lu %6.3f %6.3f %8.5f %5.2f %g\n",
               pose.t.getSec(), pose.t.getMicrosec(), pose.x, pose.y, pose.h, pose.vel, pose.q);
      sendText(s);
    }
  }
  sendHelpDone();
  return true;
}

////////////////////////////////////////////////////

bool UFunctionPoseHist::handlePoseHistPush(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4List = false;
  const int MRL = 1000;
  char reply[MRL];
  int cm, ca;
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4List = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  {
    snprintf(reply, MRL, "<help subject=\"%s\">\n", aliasNamePush);
    sendMsg(msg, reply);
    snprintf(reply, MRL, "----------------Available %s options:\n", aliasNamePush);
    sendText(msg, reply);
    sendText(msg, "flush=cmd          Remove 'cmd' command(s) defined by this client (default all)\n");
    sendText(msg, "total=k or n=k     Stop after k push events (def: no stop)\n");
    sendText(msg, "interval=k or i=k  Push with this interval (in update count, def = 1)\n");
    sendText(msg, "cmd=\"cmd\"          'cmd' is the command to put on command queue on push event\n");
    sendText(msg, "list               List all defined commands (as help text)\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (poseHist != NULL)
  {
    if (ask4List)
    {
      sendMsg(msg, "<help subject=\"Push command list\">\n");
      poseHist->UServerPush::print(aliasName, reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    { // push or flush command
      result = poseHist->addPushCommand(msg);
      poseHist->getPushCmdCnt(&cm, &ca);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d cmds and %d calls",
                 cm, ca);
        sendInfo(msg, val);
      }
      else
        sendWarning(msg, "push command failed");
    }
  }
  else
    sendWarning(msg, "No interface resource");

  return result;
}
