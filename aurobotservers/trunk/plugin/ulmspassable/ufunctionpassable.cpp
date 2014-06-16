/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include "ufunctionpassable.h"

#include <ugen4/uline.h>
#include <urob4/usmltag.h>
#include <urob4/uresposehist.h>

#ifdef LIBRARY_OPEN_NEEDED

/////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFunctionPassable' with your classname, as used in the headerfile */
  return new UFunctionPassable();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionPassable::UFunctionPassable()
{ // initialization of variables in class - as needed
  int n;
  //
  setCommand("pass obst road", "funcpass", "pass (v2.1514)(" __DATE__ " " __TIME__ " by Christian Andersen)");
  pass = NULL;       // initially the resource is not created
  //passLocal = false; // ... and is thus not local
  lasPool = NULL;
  odoHist = NULL;
  obsts = NULL;
  obstsLocal = false;
  roads = NULL;
  //roadsLocal = false;
  for (n = 0; n < MAX_LASER_DEVS; n++)
    lastSerial[n] = 0;
}

///////////////////////////////////////////////////

UFunctionPassable::~UFunctionPassable()
{ // possibly remove allocated variables here - if needed
  if (pass != NULL)
    delete pass;
  if (obsts != NULL and obstsLocal)
    delete obsts;
  if (roads != NULL)
    delete roads;
}

///////////////////////////////////////////////////

// const char * UFunctionPassable::name()
// {
//   return "pass (v1.72)(" __DATE__ " " __TIME__ " by Christian Andersen)";
// }

///////////////////////////////////////////////////

// const char * UFunctionPassable::commandList()
// { // space separated list og command keywords handled by this plugin
//   return "pass obst road";
// }

///////////////////////////////////////////////////

bool UFunctionPassable::setResource(UResBase * resource, bool remove)
{ // load resource as provided by the server (or other plugins)
  bool result = true;
  // test if the provided resource is relevant
  if (resource->isA(UResObstacle::getResClassID()))
  { // pointer to server the resource that this plugin can provide too
    // but as there might be more plugins that can provide the same resource
    // use the provided
    if (obstsLocal)
      // resource is owned by this plugin, so do not change (or delete)
      result = false;
    else if (remove)
      // the resource is unloaded, so reference must be removed
      obsts = NULL;
    else if (obsts != (UResObstacle *)resource)
      // resource is new or is moved, save the new reference
      obsts = (UResObstacle *)resource;
    else
      // reference is not used
      result = false;
  }
  if (resource->isA(ULaserPool::getResClassID()))
  { // is a laser pool ressource
    if (remove)
      lasPool = NULL;
    else if (lasPool != resource)
      lasPool = (ULaserPool *) resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // is a laser pool ressource
    if (remove)
      odoHist = NULL;
    else if (odoHist != resource)
      odoHist = (UResPoseHist *) resource;
    else
      result = false;
  }
  else
    // other resource types may be needed by base function.
    result = UFunctionBase::setResource(resource, remove);
  return result;
}

///////////////////////////////////////////////////

void UFunctionPassable::createResources()
{
  if (pass == NULL)
  { // no cam pool loaded - create one now
    pass = new UResPassable();
    addResource(pass, this);
  }
  if (obsts == NULL)
  { // no obstacle handler loaded (ulmspassable) - create one now
    obsts = new UResObstacle();
    obstsLocal = true;
    addResource(obsts, this);
  }
  if (roads == NULL)
  { // no cam pool loaded - create one now
    roads = new UResRoadLine();
    addResource(roads, this);
  }
}

// UResBase * UFunctionPassable::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResPassable::getResClassID()) == 0)
//   {
//     if (pass == NULL)
//     { // no cam pool loaded - create one now
//       pass = new UResPassable();
//       if (pass != NULL)
//         // mark as locally created (to delete when appropriate)
//         passLocal = true;
//     }
//     result = pass;
//   }
//   else if (strcmp(resID, UResObstacle::getResClassID()) == 0)
//   {
//     if (obsts == NULL)
//     { // no obstacle handler loaded - create one now
//       obsts = new UResObstacle();
//       if (obsts != NULL)
//         // mark as locally created (to delete when appropriate)
//         obstsLocal = true;
//     }
//     result = obsts;
//   }
//   else if (strcmp(resID, UResRoadLine::getResClassID()) == 0)
//   {
//     if (roads == NULL)
//     { // no cam pool loaded - create one now
//       roads = new UResRoadLine();
//       if (roads != NULL)
//         // mark as locally created (to delete when appropriate)
//         roadsLocal = true;
//     }
//     result = roads;
//   }
//   else
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionPassable::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (pass != NULL);
//   if (not isOK)
//   { // missing this resource
//     if (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResPassable::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   isOK = (obsts != NULL);
//   if (not isOK)
//   { // missing this resource
//     if (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResObstacle::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   isOK = (roads != NULL);
//   if (not isOK)
//   { // missing this resource
//     if (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResRoadLine::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   isOK = (lasPool != NULL);
//   if ((not isOK) and (missingThese != NULL))
//   { // missing this resource
//     if (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt-n, " %s", ULaserPool::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   // ask if the function base is OK too
//   isOK = UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result and isOK;
// }

///////////////////////////////////////////////////

bool UFunctionPassable::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("pass"))
    result = handlePass(msg, extra);
  else if (msg->tag.isTagA("obst"))
    result = handleObstGet(msg, extra);
  else if (msg->tag.isTagA("road"))
    result = handleRoad(msg, extra);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionPassable::handlePass(UServerInMsg * msg, void * extra)
{ // do the needed processing and send a reply back to the client
  const int MRL = 1200;
  char reply[MRL];
  bool ask4help;
  bool doPose = true;
  bool doScan = true;
  bool doGetOnly = false;
  bool doMakeOnly = false;
  const int MVL = 50;
  char val[MVL];
  ULaserDevice * lasDev = NULL;
  int fake = -1;
  //bool aNew = true;
  bool anObstOnly = false;
  ULaserData * laserData = (ULaserData *)extra;
  ULaserData laserDataBuff;
  int i, n, iL, iR;
  UPassQual iQ;
  bool result;
  UTime t;
  USmlTag tag;
  ULaserPi * pis;
  const int MSL = 100;
  char s[MSL];
  UPose pose;
  bool outdoor = true;
  double obstExt = 1.0;
  unsigned long * last = NULL;
  ULaserScan * piScan;
  //
  if (lasPool != NULL)
  { // get default device
    lasDev = lasPool->getDefDevice();
    // get last serial number
    last = &lastSerial[lasPool->getDefDeviceNumber()];
  }
  //
  // check for attributes (command parameters)
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    //aNew = msg->tag.getAttValue("new", val, MVL);
    anObstOnly = msg->tag.getAttValue("obstonly", val, MVL);
    if (msg->tag.getAttValue("fake", val, MVL))
      fake = strtol(val, NULL, 0);
    if (msg->tag.getAttValue("any", val, MVL))
      *last = 0;
    if (msg->tag.getAttValue("pose", val, MVL))
    {
      if (strlen(val) > 0)
        doPose = str2bool(val);
      else
        doPose = true;
    }
    if (msg->tag.getAttValue("getOnly", val, MVL))
    {
      if (strlen(val) > 0)
        doGetOnly = str2bool(val);
      else
        doGetOnly = true;
    }
    if (msg->tag.getAttValue("makeOnly", val, MVL))
    {
      if (strlen(val) > 0)
        doMakeOnly = str2bool(val);
      else
        doMakeOnly = true;
    }
    if (msg->tag.getAttValue("scan", val, MVL))
    {
      if (strlen(val) > 0)
        doScan = str2bool(val);
      else
        doScan = true;
    }
    if (msg->tag.getAttValue("device", val, MVL))
    {
      n = strtol(val, NULL, 0); // device
      if (n < 0)
        n = 0;
      if (lasPool != NULL)
      { // get new device (do not make it default)
        // lasPool->setDefDevice(n);
        lasDev = lasPool->getDevice(n);
      }
      // get last serial number
      last = &lastSerial[n];
    }
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendMsg( msg, "<help subject=\"PASS\">\n");
    sendText( msg, "--- available PASS options\n");
    sendText( msg, "Analyze a new scan for passable segments and obstacles\n");
    sendText( msg, "obstOnly     Analize for obstacles only (else road and obstacles)\n");
    sendText( msg, "pose[=false] add pose to result\n");
    sendText( msg, "fake=N       Use fake data (4 uses tilted device)\n");
    sendText( msg, "device=N     Get data from this device (else default device)\n");
    sendText( msg, "any          Use any available scan (ignore scan number)\n");
    sendText( msg, "getOnly      Get current passable interval result\n");
    sendText( msg, "makeOnly     Make passable interval - but do not send results\n");
    sendText( msg, "scan         add analysis of scan to result (extended scanget)\n");
    sendText( msg, "help         This message\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if ((pass == NULL) or (lasPool == NULL) or (lasDev == NULL))
    sendWarning(msg, "no resource to do that (need laserPool,device and Pass) - try 'module list' for help");
  else
  { // process scan - first ensure data is available
    result = true;
    // aquire data
    if (not doGetOnly)
    {
      if (laserData == NULL)
      { // not push data
        laserData = &laserDataBuff;
        result = lasDev->getNewestData(laserData, *last, fake);
      }
      else
        // push'ed data get device pointer (for sensor position)
        lasDev = lasPool->getDevice(laserData->getDeviceNum());
      //
      if (not result or not laserData->isValid())
        sendWarning(msg, "No (raw) scandata available");
      else
      { // data is available - load to ULaserScan structure.
        *last = laserData->getSerial();
        if (obsts != NULL)
        { // obstacle configuration
          outdoor = obsts->isOutdoorContext();
          // get search extension distance for obstacles */
          obstExt = obsts->getOutdoorExtraDist();
          //obsts->getValue("obstExtDistance", &obstExt);
        }
        // get robot pose at scantime
        if (fake > 0)
          pose = laserData->getFakePose();
        else if (odoHist != NULL)
        {
          pose = odoHist->getPoseAtTime(laserData->getScanTime());
          laserData->setFakePose(pose); // save pose with scandata
        }
        else
          pose.clear();
        // find passable areas and obstacles
        if (not anObstOnly)
          result = pass->doFullAnalysis(laserData, pose, lasDev->getDevicePose(),
                                      obsts, outdoor, obstExt, roads);
        else
          result = pass->doObstAnalysis(laserData, pose, lasDev->getDevicePose(),
                                      obsts, outdoor);
        // update obstacle database with nearby mapped obstacles
        obsts->updateMappedObstacles();
      }
    }
    if (not result)
      sendWarning(msg, "Passable interval analysis failed");
    else if (doMakeOnly)
    {
      if (msg->client >= 0)
        // do not copy to server console
        sendInfo(msg, "Passable interval analysis done");
    }
    else
    {
      piScan = pass->getScan();
      result = piScan->valid;
      if (result)
      { // send result
        t = piScan->time;
        // number of found segments
        n = pass->getPisCnt();
        snprintf(reply, MRL, "<%s scan=\"%lu\" cnt=\"%d\" tod=\"%lu.%06lu\">\n",
                msg->tag.getTagName(), piScan->getSerial(), n, t.getSec(), t.getMicrosec());
        sendMsg(msg, reply);
        if (doPose)
        { // send also robot and scanner 3D pose
          tag.codePose(piScan->getRobotPose(), reply, MRL, "robot");
          sendMsg(msg, reply);
            // send also sensor pose - as a 3D position and a 3D rotation
          sendMsg(msg, tag.codePosition(lasDev->getDevicePos(), reply, MRL, "device"));
          sendMsg(msg, tag.codeRotation(lasDev->getDeviceRot(), reply, MRL, "device"));
        }
        // send passable interval lines
        piScan = pass->getScan();
        for (i = 0; i < n; i++)
        {
          pis = pass->getPi(i);
          snprintf(s, MSL, "center=\"%.3f\" centerValid=\"%s\" var=\"%.7e\" "
              "varMin=\"%.7e\"", pis->getCenter(), bool2str(pis->getCenterValid()),
                  pis->getFitVariance(), pis->getVarMin2());
          tag.codeLineSegment(pis->getSegment(), reply, MRL, "pis", s);
          sendMsg(msg, reply);
          // mark pi's in laserscan flags
          iL = pis->getLeft();
          iL = piScan->getRight(iL);
          iR = pis->getRight();
          iQ = piScan->getQ(iL);
          piScan->setQ(iL, UPassQual(iQ + 10));
          iQ = piScan->getQ(iR);
          piScan->setQ(iR, UPassQual(iQ + 20));
        }
        snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
        sendMsg(msg, reply);
        //
        if (doScan)
        { // fake reply as a scanget
          sendFullScan(msg, "scanget", lasDev, piScan);
        }
  /*      n = pass->getPisCnt();
        snprintf(reply, MRL, "Found %d passable intervals", n);
        sendInfo(msg, reply);*/
      }
      else
        sendWarning(msg, "passable interval: No (processed) scandata available");
    }
  }
  // return true if the function is handled with a positive result
  return true;
}

///////////////////////////////////////////////////////////////


bool UFunctionPassable::sendFullScan(UServerInMsg * msg, const char * tagName,
//                                  ULaserData * laserData,
                                  ULaserDevice * sick,
                                  ULaserScan * scan)
{
  const int RL = 400;
  char reply[RL];
  int i;//, n;
  ULaserPoint * lp;
  bool result;
  UPosition pLeft, pRight, ptop, pls, prs;
  UTime t;
  USmlTag tag;
  double angleRes;
  double angleStart, angleEnd;
  UPosition p1, p2;
  //
  p1 = scan->data[0].pos;
  p2 = scan->data[scan->dataCnt - 1].pos;
  angleStart = atan2(p1.y , p1.x)  * 180.0 / M_PI;
  angleEnd = atan2(p2.y , p2.x)  * 180.0 / M_PI;
  angleRes = fabs(angleEnd - angleStart)/double(scan->dataCnt - 1);
  t = scan->time;
  snprintf(reply, RL, "<%s serial=\"%lu\" interval=\"%g\" "
      "count=\"%d\" tod=\"%ld.%06ld\" "
          "min=\"%4.2f\" max=\"%4.2f\" statValid=\"true\" codex=\"tag\">\n",
      tagName, scan->getSerial(),
      angleRes, scan->getDataCnt(),
      t.GetSec(), t.getMicrosec(),
      angleStart, angleEnd);
  result = sendMsg(msg, reply);
  //
  if (result)
  { // send also robot and sensor pose
  sendMsg(msg, tag.codePose(scan->getRobotPose(), reply, RL, "robot"));
      // sensor position (x,y,z) relative to robot origin
  sendMsg(msg, tag.codePosition(sick->getDevicePos(), reply, RL, "device"));
      // sensor roattion (Omega, Phi, Kappa) relative to robot
  sendMsg(msg, tag.codeRotation(sick->getDeviceRot(), reply, RL, "device"));
  }
  //
  if (result and (scan->dataCnt > 0))
  { // send data as one binary pack
    //n = 0;
    lp = scan->data;
    for (i = 0; i < scan->dataCnt; i++)
    { // send count number of messages - decode values
      snprintf(reply, RL, "<lval zz=%d ang=%.2f dist=%1.3f"
          /*" var=%.5f */ " varL=%.5f tilt=%.3f/>\n",
               lp->passQ,
               angleStart + double(i)* angleRes,
               lp->range, /*lp->var,*/ lp->varL, lp->tilt);
      //
      result = sendMsg(msg, reply);
      if (not result)
        break;
      lp++;
    }
  }
  snprintf(reply, RL, "</%s>\n", tagName);
  result = sendMsg(msg, reply);
  return result;
}


////////////////////////////////////////////////////////////


const char * UFunctionPassable::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s uses resources: pass, obsts, laserPool and poseHist", preString);
  return buff;
}

////////////////////////////////////////////////////////////

bool UFunctionPassable::handleObstGet(UServerInMsg * msg, void * extra)
{ // get parameters
  bool result = false;
  char att[MAX_SML_NAME_LENGTH];
  const int MVL = 50;
  char val[MVL];
  const int MRL = 500;
  char reply[MRL];
  bool ask4help = false;
  bool updatesOnly = false;
  int cnt = 3;
  bool anAdd = false;
  bool anAddRel = false;
  bool aClearAll = false;
  bool aClearGrp = false;
  bool aFixedToo = false;
  int aClearGrpIsx = 0;
  bool aGetFromMap = false;
  bool aMakeOnly = false;
  bool polySetAll = false;
  bool polyUpdates = false;
  UPosition pos1, pos2, pos3, pos4;
  UPoseTime poseT;
  UObstacleGroupLaser * og;
  UPolygon40 poly1, poly2;
  double mergeDist = 0.1;
  //USmlTag tag;
  //
  while (msg->tag.getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "max") == 0)
      cnt = strtol(val, NULL, 10);
    else if (strcasecmp(att, "update") == 0)
      updatesOnly = str2bool2(val, true);
    else if (strcasecmp(att, "polyUpdate") == 0)
      polyUpdates = str2bool2(val, true);
    else if (strcasecmp(att, "polySet") == 0)
      polySetAll = str2bool2(val, true);
    else if (strcasecmp(att, "fixed") == 0)
      aFixedToo = str2bool2(val, true);
    else if (strcasecmp(att, "add") == 0)
      anAdd=true;
    else if (strcasecmp(att, "addRel") == 0)
      anAddRel=true;
    else if (strcasecmp(att, "x") == 0)
      pos1.x = strtod(val, NULL);
    else if (strcasecmp(att, "x1") == 0)
      pos1.x = strtod(val, NULL);
    else if (strcasecmp(att, "x2") == 0)
      pos2.x = strtod(val, NULL);
    else if (strcasecmp(att, "y") == 0)
      pos1.y = strtod(val, NULL);
    else if (strcasecmp(att, "y1") == 0)
      pos1.y = strtod(val, NULL);
    else if (strcasecmp(att, "y2") == 0)
      pos2.y = strtod(val, NULL);
    else if (strcasecmp(att, "x3") == 0)
      pos3.x = strtod(val, NULL);
    else if (strcasecmp(att, "y3") == 0)
      pos3.y = strtod(val, NULL);
    else if (strcasecmp(att, "x4") == 0)
      pos4.x = strtod(val, NULL);
    else if (strcasecmp(att, "y4") == 0)
      pos4.y = strtod(val, NULL);
    else if (strcasecmp(att, "merge") == 0)
      mergeDist = strtod(val, NULL);
    else if (strcasecmp(att, "clearAll") == 0)
      aClearAll = true;
    else if (strcasecmp(att, "clearGrp") == 0)
    {
      aClearGrp = true;
      if (strlen(val) > 0)
        aClearGrpIsx = strtol(val, NULL, 0);
    }
    else if (strcasecmp(att, "getFromMap") == 0)
      aGetFromMap = true;
    else if (strcasecmp(att, "makeOnly") == 0)
      aMakeOnly = true;
    else
    {
      snprintf(reply, MRL, "attribute '%s=\"%s\"' not used (try help)", att, val);
      sendWarning(msg, reply);
    }
    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendHelpStart("OBST command");
    sendText("Obstacle management and obstacle group request\n");
    sendText("--- Available OBST options:\n");
    sendText("max=N          : Get the most recent N groups (def = 3)\n");
    sendText("update[=false] : Get latest updates only (default = false)\n");
    sendText("polySet        : update polygon plug-in with all obstacles - in max groups\n");
    sendText("polyUpdate     : update polygon plug-in with latest obstacles only\n");
    sendText("getFromMap     : Get (near) fixed obstacles from map into obstacle list\n");
    sendText("fixed[=false]  : Get also near fixed obstacles from mapbase\n");
    sendText("add x=A y=A [x2=B y2=B] [x3=C y3=C] [x4=D y4=D] [merge=dist] (odo coordinates)\n");
    sendText("addrel x=A y=A ... Add obstacle as add, coordinates relative to robot\n");
    sendText("      Add an obstacle area ABCD and merge with other obstacles\n");
    sendText("makeOnly       : Do not send any obstacles in reply\n");
    sendText("clearAll       : Remove all detected obstacles (all obstacle groups)\n");
    sendText("clearGrp=N     : Remove obstacle group N (n=0 is the newest (and default))\n");
    sendText("help           : Send this text\n");
    sendText("---\n");
    sendText("See also commands: pass road scanget\n");
    sendText("Obstacle detection parameters see: 'var obst'\n");
    sendHelpDone();
    result = true;
  }
  else
  {
    if (anAdd or anAddRel)
    {
      if (pos1.dist() < 0.01)
        sendWarning("The (first) obstacle position must not be 0.0");
      else if (odoHist == NULL)
        sendWarning("Needs odometry history resource to add obstacles");
      else
      { // get current pose and newest obstacle group
        poseT = odoHist->getNewest(NULL);
        og = obsts->getObstGrp(poseT);
        if (og != NULL)
        { // clear coordinate conversion if not relative to current pose
          if (anAdd)
            poseT.clear();
          // add points if not zero in odometry coordinates
          poly1.add(poseT.getPoseToMap(pos1));
          if (pos2.dist() > 0.1)
            poly1.add(poseT.getPoseToMap(pos2));
          if (pos3.dist() > 0.1)
            poly1.add(poseT.getPoseToMap(pos3));
          if (pos4.dist() > 0.1)
            poly1.add(poseT.getPoseToMap(pos4));
          // arrange in CCV order
          poly1.extractConvexTo(&poly2);
          // add to obstacle group - may merge
          og->addObstPoly(&poly2, poseT, false, mergeDist, 2);
          // mark as updated
          obsts->obstDataUpdated( poseT.t);
        }
        else
          sendWarning("Failed to get obstacle group for add");
      }
    }
    if (aClearAll)
    {
      obsts->clear();
      sendInfo("removed all obstacles");
    }
    else if (aClearGrp)
    {
      obsts->clearGrp(aClearGrpIsx);
      sendInfo("removed obstacles from group");
    }
    if (aGetFromMap)
    {
      obsts->updateMappedObstacles();
      snprintf(reply, MRL, "Got %d obstacles from mapbase", obsts->getFixedObstCnt());
      sendInfo(reply);
    }
    if (polySetAll or polyUpdates)
      makePoly(cnt, polyUpdates, aFixedToo);
    if (not aMakeOnly)
      result = sendObstacles(msg, cnt, updatesOnly, aFixedToo);
  }
  return result;
}

////////////////////////////////////////////////////////////

bool UFunctionPassable::handleRoad(UServerInMsg * msg, void * extra)
{ // get parameters
  bool result = false;
  char att[MAX_SML_NAME_LENGTH];
  const int MVL = 50;
  char val[MVL];
  const int MRL = 500;
  char reply[MRL];
  bool ask4help = false;
  int cnt = 3;
  double qual = 0.0;
  bool bestOnly = false;
  //USmlTag tag;
  //
  while (msg->tag.getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "cnt") == 0)
      cnt = strtol(val, NULL, 10);
    else if (strcasecmp(att, "q") == 0)
      qual = strtod(val, NULL);
    else if (strcasecmp(att, "best") == 0)
    {
      bestOnly = strlen(val) == 0;
      if (not bestOnly)
        bestOnly = str2bool(val);
    }
    else
    {
      snprintf(reply, MRL, "attribute '%s=\"%s\"' not used (try help)", att, val);
      sendWarning(msg, reply);
    }
    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"ROAD options\">\n");
    sendText(msg, "--- Available ROAD options:\n");
    sendText(msg, "Sends the current road edge lines\n");
    sendText(msg, "cnt=N       Send lines with at leat N updates (default 3)\n");
    sendText(msg, "q=V         Send lines with at leat quality V [0..1] (default 0.0)\n");
    sendText(msg, "left[=dist] Send error distance if left line should be at dist\n");
    sendText(msg, "top[=dist] Send error distance if left line should be at dist\n");
    sendText(msg, "cent[=dist] Send error distance if left line should be at dist\n");
    sendText(msg, "help        Send this text\n");
    sendText(msg, "Parameters see: 'var road.all'\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (lasPool != NULL)
    result = sendRoadLines(msg, cnt, qual, bestOnly);
  return result;
}

//////////////////////////////////////////////////

bool UFunctionPassable::sendObstacles(UServerInMsg * msg, int maxCnt,
                                     bool updateOnly, bool andFixed)
{
  bool result = true;
  int cnt;
  int n, m, n2, nf = 0;
  const int MRL = 10000;
  char reply[MRL];
  const char * OBST_GRP_TAG = "obstGrp";
  UObstacleGroup * og;
  UObstacle * ob;
  UObstacleGroup * ogFix = NULL;
  USmlTag tag;
  UTime ut;
  //
  n = obsts->getGroupsCnt();
  cnt = mini(n, maxCnt);
    // count number of groups with obstacles
  if (cnt > 0)
  { // get newest update tiime
    og = obsts->getGroup(0);
    ut = og->getPPoseLast()->t;
  }
  n2 = 0;
  for (n = 0; n < cnt; n++)
  {
    og = obsts->getGroup(n);
    if (not updateOnly or (ut == og->getPPoseLast()->t) or og->getObstsCnt() == 0)
      n2++;
  }
  if (andFixed)
  {
    ogFix = obsts->getFixeds();
    if (ogFix!= NULL)
      if (ogFix->getObstsCnt() > 0)
        nf = 1;
  }
  snprintf(reply, MRL, "<%s cnt=\"%d\" tod=\"%lu.%06lu\">\n",
           msg->tag.getTagName(), n2 + nf, ut.getSec(), ut.getMicrosec());
  sendMsg(msg, reply);
  //
  for (n = 0; n < cnt + nf; n++)
  {
    /*
    <obstGrp name="maybe" cnt="31" update="false">
    <poset name="first">
    <pose x="0.034" y="-0.001" h="-0.062"/>
    <tod />
    <timeofday tod="12345678.123456"/>\n"
    </poset>
    <poset name="last"> ...
    </poset>
    <obst ...> ... </obst>
    <obst ...> ... </obst>
    </obstGrp> */
    if (n < cnt)
      og = obsts->getGroup(n);
    else
    {
      og = ogFix;
      updateOnly = false;
    }
    if (not updateOnly or (ut == og->getPPoseLast()->t) or og->getObstsCnt() == 0)
    { // send always, if no obstacles in group - else display will not be kept up to date
      snprintf(reply, MRL, "<%s cnt=\"%d\" serial=\"%lu\" update=\"%s\" fixed=\"%s\">\n",
               OBST_GRP_TAG, og->getObstsCnt(),
               og->getSerial(), bool2str(updateOnly), bool2str(n >= cnt));
      sendMsg(msg, reply);
          // send first pose
      tag.codePoseTime(og->getPoseFirst(), reply, MRL, "first");
      sendMsg(msg, reply);
          // send last pose
      tag.codePoseTime(og->getPoseLast(), reply, MRL, "last");
      sendMsg(msg, reply);
      // get newest update time
      ut = og->getPPoseLast()->t;
          // send obstacles
      for (m = 0; m < og->getObstsCnt(); m++)
      {
        ob = og->getObstacle(m);
        if ((not updateOnly) or (ut == ob->getPPoseLast()->t))
        { // obstacle is updated (or all obstacles are requested)
          tag.codeObstacle(og->getObstacle(m), reply, MRL, NULL);
          result = sendMsg(msg, reply);
          if (not result)
                // TX-timeout - stop here and try to send end tags
            break;
        }
        else
        { // no change to this obstacle
          snprintf(reply, MRL, "<obst serial=\"%lu\" noChange=\"true\"/>\n",
                   ob->getSerial());
          result = sendMsg(msg, reply);
        }
        // debug
        //printf("**** Send(%s) :%d obsts\n", bool2str(result), og->getObstsCnt());
        // debug end
      }
          // send end tag
      snprintf(reply, MRL, "</%s>\n", OBST_GRP_TAG);
      sendMsg(msg, reply);
    }
  }
  //
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  result = sendMsg(msg, reply);
  //
  return result;
}

///////////////////////////////////////////////////////////////////

bool UFunctionPassable::polygonToPolyPlugin(UPolygon * poly,
                                            char * name,
                                            int coordinateSystem,
                                            char color)
{
  bool isOK;
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase *db, *dbr;
  int n;
  // parameter 0 is name
  vs.setValues(name, 0, true);
  par[0] = &vs;
  // parameter 1 is polygon to add
  db = poly;
  poly->color[0] = color;
  par[1] = (UVariable *) db;
  // parameter 2 is coordinate system (0=odo, 1=utm, 2=map)
  vCoo.setDouble(coordinateSystem);
  par[2] = &vCoo;
  // number of parameters
  n = 3;
  // set return value
  dbr = &vr;
  // do the call
  isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
  if (not isOK or not vr.getBool())
    printf("UResAvoid::copyFootprintPolys: failed to 'poly.setPoly(%s,c,d)'\n", name);
  return isOK;
}

///////////////////////////////////////////////////////////////////

// bool UFunctionPassable::polygonDelete(char * name)
// { 
//   bool isOK;
//   UVariable * par[3];
//   UVariable vs;
//   UVariable vr;
//   UVariable vCoo;
//   UDataBase *db, *dbr;
//   int n;
//   //
//   vs.setValues(name, 0, true);
//   par[0] = &vs;
//   // parameter 1 is polygon to add
//   db = poly;
//   par[1] = (UVariable *) db;
//   // parameter 3 is coordinate system (0=odo, 1=utm, 2=map)
//   vCoo.setDouble(coordinateSystem);
//   par[2] = &vCoo;
//   // number of parameters
//   n = 3;
//   // set return value
//   dbr = &vr;
//   // do the call
//   isOK = callGlobalV("poly.del", "scd", par, &dbr, &n);
//   if (not isOK or not vr.getBool())
//     printf("UResAvoid::copyFootprintPolys: failed to 'poly.setPoly(%s,c,d)'\n", name);
//   return isOK;
// }

//////////////////////////////////////////////////////////

bool UFunctionPassable::makePoly(int maxCnt,
                                 bool newestUpdatesOnly,
                                 bool andFixed)
{
  bool result = true;
  int cnt;
  int n, m, nf = 0;
  const int MSL = 100;
  char s[MSL];
  const char * OBST_GRP_NAME = "obstacle";
  UObstacleGroup * og;
  UObstacle * ob;
  UObstacleGroup * ogFix = NULL;
  USmlTag tag;
  UTime ut;
  //
  n = obsts->getGroupsCnt();
  cnt = mini(n, maxCnt);
    // count number of groups with obstacles
  if (cnt > 0)
  { // get newest update time
    og = obsts->getGroup(0);
    ut = og->getPPoseLast()->t;
  }
  if (andFixed)
  {
    ogFix = obsts->getFixeds();
    if (ogFix!= NULL)
      if (ogFix->getObstsCnt() > 0)
        nf = 1;
  }
  //
  for (n = 0; n < cnt + nf; n++)
  {
    char color = n % 10 + '0'; // polygon color follow group number in gray-scale
    if (n < cnt)
      og = obsts->getGroup(n);
    else
      og = ogFix;
    // send obstacles
    for (m = 0; m < og->getObstsCnt(); m++)
    {
      ob = og->getObstacle(m);
      if (not newestUpdatesOnly or ob->getPoseLast().t == ut)
      { // should be updated
        snprintf(s, MSL, "%s-grp%03d-%03d", OBST_GRP_NAME, n, m);
        polygonToPolyPlugin(ob, s, 0, color);
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////

bool UFunctionPassable::sendRoadLines(UServerInMsg * msg, int upds, double qual, bool bestOnly)
{
  bool result = true;
  int cnt;
  int n;
  const int MRL = 1000;
  char reply[MRL];
  const int MSL = 100;
  char s1[MSL];
  char s2[MSL];
  USmlTag tag;
  URoadLine * road;
  unsigned long serial = 0;
  UPoseTime pt;
  bool best;
  //
  cnt = roads->getRoadsCnt();
  serial = lastSerial[lasPool->getDefDeviceNumber()];
  if (odoHist != NULL)
    pt = odoHist->getNewest(NULL);
  else
    pt.clear();
  // format common header
  snprintf(reply, MRL, "<%s scan=\"%lu\" tod=\"%lu.%06lu\" "
          "updLim=\"%d\" qualLim=\"%g\" bestOnly=\"%s\">\n",
           msg->tag.getTagName(), serial, pt.t.getSec(), pt.t.getMicrosec(),
          upds, qual, bool2str(bestOnly));
  sendMsg(msg, reply);
  //
  road = roads->getRoadLine(0);
  for (n = 0; n < cnt; n++)
  { /* -- format
    <road name="road" scan="777" tod="11654321.098765">
      <lineSeg name="left" length="3.21" q="0.8765">
        <pos3D name="start" x="123.456" y="0" z="0"/>
        <pos3D name="vector" x="123.456" y="0" z="0"/>
      </lineSeg>
      <lineSeg name="center" length="3.21" q="0.8765">
        <pos3D name="start" x="123.456" y="0" z="0"/>
        <pos3D name="vector" x="123.456" y="0" z="0"/>
      </lineSeg>
    </road> */
    best = (n == roads->getLeftIdx()) or
        (n == roads->getRightIdx()) or
        (n == roads->getCentIdx());
    if (road->isValid() and
        (road->getUpdateCnt() > upds) and
        (road->getQual() > qual) and
        (best or not bestOnly))
    {
      if (best)
        snprintf(s1, MSL, "q=\"%.5f\" n=\"%d\" valid=\"%s\" best=\"true\" idx=\"%d\" scan=\"%lu\" pis=\"%d\" serial=\"%lu\"",
                 road->getQual(), road->getUpdateCnt(), bool2str(road->isValid()), n, road->getScanSerial(), road->pisIdx, road->getLineSerial());
      else
        snprintf(s1, MSL, "q=\"%.5f\" n=\"%d\" valid=\"%s\" idx=\"%d\" scan=\"%lu\" pis=\"%d\"",
                 road->getQual(), road->getUpdateCnt(), bool2str(road->isValid()), n, road->getScanSerial(), road->pisIdx);
      switch (road->getLineType())
      {
        case 0: strcpy(s2, "left"); break;
        case 1: strcpy(s2, "center"); break;
        case 2: strcpy(s2, "right"); break;
      }
      tag.codeLineSegment(road->getLine(), reply, MRL, s2, s1);
      result = sendMsg(msg, reply);
    }
    road++;
    if (not result)
      break;
  }
  //
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  result = sendMsg(msg, reply);
  //
  return result;
}

