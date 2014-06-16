/** *************************************************************************
 *                                                                         *
 *   \file              ufuncline.cpp                                      *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson, et al        *
 *   \date              Dec 2006                                           *
 *   \brief             ScanFeatures plugin for ulmsserver v. > 1.5        *
 *                                                                         *
 *   Implementation of the ScanFeatures main class and communication with  *
 *   the laser server.                                                     *
 *                                                                         *
 *                      Copyright (C) 2006-2008 by DTU                     *
 *                      rse@oersted.dtu.dk                                 *
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

#include <urob4/usmltag.h>
#include <urob4/uresposehist.h>
#include <auef/polarlinefit.h>

#include "ufuncline.h"


#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded
UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncEfLine();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncEfLine::UFuncEfLine()
{ // initialization of variables in class - as needed
  setCommand("line", "laserline", "extraction of lines from a laserscan (deprecated?)");
  lastSerial = 0;
  lastDevice = 0;
  auef = NULL;
}

///////////////////////////////////////////////////

UFuncEfLine::~UFuncEfLine()
{
  if (auef != NULL)
    delete auef;
}

///////////////////////////////////////////////////

void UFuncEfLine::createResources()
{
  auef = new UResAuEf();
  addResource(auef, this);
}


///////////////////////////////////////////////////
bool UFuncEfLine::handleCommand(UServerInMsg * msg, void * extra)
{
  // send a short reply back to the client requested the 'bark'
  const int MRL = 200;
  char reply[MRL];
  ULaserData * pushData = (ULaserData *)extra;
  const int MVL = 30;
  char value[MVL];
  int device = -1;
  int fake = 0; // fake > 0 fakes data - no device or simulator needed (device dependent)
  bool ask4help;
  bool setLog = false;
  bool logOpen = false;
  bool doBox = false;
  bool doBoxPose;
  bool doBoxReset;
  bool doGetOnly = false;
  //UPose odo, boxOdo, rsw;
  ULaserDevice * las = NULL;
  ULaserData * data;
  bool result = false;
  //
  // check for parameters (XML tag attributes in command message)
  ask4help = msg->tag.getAttValue("help", value, MVL);
  if (msg->tag.getAttValue("device", value, MVL))
    device = strtol(value, NULL, 0);
  if (msg->tag.getAttValue("fake", value, MVL))
    fake = strtol(value, NULL, 10);
  if (msg->tag.getAttValue("log", value, MVL))
  {
    setLog = true;
    logOpen = str2bool2(value, true);
  }
  msg->tag.getAttValueBool("box", &doBox, true);
  msg->tag.getAttValueInt("device", &device);
  doBoxReset = (msg->tag.getAttValue("boxReset", value, MVL));
  doBoxPose = (msg->tag.getAttValue("boxPose", value, MVL));
  doGetOnly = (msg->tag.getAttValue("getOnly", value, MVL));
  //
  //
  if (ask4help)
  { // if help, then nothing else
    sendHelpStart(msg, "LINE");
    sendText(msg, "--- available LINE (extract feature) options\n");
    sendText(msg, "device=N        Laser device to use (see: SCANGET help)\n");
    sendText(msg, "fake=N          Fakes some range data (N=1 random, N=2..3 fake corridor)\n");
    sendText(msg, "box[=true]      Return just wall lines (4 wall box lines)\n");
    sendText(msg, "boxPose         Return position of south-west corner (x,y,th)\n");
    sendText(msg, "boxReset        Reset box-wall lines\n");
    sendText(msg, "getOnly         get last calculated value\n");
    if (auef != NULL)
    {
      snprintf(reply, MRL, "log=true|false  (re)Open or close '%s' logfile, (open=%s) \n",
               auef->llogName, bool2str(auef->isLogOpen()));
      sendText(msg, reply);
    }
    sendText(msg, "help            This message\n");
    sendText(msg, "-----\n");
    sendText(msg, "Depends on laserPool and odoPose modules\n");
    sendText(msg, "see also SCANGET\n");
    sendHelpDone(msg);
  }
  else if (auef != NULL)
  { // the calculation resource is available
    if (setLog)
    { // open or close logfile
      auef->openLog(not logOpen);
      sendInfo(msg, "done");
    }
    else if (doBoxReset)
    { // just reset, nothing else
      auef->wallReset();
      sendInfo(msg, "done");
    }
    //
    // all other options require a laser scanner device and possibly a scan
    // so
    else
    { // get laser device and data
      data = getScan(device, &las, pushData, doGetOnly, fake);
      // data fetch is finished
      if (data->isValid() or doGetOnly)
      { // laserdata is valid
        // Do processing and send reply as needed
        if (doBox)
        { // send just box lines
          sendBoxLines(msg, las, data, doGetOnly);
        }
        else if (doBoxPose)
        { // send the box-pose only, i.e. south-west corner in odometry coordinates
          // including the box width (west-east) and height (south-north)
          sendBoxPose(msg, las, data, doGetOnly);
        }
        else // default is all raw lines
        { // return all found segments to a full XML able client
          sendAllLines(msg, las, data, doGetOnly);
        }
        result = true;
      }
      else
        sendWarning(msg, "No scandata available");
    }
  }
  else // (auef == NULL)
    sendWarning(msg, "extract feature resource is missing");
  // return true if the function is handled with a positive result
  return result;
}

//////////////////////////////////////////////////////////////

ULaserData * UFuncEfLine::getScan(int device,
                                ULaserDevice ** las,
                                ULaserData * pushData,
                               bool getOnly,
                               int fake)
{
  ULaserPool * lasPool;
  ULaserData * scan = &dataBuff;
  //
  // get pouinter to laser pool (but do not create if not available - should be)
  lasPool = (ULaserPool *)getStaticResource("lasPool", false);
  if (lasPool != NULL)
  { // laser pool is available
    if (not getOnly)
      // get (or use) a new scan
      scan = lasPool->getScan(device, las, &dataBuff, pushData, fake);
    else
      // get laser scanner device only
      lasPool->getScan(device, las, NULL, NULL, 0);
  }
  return scan;
}

////////////////////////////////////////////////////////////////

UPose UFuncEfLine::getCurrentOdoPose(ULaserData * data)
{
  UPose pose;
  UResPoseHist * odoPose;
  //
  if (data->isFake())
    pose = data->getFakePose();
  else
  { // odometry resource is needed
    odoPose = (UResPoseHist*)getStaticResource("odoPose", true);
    if (odoPose != NULL)
      // get pose at time of scan
      pose = odoPose->getPoseAtTime(data->getScanTime());
    else
      // no pose data available
      pose.clear();
  }
  //
  return pose;
}

////////////////////////////////////////////////////////////

bool UFuncEfLine::sendRobotPoseAndSensorPosition(UServerInMsg * msg,
    ULaserDevice * las, ULaserData * data)
{
  UPose pose;  // x,y,h
  USmlTag tag; // codes known data types into XML
  bool sendOK;
  const int MRL = 200;
  char reply[MRL];
  //
  // fetch robot pose and add as pose message
  pose = getCurrentOdoPose(data);
  tag.codePose(pose, reply, MRL, "robot");
  sendMsg(msg, reply);
  // send also sensor pose - as a 3D position and a 3D rotation
  sendMsg(msg, tag.codePosition(las->getDevicePos(), reply, MRL, "device"));
  sendOK = sendMsg(msg, tag.codeRotation(las->getDeviceRot(), reply, MRL, "device"));
  //
  return sendOK;
}

//////////////////////////////////////////////////////

bool UFuncEfLine::sendAllLines(UServerInMsg * msg,
                  ULaserDevice * las,
                  ULaserData * data,
                  bool getOnly)
{
  int segs;
  bool sendOK = false;
  const int MRL = 200;
  char reply[MRL];
  UTime t;
  //
  auef->extractLock.lock();
  if (not getOnly)
  { // Treat data
    t = data->getScanTime();
    auef->findFeatures(data);
  }
  else
    t = auef->getLastUpdateTime();
  //
  //
  // return all found segments to a full XML able client
  segs = auef->linesCnt;
  snprintf(reply, MRL, "<%s timestamp=\"%lu.%03lu\" lines=\"%d\">\n",
           msg->tag.getTagName(), t.GetSec(), t.getMilisec(), segs);
  sendMsg(msg, reply);

  // send also robot and sensor info
  sendRobotPoseAndSensorPosition(msg, las, data);
  // send the lines
  for (int i = 0; i < segs; i++) {
    auef->lines[i].toXMLString(reply, MRL);
    sendMsg(msg, reply);
    snprintf(reply, MRL, "<line x=\"%g\" y=\"%g\" l=\"0.001\" name=\"w\" age=\"%d\"/>\n", auef->lines[i].x, auef->lines[i].y, i);
    sendMsg(msg, reply);
  }
  sendOK = sendEndTag(msg);
  //
  auef->extractLock.unlock();
  //
  return sendOK;
}

////////////////////////////////////////////////////////

bool UFuncEfLine::sendBoxLines(UServerInMsg * msg,
                               ULaserDevice * las,
                               ULaserData * data,
                               bool getOnly)
{
  int segs;
  bool sendOK = false;
  const int MRL = 200;
  char reply[MRL];
  UTime t;
  const char * sideName[4] = {"north", "east", "south", "west"};
  const int MSL = 50;
  char s[MSL];
  UPose odo;
  //
  auef->extractLock.lock();
  if (not getOnly)
  { // Treat data
    t = data->getScanTime();
    // get the line features from scan
    auef->findFeatures(data);
    // do wall detection and update
    odo = getCurrentOdoPose(data);
    segs = auef->updateWalls(odo);
  }
  else
    // just need update time
    t = auef->getLastUpdateTime();
  // get number of raw line features in scan
  segs = auef->linesCnt;
  //
  // send the reply - first XML start tag (using same name as the request
  snprintf(reply, MRL, "<%s timestamp=\"%lu.%03lu\" lines=\"%d\">\n",
           msg->tag.getTagName(), t.GetSec(), t.getMilisec(), segs);
  sendMsg(msg, reply);
  // send also robot and sensor info
  sendRobotPoseAndSensorPosition(msg, las, data);
  // now send the 4 wall lines
  for (int i = 0; i < 4; i++)
  { // code extra attributes holding name and age flags
    snprintf(s, MSL, "name=\"%s\" age=\"%d\"",
             sideName[i], auef->wallsAge[i]);
    // code the line itself (the line is able to do that)
    auef->wallSeg[i].toXMLString(reply, MRL, s);
    // send the message to the client
    sendMsg(msg, reply);
  }
  // send the end XML tag
  sendOK = sendEndTag(msg);
  //
  auef->extractLock.unlock();
  //
  return sendOK;
}

//////////////////////////////////////////////////////////////

bool UFuncEfLine::sendBoxPose(UServerInMsg * msg,
                               ULaserDevice * las,
                               ULaserData * data,
                               bool getOnly)
{
  int segs;
  bool sendOK = false;
  const int MRL = 200;
  char reply[MRL];
  UTime t;
  UPose odo, rsw, boxOdo;
  //
  odo = getCurrentOdoPose(data);
  auef->extractLock.lock();
  if (not getOnly)
  { // Treat data
    t = data->getScanTime();
    // get the line features from scan
    auef->findFeatures(data);
    // do wall detection and update
    segs = auef->updateWalls(odo);
  }
  else
    // just need update time
    t = auef->getLastUpdateTime();
  // get number of raw line features in scan
  segs = auef->linesCnt;
  //
  // get box pose in scanner coordinates
  rsw = auef->boxPose;
  // move back to robot coordinates
  rsw.x += las->getDevicePose().getX();
  // transfer from robot coordinates to odometry coordinates
  boxOdo = odo.getPoseToMapPose(rsw);
  // reply is for MRC in the format
  // <laser l1="x" l2="y" l3="th" l4="width" l5="height"/>
  snprintf(reply, MRL, "<laser l0=\"%.4f\" l1=\"%4f\" "
      " l2=\"%.6f\" l3=\"%.2f\" l4=\"%.2f\"/>\n",
      boxOdo.x, boxOdo.y, boxOdo.h,
      auef->boxEW, auef->boxNS);
  sendMsg(msg, reply);
  //
  auef->extractLock.unlock();
  //
  return sendOK;
}
