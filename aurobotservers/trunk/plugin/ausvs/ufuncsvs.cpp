/** *************************************************************************
 *                                                                         *
 *   \file              ufuncsvs.cpp                                       *
 *   \author            Christian Andersen                                 *
 *   \date              mar 2008                                           *
 *   \brief             Stereo interface plugin for SVS library            *
 *                                                                         *
 *   Implementation of SVS library for stereo camera                       *
 *                                                                         *
 *                      Copyright (C) 2008 by DTU                          *
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
 ************************************************************************* */

//#ifdef USE_SVS

#include <stdio.h>

#include <urob4/usmltag.h>

#include "ufuncsvs.h"

///////////////////////////////////////////////////

// this part may be removed to allow module tester debug
// using these modules as static

UTimeNow libLoadTime;
bool libLoadTimeSet = false;

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncSVS();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncSVS::UFuncSVS()
{
  setCommand("svs svsPush", "SVSstereo", "stereo library interface module");
  if (libLoadTimeSet)
  {
    svsTimeRef.now();
    printf("Time set at library load time %fms ago\n", (svsTimeRef - libLoadTime) * 1000.0);
    svsTimeRef = libLoadTime;
  }
  else
    svsTimeRef.now();
  // initialization of variables in class - as needed
  lastSerial = 0;
  ressvs = NULL;
  //ressvsLocal = false;
}

///////////////////////////////////////////////////

UFuncSVS::~UFuncSVS()
{
  if (ressvs != NULL)
    delete ressvs;
}

///////////////////////////////////////////////////

const char * UFuncSVS::name()
{
  return "svs (Stereo Vision system interface) (" __DATE__ " " __TIME__ " by DTU/Videre design)";
}

/////////////////////////////////////////////////////

void UFuncSVS::createResources()
{
  ressvs = new UResSVS();
  addResource(ressvs, this);
  ressvs->setImageTimeRef(svsTimeRef);
}

///////////////////////////////////////////////////

bool UFuncSVS::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("svs"))
  {
    result = handleSVS(msg, (USvsImageSet *)extra);
  }
  else if (msg->tag.isTagA("svsPush"))
  {
      result = handlePushCommand(msg);
  }
  else
  {
    sendDebug(msg, "Command not handled (by me)");
  }
  return result;
}

///////////////////////////////////////////////////

const char * UFuncSVS::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Stereo processing (Videre Design library)\n",preString);
  return buff;
}

///////////////////////////////////////////////////

bool UFuncSVS::handleSVS(UServerInMsg * msg, USvsImageSet * pushUsi)
{
  // send a short reply back to the client requested the 'bark'
  const int MRL = 200;
  char reply[MRL];
  const int MVL = 30;
  char value[MVL];
  USmlTag tag;
  bool ask4help;
  bool setLog = false;
  bool logOpen = false;
  bool doSvs = false;
  bool doOpen;
  bool doOpenValue = false;
  bool result;
  USvsImageSet * usi = pushUsi;
  bool doToImgPool = false;  // default action
  bool doReplay = false;
  bool doReplayValue = false;
  bool do3Dfile;
  bool replyOK = false;
  bool doStep;
  int doStepCnt = 1;
  bool doShutter;
  int doShutterValue = 80;
  bool doGain;
  int doGainValue = 0;
  bool doCalib;
  char doCalibValue[MAX_FILENAME_SIZE];
  bool doVerbose;
  bool doVerboseValue = true;
  bool doPose = false;
  bool doPoseVal = false;
  bool doImgSubdir;
  bool doFran;
  bool silent;
  int doFranVal = -1;
  char imgSubdir[MAX_FILENAME_SIZE];
  UPosRot setPose6d;
  // get default pose
  if (ressvs != NULL);
    setPose6d = ressvs->getSensorPose();
  //
  // check for parameters (XML tag attributes in command message)
  ask4help = msg->tag.getAttValue("help", value, MVL);
  setLog = msg->tag.getAttValueBool("log", &logOpen, true);
  msg->tag.getAttValueBool("do", &doSvs, true);
  msg->tag.getAttValueBool("img", &doToImgPool, true);
  doReplay = msg->tag.getAttValueBool("replay", &doReplayValue, true);
  //msg->tag.getAttValueBool("fake", &fake, true);
  do3Dfile =  msg->tag.getAttValue("3Dfile", value, MVL);
  doOpen =  msg->tag.getAttValueBool("open", &doOpenValue, true);
  doStep = msg->tag.getAttValueInt("step", &doStepCnt);
  doShutter = msg->tag.getAttValueInt("shutter", &doShutterValue);
  doGain = msg->tag.getAttValueInt("gain", &doGainValue);
  doCalib = msg->tag.getAttValue("calib", doCalibValue, MAX_FILENAME_SIZE);
  doImgSubdir = msg->tag.getAttValue("imgSubdir", imgSubdir, MAX_FILENAME_SIZE);
  doVerbose = msg->tag.getAttValueBool("verbose", &doVerboseValue, true);
  silent = msg->tag.getAttValueBool("silent", NULL, true);
  doPose = msg->tag.getAttValue("pose", value, MVL);
  doPoseVal |= msg->tag.getAttValueD("x", &setPose6d.x);
  doPoseVal |= msg->tag.getAttValueD("y", &setPose6d.y);
  doPoseVal |= msg->tag.getAttValueD("z", &setPose6d.z);
  doPoseVal |= msg->tag.getAttValueD("o", &setPose6d.Omega);
  doPoseVal |= msg->tag.getAttValueD("p", &setPose6d.Phi);
  doPoseVal |= msg->tag.getAttValueD("k", &setPose6d.Kappa);
  doFran = msg->tag.getAttValueInt("fran", &doFranVal);
  //
  //
  if (ask4help)
  { // send help text
    sendHelpStart(msg, "SVS");
    sendText(msg,          "--- available SVS (stereo processing) options\n");
    if (ressvs != NULL)
    {
      snprintf(reply, MRL, "calib[=file]    Set svs calibration file (before open), is %s\n",
               ressvs->getSvsCalibFile());
      sendText(msg, reply);
      sendText(msg,        "do[=true]       Do stereo processing of from device\n");
      snprintf(reply, MRL, "open[=false]    Explicit open (in streaming mode) or close (is open: %s)\n",
               bool2str(ressvs->isStreaming));
      sendText(msg, reply);
      sendText(msg,        "img[=false]     Save images to image pool (default is true)\n");
      sendText(msg,        "                (see 'var svs' for image numbers)\n");
      sendText(msg,        "3Dfile[=true]   Save disparity as a 3D file (svs3D.txt)\n");
      snprintf(reply, MRL, "replay[=true]   Set source as replay (svs.log in replayPath) is %s\n", bool2str(ressvs->isReplay()));
      sendText(msg, reply);
      snprintf(reply, MRL, "imgSubdir=\"sub/\" Set replay imageset subdir, is %s\n", ressvs->getImageSetSubdir());
      sendText(msg, reply);
      sendText(msg,        "step[=n]        Step one image set during replay\n");
      snprintf(reply, MRL, "log=true|false  (re)Open or close '%s' logfile, (open=%s) \n",
               ressvs->getImgLogName(), bool2str(ressvs->isImgLogOpen()));
      sendText(msg, reply);
      snprintf(reply, MRL, "shutter=S       Set shutter value 0-100 (-1 is auto), is %d auto=%s\n",
               ressvs->getShutter(), bool2str(ressvs->isShutterAuto()));
      sendText(msg, reply);
      snprintf(reply, MRL, "gain=S          Set video gain value 0-100 (-1 is auto), is %d auto=%s\n",
               ressvs->getGain(), bool2str(ressvs->isGainAuto()));
      sendText(msg, reply);
      //sendText(msg,        "fran            Do hard coded fran analysis (exposure etc)\n");
      sendText(msg,        "pose x=X y=Y z=Z omega=O phi=P kappa=K Set sensor pose\n"
                           "                X,Y,Z in meter and O,P,K in radins\n");
      snprintf(reply, MRL, "verbose[=false] Set verbose flag to true or false, (is %s)\n",
               bool2str(ressvs->verbose));
      sendText(msg, reply);
      sendText(            "silent          Suppress most replies\n");
    }
    else
      sendText(msg,        "**** error svs resource is not available - reload module ****\n");
    sendText(msg,          "help            This message\n");
    sendText(msg,          "-----\n");
    sendText(msg,          "see also SVSPUSH POOLLIST, 'server replayPath' and 'var svs' for other parameters\n");
    sendHelpDone(msg);
  }
  else if (ressvs != NULL)
  {
    if (doPose)
    {
      if (doPoseVal)
        ressvs->setSensorPose(&setPose6d);
      snprintf(reply, MRL, "sensor pose %gx %gy %gz %gO %gP %gK",
               setPose6d.x, setPose6d.y, setPose6d.z,
               setPose6d.Omega, setPose6d.Phi, setPose6d.Kappa);
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doCalib)
    {
      ressvs->setSvsCalibFile(doCalibValue);
      snprintf(reply, MRL, "Using %s at next camera open", ressvs->getSvsCalibFile());
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doOpen)
    {
      if (doOpenValue)
        ressvs->startStreaming();
      else
        ressvs->stopStreaming();
      snprintf(reply, MRL, "streaming=%s", bool2str(ressvs->isStreaming));
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (setLog)
    { // open or close logfile
      ressvs->openLogs(logOpen);
      snprintf(reply, MRL,
          "\nlogging images   (%s) to %s\n"
            "logging horizon  (%s) to %s\n"
            "logging exposure (%s) to %s\n",
          bool2str(ressvs->isImgLogOpen()), ressvs->getImgLogName(),
          bool2str(ressvs->isHozLogOpen()), ressvs->getHozLogName(),
          bool2str(ressvs->isExposureLogOpen()), ressvs->getExposureLogName()
              );
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doImgSubdir)
    {
      ressvs->setImageSetSubdir(imgSubdir);
      snprintf(reply, MRL, "Replay path is now %s/%s", replayPath, ressvs->getImageSetSubdir());
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doReplay)
    {
      ressvs->setReplay(doReplayValue);
      snprintf(reply, MRL, "Replay now set to %s", bool2str(ressvs->getReplay()));
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doStep)
    {
      ressvs->replayStep(maxi(0, doStepCnt));
      if (not silent)
      {
        snprintf(reply, MRL, "Replay step to frame %d started", ressvs->getReplayFrame());
        sendHelp(msg, reply);
      }
      replyOK = true;
    }
    if (doSvs)
    { // do sample processing
      usi = ressvs->getImageSet(pushUsi, doToImgPool, doReplay);
      // send start tag with same name as command (i.e. line)
      result = (usi != NULL);
      if (result)
      { // image set is ready for stereo calculation
        result = ressvs->doDisparityCalculation(usi, doToImgPool);
        if (do3Dfile)
          ressvs->to3Dfile(usi);
        if (pushUsi == NULL)
          // release image buffer
          usi->unlock();
      }
      if (not silent)
      {
        snprintf(reply, MRL, "Stereo processed (%s)", bool2str(result));
        sendHelp(msg, reply);
      }
      replyOK = true;
    }
    if (doToImgPool and not doSvs)
    { // get and log an image pair
      usi = ressvs->getImageSet(pushUsi, doToImgPool, doReplay);
      // send start tag with same name as command (i.e. line)
      result = (usi != NULL);
      if (result)
        result = usi->si != NULL;
      if (result)
        result = usi->si->haveColor;
      if (usi != NULL)
      { // image set is now in imagepool
        usi->unlock();
      }
      snprintf(reply, MRL, "Images saved in pool (%s) and file (%s)",
               bool2str(result), bool2str(ressvs->isImgLogOpen()));
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doShutter)
    {
      ressvs->setShutter(doShutterValue);
      snprintf(reply, MRL, "Shutter is now %d auto=%s", ressvs->getShutter(), bool2str(ressvs->isShutterAuto()));
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doGain)
    {
      ressvs->setGain(doGainValue);
      snprintf(reply, MRL, "Video gain is now %d auto=%s", ressvs->getGain(), bool2str(ressvs->isGainAuto()));
      sendHelp(msg, reply);
      replyOK = true;
    }
    if (doVerbose)
    {
      ressvs->verbose = doVerboseValue;
      replyOK = true;
    }
    if (doFran)
    {
      //doFranCode(msg, doFranVal);
      sendInfo(msg, "Fran test-image code disabled for now");
      replyOK = true;
    }
    if (not silent)
    {
      if (not replyOK)
        sendWarning(msg, "done nothing");
      else
        sendInfo(msg, "done");
    }
  }
  else
    sendWarning(msg, "No stereo resource available");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////////

bool UFuncSVS::handlePushCommand(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4list = false;
  const int MRL = 1000;
  char reply[MRL];
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4list = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"SVSPUSH\">\n");
    sendText(msg, "----------------Available SVSPUSH settings:\n");
    sendText(msg, "total=k or n=k    Stop push after k events (def: no stop)\n");
    sendText(msg, "interval=k or i=k Push with this event interval (in images, def = 1)\n");
    sendText(msg, "flush[=cmd]       Remove push command from client (default all)\n");
    sendText(msg, "cmd=\"cmd\"         Do execute 'cmd' command for every event\n");
    sendText(msg, "call=method(par)  Do call method on event\n");
    sendText(msg, "list              List all active push commands\n");
    sendText(msg, "See also PUSH (timed push), SVS\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (ressvs != NULL)
  { // push command
    if (ask4list)
    {
      sendMsg(msg, "<help subject=\"svsPush command list\">\n");
      ressvs->UServerPush::print("svsPush", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    {
      result = ressvs->addPushCommand(msg);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d",
                 ressvs->getPushCmdCnt(NULL, NULL));
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

//////////////////////////////////////////////////
// bool UFuncSVS::doFranCode(UServerInMsg * msg, int franVal)
// {
//   if (ressvs != NULL)
//   {
//     ressvs->doFranCode(franVal);
//   }
//   return true;
// }


//#endif

