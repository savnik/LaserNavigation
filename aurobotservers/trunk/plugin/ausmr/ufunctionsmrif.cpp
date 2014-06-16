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

#include <stdio.h>

#include "ufunctionsmrif.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionSmrIf' with your classname, as used in the headerfile */
  return new UFunctionSmrIf();
}
//
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionSmrIf::UFunctionSmrIf()
{ 
  // command list and version text
  setCommand(commandList(), "AuSMR", name());
  // initialization of variables in class - as needed
  smrif = NULL;       // initially the resource is not created
//  smrifLocal = false; // ... and is thus not local
  smrctl = NULL;
//  smrctlLocal = false;
}

///////////////////////////////////////////////////

UFunctionSmrIf::~UFunctionSmrIf()
{ // possibly remove allocated variables here - if needed
  if (smrif != NULL)
    delete smrif;
  if (smrctl != NULL)
    delete smrctl;
}

///////////////////////////////////////////////////

const char * UFunctionSmrIf::name()
{
  return "smrif (v2.192) (" __DATE__ " " __TIME__ " by Christian)";
}

///////////////////////////////////////////////////

const char * UFunctionSmrIf::commandList()
{ // space separated list og command keywords handled by this plugin
  return "smr smrConnect";
}

///////////////////////////////////////////////////

void UFunctionSmrIf::createResources()
{ // load resource for this plugin
  smrif = new UResSmrIf();
  // tell server about the resource, and that this object is the owner
  addResource(smrif, this);
  //
  smrctl = new UResSmrCtl();
  addResource(smrctl, this);
}


///////////////////////////////////////////////////

bool UFunctionSmrIf::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("smr"))
    result = handleSmr(msg);
  else if (msg->tag.isTagA("smrConnect"))
    result = handleSmrPush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionSmrIf::handleSmr(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 150;
  char reply[MRL];
  bool ask4help;
  const int MVL = 150;
  char val[MVL];
  const int MML = 150;
  char smrMsg[MML];
  char smrEval[MVL];
  bool gotMsg;
  bool gotEval;
  bool handled = false;
//  const char * smrReply = NULL;
  bool gotHost;
  char host[MVL] = "localhost";
  bool gotPort;
  int port = 31001;
  bool gotHup = false;
  bool isOK;
  bool gotConnect;
  bool gotGetEvent = false;
  bool isConnected = false;
  char * hostName = host;
  char *p1, *p2;
  double v;
  bool logIOopen = false;
  int logIOMode = 0;
  bool gotLog;
  bool gotLogVal = false;
  bool gotLogM;
  int gotLogMVal = -1;
  bool streamStarted = false;
  bool gotStream;
  int gotStreamVal = 0;
//  bool gotStart;
//  bool gotStartVal = false;
  bool gotStop = false;
  bool gotStreamGpsN = false, gotStreamOdoN = false, gotStreamInsN = false;
  int streamGpsN = 0, streamOdoN = 0, streamInsN = 0;
  int streamGpsCnt = 0, streamOdoCnt = 0, streamInsCnt = 0;
  bool logINSopen = false;
  bool gotLogIns = false;
  bool logIns = false;
  bool gotLogCtl = false, logCtl=true;
  bool gotLogPrim = false, logPrim=true;
  bool gotSaveMrcLog = false;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  // get all other (needed) parameters
  gotMsg = msg->tag.getAttValue("cmd", smrMsg, MML);
  gotMsg = msg->tag.getAttValue("do", smrMsg, MML);
  gotEval = msg->tag.getAttValue("eval", smrEval, MVL);
  gotHost = msg->tag.getAttValue("host", host, MVL);
  gotPort = msg->tag.getAttValue("port", val, MVL);
  if (gotPort)
    port = strtol(val, NULL, 10);
  gotConnect = msg->tag.getAttValue("connect", val, MVL);
  if (gotConnect and (strlen(val) > 0) and not gotHost and not gotPort)
  { // colon-notation used   'host:port'
    p2 = val;
    p1 = strsep(&p2, ":");
    if (p2 != NULL)
    { // got a colon, so hoat:port format
      gotPort = strlen(p2) > 0;
      if (gotPort)
        port = strtol(p2, NULL, 0);
      gotHost= strlen(p1) > 0;
      if (gotHost)
        strncpy(host, p1, MVL);
    }
  }
  gotHup = msg->tag.getAttValue("hup", val, MVL);
  gotGetEvent = msg->tag.getAttValue("getevent", val, MVL);
  gotLog = msg->tag.getAttValue("log", val, MVL);
  if (gotLog)
    gotLogVal = str2bool2(val, true);
  gotLogM = msg->tag.getAttValue("logMode", val, MVL);
  if (gotLogM)
    gotLogMVal = strtol(val, NULL, 10);
/*  gotStart = msg->tag.getAttValue("start", val, MVL);
  if (gotStart)
    gotStartVal = str2bool2(val, true);*/
  gotStop = msg->tag.getAttValue("stop", val, MVL);
  gotStream = msg->tag.getAttValue("stream", val, MVL);
  if (gotStream and strlen(val) > 0)
    gotStreamVal = strtol(val, NULL, 10);
  gotStreamGpsN = msg->tag.getAttValue("streamShowGps", val, MVL);
  if (gotStreamGpsN)
    streamGpsN = strtol(val, NULL, 10);
  gotStreamOdoN = msg->tag.getAttValue("streamShowOdo", val, MVL);
  if (gotStreamOdoN)
    streamOdoN = strtol(val, NULL, 10);
  gotStreamInsN = msg->tag.getAttValue("streamShowIns", val, MVL);
  if (gotStreamInsN)
    streamInsN = strtol(val, NULL, 10);
  gotLogIns = msg->tag.getAttValue("logIns", val, MVL);
  if (gotLogIns)
    logIns = str2bool2(val, true);
  gotLogCtl = msg->tag.getAttValue("logCtl", val, MVL);
  if (gotLogCtl)
    logCtl = str2bool2(val, true);
  gotLogPrim = msg->tag.getAttValue("logPrim", val, MVL);
  if (gotLogPrim)
    logPrim = str2bool2(val, true);
  gotSaveMrcLog = msg->tag.getAttValue("saveMrcLog", val, MVL);
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    if (smrif != NULL)
    {
      isConnected = smrif->isConnected();
      hostName = smrif->getHost();
      port = smrif->getPort();
      logIOopen = smrif->isLogging();
      logIOMode = smrif->getLogMode();
      logINSopen = smrif->isInsLogOpen();
      streamStarted = smrif->isStreaming();
      streamGpsN = smrif->streamShowGpsEvery;
      streamOdoN = smrif->streamShowOdoEvery;
      streamGpsCnt = smrif->streamShowGpsCnt;
      streamOdoCnt = smrif->streamShowOdoCnt;
      streamInsCnt = smrif->streamShowInsCnt;
    }
    sendMsg( msg, "<help subject=\"SMR\">\n");
    sendText(msg, "--- available SMR options\n");
    snprintf(reply, MRL, "connect[=host:port]  Connect to MRC (connected=%s)\n", bool2str(isConnected));
    sendText(msg, reply);
    snprintf(reply, MRL, "host='host'          Set host name (is '%s')\n", hostName);
    sendText(msg, reply);
    snprintf(reply, MRL, "port='P'             Set connection port number (is '%d')\n", port);
    sendText(msg, reply);
    sendText(msg,        "cmd='cmd'            Send this command to MRC\n");
    sendText(msg,        "do='cmd'             Send this command to MRC\n");
    sendText(msg,        "hup                  Stop and hang up connection to MRC\n");
    sendText(msg,        "stop                 Stop the robot now (keeps connection open)\n");
    sendText(msg,        "eval='variable'      Get value of variable in MRC\n");
    sendText(msg,        "getevent             Empty the getevent buffer from MRC\n");
    snprintf(reply, MRL, "log=false            start or stop smrcl.log (open=%s)\n", bool2str(logIOopen));
    sendText(msg, reply);
    snprintf(reply, MRL, "logMode=0..3         0=just errors, 1=+send, 2=+rec, 3+mgmt (is=%d)\n", logIOMode);
    sendText(msg, reply);
    sendText(            "saveMrcLog           Tell MRC to save log (mrc_yyyymmdd.hhmmss.ccc.logg)\n");
    if (smrctl != NULL)
    {
      snprintf(reply, MRL, "logCtl[=false]       drive control log 'mrcctl.log' (open=%s)\n", bool2str(smrctl->logctl.isOpen()));
      sendText(msg, reply);
      snprintf(reply, MRL, "logPrim[=false]      drive control lines and arcs 'mrcprim.log' (open=%s)\n", bool2str(smrctl->logprim.isOpen()));
      sendText(msg, reply);
    }
//    sendText(msg,        "start[=false]        Start (or stop) interface control loop\n");
    snprintf(reply, MRL, "stream=n             Start streaming at this interval - (odo and gps) (started=%s)\n", bool2str(streamStarted));
    sendText(msg, reply);
    snprintf(reply, MRL, "streamShowOdo=N      Show odo every N update on console (N is %d - now %d received)\n", streamOdoN, streamOdoCnt);
    sendText(msg, reply);
    snprintf(reply, MRL, "streamShowGps=N      Show gps every N update on console (N is %d - now %d received)\n", streamGpsN, streamGpsCnt);
    sendText(msg, reply);
    snprintf(reply, MRL, "streamShowIns=N      Show INS every N update on console (N is %d - now %d received)\n", streamInsN, streamInsCnt);
    sendText(msg, reply);
/*    snprintf(reply, MRL, "logIns=true|false    log INS data to ins.log (open=%s)\n", bool2str(logINSopen));
    sendText(msg, reply);*/
    sendText(msg,        "help                 This message\n");
    sendText(msg, "-----\n");
    sendText(msg,        "See also             smrConnect (on connect actions)\n");
    if (smrif != NULL)
    { // print also odometry, gps and ins data
      sendText(msg, "-----\n");
      snprintf(reply, MRL, "gps stream %.3fN %.3fE %dsats %.2fdof\n", smrif->gpsVals[0],
               smrif->gpsVals[1], roundi(smrif->gpsVals[2]), smrif->gpsVals[3]);
      sendText(msg, reply);
/*      snprintf(reply, MRL, "ins stream %.4fx %.4fy %.4fz %.4fO %.4fP %.4fK\n", smrif->ins[0],
               smrif->ins[1], smrif->ins[2], smrif->ins[3], smrif->ins[4], smrif->ins[5]);
      sendText(msg, reply);*/
    }
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
/*  else if (smrif == NULL)
  {
    sendWarning(msg, "no resource to do that - try 'module list' for help");
  } */
  else
  { // create the reply in XML-like (html - like) format
    if (smrctl != NULL)
    {
      reply[0] = '\0';
/*      if (false) //gotStart)
      {
        if (gotStartVal)
        {
          if (smrctl->start())
            strncpy(reply, "started", MRL);
          else
            strncpy(reply, "failed - is running already?", MRL);
        }
        else
        {
          smrctl->stop(true);
          strncpy(reply, "stopped", MRL);
        }
      }*/
      if (gotStream)
      {
        gotStreamVal = maxi(1, gotStreamVal);
        if (smrctl->startPoseStreaming(gotStreamVal))
          strncpy(reply, "started", MRL);
        else
          strncpy(reply, "failed - is streaming already? or no smr?", MRL);
      }
      // send reply
      if (strlen(reply) > 0)
      {
        sendInfo(msg, reply);
        handled = true;
      }
    }
    if (smrif != NULL)
    {
      reply[0] = '\0';
      if (gotLogM)
      {
        logIOMode = smrif->getLogMode();
        smrif->setLogMode(gotLogMVal);
        snprintf(reply, MRL, "logfile mode changed from %d to %d", logIOMode, smrif->getLogMode());
      }
      if (gotLog)
      {
        if (gotLogVal)
          smrif->logIO.openLog();
        else
          smrif->logIO.closeLog();
        snprintf(reply, MRL, "logfile (smrcl.log) open=%s", bool2str(smrif->isLogging()));
      }
      if (gotPort)
      {
        snprintf(reply, MRL, "port set to '%d' (from '%d')", port, smrif->getPort());
        smrif->setPort(port);
      }
      if (gotHost)
      {
        snprintf(reply, MRL, "host set to '%s' (from '%s')", host, smrif->getHost());
        smrif->setHost(host);
      }
      if (gotStop)
      {
        snprintf(reply, MRL, "Robot stopping, but is allowed to restart");
        smrif->stopRobot();
      }
      if (gotHup)
      {
        if (smrif->isConnected())
        {
          snprintf(reply, MRL, "disconnected from '%s' port %d", smrif->getHost(), smrif->getPort());
        }
        else
        {
          snprintf(reply, MRL, "not connected to '%s' port %d", smrif->getHost(), smrif->getPort());
        }
        smrif->doDisconnect();
      }
      if (gotConnect)
      {
        isOK = smrif->tryConnect();
        if (isOK)
        {
          snprintf(reply, MRL, "connected to '%s' port %d", smrif->getHost(), smrif->getPort());
        }
        else
          snprintf(reply, MRL, "failed to connected to '%s' port %d", smrif->getHost(), smrif->getPort());
      }
      if (strlen(reply) > 0)
      {
        sendInfo(msg, reply);
        handled = true;
      }
      if (gotMsg or gotGetEvent or gotEval)
      { // command that require an established connection
        if (smrif->isConnected())
        {
          if (gotMsg)
          {
            if (smrif->sendString(smrMsg, "\n"))
            {
              snprintf(reply, MRL, "send %s", smrMsg);
              sendInfo(reply);
            }
            else
            {
              snprintf(reply, MRL, "failed to send %s", smrMsg);
              sendWarning(reply);
            }
            handled = true;
          }
          if (gotEval)
          {
            if (smrif->sendSMReval( smrEval, 0.9, &v, smrMsg, MML))
              snprintf(reply, MRL, "<%s %s=\"%g\"/>\n", msg->tag.getTagName(), smrEval, v);
            else
              snprintf(reply, MRL, "<%s failed=\"true\" variable=\"%s\" reply=\"%s\"/>\n", msg->tag.getTagName(), smrEval, smrMsg);
            sendMsg(msg, reply);
            handled = true;
          }
          if (gotGetEvent)
          {
            snprintf(reply, MRL, "<%s queued=\"%d\" started=\"%d\" finished=\"%d\" syntaxErr=\"%d\"/>\n",
                      msg->tag.getTagName(),
                      smrif->cmdLineQueued, smrif->cmdLineStarted, smrif->cmdLineFinished, smrif->cmdLineSyntaxError);
            sendMsg(msg, reply);
            handled = true;
          }
        }
        else
        {
          sendWarning(msg, "not connected");
          handled = true;
        }
      }
      if (gotStreamOdoN)
      {
        smrif->streamShowOdoEvery = streamOdoN;
        handled = true;
        sendInfo(msg, "done odo every");
      }
      if (gotStreamGpsN)
      {
        smrif->streamShowGpsEvery = streamGpsN;
        handled = true;
        sendInfo(msg, "done gps every");
      }
      if (gotStreamInsN)
      {
        smrif->streamShowInsEvery = streamInsN;
        handled = true;
        sendInfo(msg, "done ins every");
      }
      if (gotLogIns)
      {
        smrif->openINSlog(logIns);
        handled = true;
        sendInfo(msg, "done logIns");
      }
      if (gotLogCtl)
      {
        if (logCtl)
          smrctl->logctl.openLog();
        else
          smrctl->logctl.closeLog();
        handled = true;
        sendInfo(msg, "done logCtl");
      }
      if (gotLogPrim)
      {
        if (logPrim)
          smrctl->logprim.openLog();
        else
          smrctl->logprim.closeLog();
        handled = true;
        sendInfo(msg, "done logPrim");
      }
      if (gotSaveMrcLog)
      {
        smrif->saveMrcLog(true);
        sendInfo(msg, "MRC told to save log and resume logging");
      }
      if (not handled)
        sendWarning(msg, "unhandled command");
    }
    else
      sendWarning(msg, "no interface resource (see module list)");
  }
  // return true if the function is handled with a positive result
  return true;
}

///////////////////////////////////////////////////

const char * UFunctionSmrIf::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s smr interface resource (empty %s)\n",
                  preString, bool2str(smrif == NULL));
  return buff;
}

////////////////////////////////////////////////////

bool UFunctionSmrIf::handleSmrPush(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4List = false;
  const int MRL = 1000;
  char reply[MRL];
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4List = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"SmrConnect - on connect event setup\">\n");
    sendText(msg, "----------------Available SmrConnect (connect event) options:\n");
    sendText(msg, "flush=cmd       Remove 'cmd' command(s) defined by client (default all)\n");
    sendText(msg, "cmd=\"cmd\"       Do execute 'cmd' on connect (on this server)\n");
    sendText(msg, "call=method(pr) Do call method on connect (on this server)\n");
    sendText(msg, "list            List all 'on-connect' commands (as help text)\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (smrif != NULL)
  {
    if (ask4List)
    {
      sendMsg(msg, "<help subject=\"IfConnect command list\">\n");
      smrif->UServerPush::print("camif", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    { // push or flush command
      result = smrif->addPushCommand(msg);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d",
                 smrif->getPushCmdCnt(NULL, NULL));
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
