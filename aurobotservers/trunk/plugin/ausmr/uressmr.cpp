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

#include "uressmr.h"

void UResSmrIf::UResSmrIfInit()
{ // these first two lines is needed
  // to save the ID and version number
/*  setResID(getResID());
  resVersion = getResVersion();*/
  // other local initializations
  poseHist = NULL;
  utmPose = NULL;
  createVarSpace(20, 0, 0, "MRC interface variables and methods", false);
  createBaseVar();
  poseStreaming = false;
  driveUserEventID = 100;
  //logGPS = NULL;
  logGPSuse = true;
  logINS = NULL;
//  driveCommandLast[0] = '\0';
  events = NULL;
}

///////////////////////////////////////////

void UResSmrIf::createBaseVar()
{
  addVar("connected", 0.0, "d", "(ro) Connected to MRC if '1'");
  varIdQueued    = addVar("idQueued", -1.0, "d", "(ro) Last line number queued by MRC");
  varIdStarted   = addVar("idStarted", -1.0, "d", "(ro) last line number started by MRC");
  varIdFinished  = addVar("idFinished", -1.0, "d", "(ro) last line number finished by MRC");
  varIdSyntaxErr = addVar("idSyntaxErr", -1.0, "d", "(ro) Last line that had a syntax error");
  varIdUserEvent = addVar("idUserEvent", -1.0, "d", "(ro) Last user event received from MRC");
  varSpeed =       addVar("speed", 0.4, "d", "Desired drive speed");
  varAcc =         addVar("acc", 0.33, "d", "Used acceleration in MRC commands");
  varTurnAcc =     addVar("turnAcc", 0.33, "d", "Maximum desired turn acceleration");
  varMinTurnRad =  addVar("minTurnRad", 0.10, "d", "Minimum turn radius to be suggested");
  varHakoManual =  addVar("hakomanual", 0.0, "d", "(r) is hako in manual mode (else automatic)");
  varLiftPos    =  addVar("liftPos", -1.0, "d", "(r) is current position of lift (100 is about mid, 80 is high, 120 is low).");
  varPtoSpeed   =  addVar("PTOspeed", -1.0, "d", "(r) is current PTO speed.");
  //
  addMethod("send", "s", "Send this string to MRC");
  addMethod("eval", "s", "Get value of this variable in MRC");
  addMethod("do", "s", "Send this command to the smr connection - "
                "returns 0 if not send or syntax eror, "
                "1 if send and not completed, 2 if send and completed");
  addMethod("do", "sd", "Send this command to the smr connection - as above, but may be used as a control statement. "
          "if second parameter 'd' > 0 then the call just check the completion state (returns 2 for completed)");
  addMethod("event", "d", "Put an event on the MRC queue (if 'd'==0) and return 2 when completed"
                " Returnes 1 if send but not completed and 0 if not send or syntax error. "
                "If 'd'>0 just the test is performed.");
//    vp->addMethod(this, "drive", "c", "Use this maoeuvre sequence as the drive plan");
//    vp->addMethod(this, "drive", "cd", "Use this maoeuvre sequence as the drive plan, with a repeat number (used by sequencer)");
/*  addMethod("pose", "", "Get current pose from smr. Returns a structure,"
      " either a pose,"
      " a posetime, a posev or a 3d position (value is true if data"
      " is received)");*/
  addMethod("addWatch", "ss", "Add a watch to MRC, first parameter"
      " is the watch name, second is the trigger condition");
  addMethod("gotEvent", "d", "Test if a user event 'evX' is received,"
      " where X is the integer part of the provided parameter."
      " Returnes 1 (true) if event was received and 0 (false) if not."
      " The event is removed after the call (if received)");
  addMethod("gotEvent", "dd", "Same as above, but saves the event if parameter 2 is 1 (true)");
  addMethod("gotEventTime", "d", "Test if a user event 'evX' is received"
      " and return the receive time if found. if not found, then 0 (false) is returned."
      " event is removed after the call (if event 'evX' is received)");
  addMethod("eventFlush", "", "Flush all received 'evX' type events.");
  addMethod("putEvent", "d", "Add a 'evX' type events on the MRC command queue,"
      " where X is a positive integer ID number. Returns ID if queued and 0 (false) if not.");
  addMethod("savemrclog", "", "Tell the MRC to save the log with a timestamp - as mrc_yyyymmdd_hhmmss.ccc.logg. Restarts logging after save.");
}

///////////////////////////////////////////

UResSmrIf::~UResSmrIf()
{
  doDisconnect();
  if (logINS != NULL)
    fclose(logINS);
  flushMRCUserEvents();
}

///////////////////////////////////////////

const char * UResSmrIf::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s smrIf connected=%s to port %d\n", preString,
           bool2str(isConnected()), getPort());
  return buff;
}

///////////////////////////////////////////

void UResSmrIf::lineStateUpdated()
{
  varIdQueued->setValued(cmdLineQueued, 0);
  varIdFinished->setValued(cmdLineFinished, 0);
  varIdStarted->setValued(cmdLineStarted, 0);
  varIdSyntaxErr->setValued(cmdLineSyntaxError, 0);
  varIdUserEvent->setValued(cmdLineUserEvent, 0);
}

///////////////////////////////////////////////

void UResSmrIf::connectionChange(bool connected)
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->setLocalVar("connected", connected, false, UVariable::b);
  }
  if (connected)
  { // set two system variables in MRC first
    sendString("smrcl_line_test=1\n");
    // tell push system to execute on-connect commands
    setUpdated("");
  }
  else
  { // not connected anymore
    poseStreaming = false;
  }
}

///////////////////////////////////////////////

bool UResSmrIf::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  bool isUsed = false;
  //
  if (resource->isAlsoA(UCmdExe::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    if (remove)
      UServerPush::setCmdExe(NULL);
    else
    { // resource is also a var-pool resource, so access as so.
      UServerPush::setCmdExe((UCmdExe *) resource);
    }
    result = true;
  }
  else if (resource->isA(UResPoseHist::getOdoPoseID()))
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
  else if (resource->isA(UResPoseHist::getUtmPoseID()))
  {
    result = true;
    if (remove)
      utmPose = NULL;
    else if (utmPose != resource)
      utmPose = (UResPoseHist *)resource;
    else
      // not used
      result = false;
  }
  isUsed = UResVarPool::setResource(resource, remove);
  //
  return result or isUsed;
}

////////////////////////////////////////////////

double UResSmrIf::getSpeed()
{
  return varSpeed->getValued();
}

////////////////////////////////////////////////

double UResSmrIf::getAcc()
{
  return varAcc->getValued();
}

//////////////////////////////////////////////////

bool UResSmrIf::methodCall(const char * name, const char * paramOrder,
                               char ** strings, const double * pars,
                               double * value,
                               UDataBase ** returnStruct,
                               int * returnStructCnt)
{
  bool result = true;
  UPoseTime pt;
  bool isOK;
//  UDataBase * db;
  UPose pose;
  UPoseV pv;
  UPosition pos;
  UTime t;
  // evaluate standard functions
  if ((strcasecmp(name, "send") == 0) and (strcmp(paramOrder, "s") == 0))
  { // bool USmrCl::sendSMR(const char * cmd,
    //       double timeoutSec, const char ** lastReply)
    if (value != NULL)
      *value = sendString(strings[0], "\n");
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "eval") == 0) and (strcmp(paramOrder, "s") == 0))
  { //bool USmrCl::sendSMReval(const char * varName, double timeoutSec, double * value)
    isOK = sendSMReval(strings[0], 0.75, value);
    if (not isOK and verbose)
    { // failed to get a value from smr
      printf("Evaluation of %s failed (timeout or not connected)\n", strings[0]);
    }
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "do") == 0) and (strcmp(paramOrder, "sd") == 0))
  { // drive using a direct smrcl command
    // must return
    //    v = 0 if result can not be evalueated
    //    v = 1 if result is evalueated and the drive is NOT completed
    //    v = 2 if result is evalueated and drive is completed
    *value = doSmrDrive(strings[0], roundi(pars[0]));
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "do") == 0) and (strcmp(paramOrder, "s") == 0))
  { // drive using a direct smrcl command
    *value = doSmrDrive(strings[0], 0);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "event") == 0) and (strcmp(paramOrder, "d") == 0))
  { // put a user event - and wait until the event is puffed of the command queue
    *value = doUserEvent(roundi(pars[0]));
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "putEvent") == 0) and (strcmp(paramOrder, "d") == 0))
  { // put a user event - and wait until the event is puffed of the command queue
    *value = sendUserEvent("ev", roundi(pars[0]));
    if (*value < 0.0)
      *value = 0.0; // return false if not send
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "gotEvent") == 0) and (strcmp(paramOrder, "d") == 0))
  { // test if an "ev" type event is received - and delete if found
    *value = testEvent(roundi(pars[0]), NULL, true);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "gotEvent") == 0) and (strcmp(paramOrder, "dd") == 0))
  { // test if an "ev" type event is received and delete if parameter 2 is false (not saved)
    *value = testEvent(roundi(pars[0]), NULL, roundi(pars[1]) == 0);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "gotEventTime") == 0) and (strcmp(paramOrder, "d") == 0))
  { // get receive time for an "ev" event, and delet if found
    *value = testEvent(roundi(pars[0]), &t, true);
    if (roundi(*value) == 1)
      *value = t.getDecSec();
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "eventFlush") == 0) and (strcmp(paramOrder, "") == 0))
  { // test if an "ev" type event is received and delete if parameter 2 is false (not saved)
    flushMRCUserEvents();
    *value = 1.0;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
/*  else if ((strcasecmp(name, "pose") == 0) and (strcmp(paramOrder, "") == 0))
  { //bool USmrCl::sendSMReval(const char * varName, double timeoutSec, double * value)
    isOK = measureOdo(0.24, &pt);
    if (isOK)
    {
      *value = 1.0;
      if (returnStructCnt != NULL)
      {
        if (*returnStructCnt > 0)
        {
          db = returnStruct[0];
          if (db != NULL)
          {
            *returnStructCnt = 1;
            if (db->isA(pt.getDataType()))
              *(UPoseTime *)db = pt;
            else if (db->isA(pose.getDataType()))
              *(UPose *)db = pt.getPose();
            else if (db->isA(pv.getDataType()))
            {
              pv.set(pt.getPose(), odoState.velocity);
              *(UPoseV *)db = pv;
            }
            else if (db->isA(pos.getDataType()))
            {
              pv.set(pt.getPose(), odoState.velocity);
              *(UPosition *)db = pt.getPos(0.0);
            }
            else
              *returnStructCnt = 0;
          }
          else
            *returnStructCnt = 0;
        }
      }
    }
    else if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }*/
  else if ((strcasecmp(name, "addwatch") == 0) and (strcmp(paramOrder, "ss") == 0))
  { // put a user event - and wait until the event is puffed of the command queue
    *value = sendAddWatch(strings[0], strings[1]);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "savemrclog") == 0) and (strlen(paramOrder) == 0))
  { // put a user event - and wait until the event is puffed of the command queue
    saveMrcLog(true);
    *value = 1.0;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else
  {
    result = false;
  }
  return result;
}

//////////////////////////////////////////////////

double UResSmrIf::getTurnAcc()
{
  return varTurnAcc->getValued();
}

/////////////////////////////////////////////////

double UResSmrIf::getMinTurnRad()
{
  return varMinTurnRad->getValued();
}

//////////////////////////////////////////////////

void UResSmrIf::eventPoseUpdated(bool streamSource)
{ // add this ose to the pose history as needed
  UPoseTVQ pt;
  //
  if (poseHist != NULL)
  {
    pt.set(odoState.pose.x, odoState.pose.y, odoState.pose.h,
           odoState.pose.t, odoState.velocity, 0.0);
    poseHist->addIfNeeded(pt, -4);
  }
  if (not poseStreaming)
    if (streamSource)
      poseStreaming = true;
}

////////////////////////////////////////////////////////

void UResSmrIf::eventInsUpdate(UPoseTime odoState,
                              double accx, double accy, double accz,
                              double roll, double tilt,
                              double pan, double insTime)
{
  UTime t;
  if (logINS != NULL)
  {
    t.now();
    fprintf(logINS, "%.6f %g %g %g  %g %g %g %lu.%06lu\n",
            insTime, accx, accy, accz, roll, tilt, pan,
                  t.getSec(), t.getMicrosec());
  }
}

////////////////////////////////////////////////////////

void UResSmrIf::eventHakoVarUpdate(int hakoManual, int liftPos, int ptoSpeed)
{
  varHakoManual->setValued(hakoManual);
  varLiftPos->setValued(liftPos);
  varPtoSpeed->setValued(ptoSpeed);
}

////////////////////////////////////////////////////////

void UResSmrIf::openINSlog(bool open)
{
  const int MFL = MAX_FILENAME_LENGTH;
  char fn[MFL];
  if (logINS != NULL)
  {
    fclose(logINS);
    logINS = NULL;
    printf("logins %s/ins.log closed\n", dataPath);
  }
  if (open)
  { // open the gpslog
    snprintf(fn, MFL, "%s/ins.log", dataPath);
    logINS = fopen(fn, "w");
    printf("logins %s opened\n", fn);
  }
}

////////////////////////////////////////////////////////

void UResSmrIf::eventGpsUpdate(UPoseTime odoPose,
                              double easting, double northing, double heading,
                              double quality, double satellites,
                              double dop)
{
  UTime t;
  UPoseTVQ pt;
  //
  if (utmPose != NULL)
  {
    pt.x = easting;
    pt.y = northing;
    pt.h = heading; // may be from odo-gps kalman
    pt.t = odoPose.t;
    pt.vel = 0.0;
    pt.q = quality;
    utmPose->addIfNeeded(pt, -11);
  }
  if (not logGPS.isOpen() and logGPSuse)
  { // open the gpslog
    logGPS.setLogName("gpsHako");
    logGPS.openLog();
    if (not logGPS.isOpen())
      // can not open a gps-log file, so do not try again
      logGPSuse = false;
  }
  if (logGPS.isOpen())
  {
    t.now();
    fprintf(logGPS.getF(), "%lu.%06lu %.3f %.3f %f %.0f %.1f %lu.%06lu\n",
            odoPose.t.getSec(), odoPose.t.getMicrosec(),
           easting, northing, quality, satellites, dop,
           t.getSec(), t.getMicrosec());
  }
}

////////////////////////////////////////////////////////

double UResSmrIf::doSmrDrive(const char * cmd, int repeat)
{
  // result = 0 if result can not be evalueated
  // result = 1 if result is evalueated and the drive is NOT completed
  // result = 2 if result is evalueated and drive is completed
  double result;
  //
  if (repeat == 0)
  { // send command once only
    sendString(cmd, "\n");
    //sendUserEvent("drive", 6);
  }
  // place a user event, and test this if repeat is > 0
  result = doUserEvent(repeat);
  return result;
}

//////////////////////////////////////////////////////

double UResSmrIf::doUserEvent(int repeat)
{
  // result = 0 if result can not be evalueated
  // result = 1 if result is evalueated and the drive is NOT completed
  // result = 2 if result is evalueated and drive is completed
  double result;
  int id;
  UTime t;
  //
  if (repeat == 0)
  { // send user event request
    id = sendUserEvent("", -1);
    if (id == driveUserEventID)
      result = 1;
    else
      result = 0;
    if (driveUserEventID == cmdLineUserEvent)
      result = 2;
  }
  else
  { // check if finished )
    if (driveUserEventID == cmdLineUserEvent)
    {
      // debug
      printf("userevent %d detected at %lu.%06lu\n",
             driveUserEventID, t.getSec(), t.getMicrosec());
      // debug end
      result = 2;
    }
    else
      result = 1;
  }
  return result;
}

///////////////////////////////////////////////////

int UResSmrIf::sendUserEvent(const char * evPreString, int evNumber)
{
  bool isOK;
  int result;
  const int MSL = 200;
  char s[MSL];
  //
  if (evNumber < 0)
  {
    result = ++driveUserEventID;
  }
  else
    result = evNumber;
  //
  snprintf(s, MSL, "putevent \"%s%d\"\n", evPreString, result);
  //isOK = sendSMRgetId(driveCommandLast, 1.0, &driveCommandID, true);
  isOK = sendString(s);
  if (not isOK)
    result = -1;
  //
  return result;
}

///////////////////////////////////////////////////

bool UResSmrIf::sendAddWatch(const char * name, const char * condition)
{
  bool isOK;
  const int MWL = 100;
  char w[MWL];
  const int MSL = 30;
  char s[MSL];
  double d;
  //
  snprintf(s, MSL, "%s.event", name);
  if (not getLocalValue(s, &d))
  { // no such variable, so create
    setLocalVar(s, 0.0, true);
    snprintf(s, MSL, "%s.time", name);
    setLocalVar(s, 0.0, true);
  }
  snprintf(w, MWL, "addwatch \"%s\" \"type\" : (%s)", name, condition);
  isOK = sendString(w, "\n");
  //
  return isOK;
}

////////////////////////////////////////////

void UResSmrIf::eventWatchFired(const char * name, double atTime)
{
  double d;
  const int MSL = 30;
  char s[MSL];
  //
  snprintf(s, MSL, "%s.event", name);
  if (getLocalValue(s, &d))
    setLocalVar(s, d + 1.0, false);
  else
    setLocalVar(s, 1.0, true);
  snprintf(s, MSL, "%s.time", name);
  setLocalVar(s, atTime, true);
}

/////////////////////////////////////////////

void UResSmrIf::stopRobot()
{
  if (connected)
  { // stop the robot
    sendString("flushcmds\n");
    sendString("stop\n");
    sendString("idle\n");
  }
}

//////////////////////////////////////////

void UResSmrIf::closeConnection()
{
  if (connected)
  { // stop the robot
    stopRobot();
    // logoff and close socket
    USmrCl::closeConnection();
  }
}

////////////////////////////////////////

void UResSmrIf::gotUserEvent(const char * eventString)
{
  int evID;
  const char * p1;
  char *p2;
  UMRCUserEvent * ev;
  //
  p1 = eventString;
  // advance past keyword "ev"
  p1 += 2;
  evID = strtol(p1, &p2, 0);
  if (p1 != p2)
  { // a valid event is detected
    ev = new UMRCUserEvent();
    ev->eventTime.now();
    ev->id = evID;
    eventsLock.lock();
    ev->next = events;
    events = ev;
    eventsLock.unlock();
  }
}

////////////////////////////////////////

void UResSmrIf::flushMRCUserEvents()
{
  UMRCUserEvent * ev, *de;
  //
  ev = events;
  events = NULL;
  while (ev != NULL)
  {
    de = ev;
    ev = ev->next;
    delete de;
  }
}

//////////////////////////////////////////////

bool UResSmrIf::testEvent(int evID, UTime * eventTime, bool removeIfFound)
{
  UMRCUserEvent * ev, *pe = NULL;
  bool result = false;
  //
  eventsLock.lock();
  ev = events;
  while (ev != NULL)
  {
    if (ev->id == evID)
      break;
    pe = ev;
    ev = ev->next;
  }
  if (ev != NULL)
  {
    result = true;
    if (eventTime != NULL)
      *eventTime = ev->eventTime;
    if (removeIfFound)
    {
      if (pe == NULL)
        // first element
        events = ev->next;
      else
        // bypass the ev event
        pe->next = ev->next;
      delete ev;
    }
  }
  eventsLock.unlock();
  return result;
}

///////////////////////////////////////////////


/////////////////////////////////////////////


