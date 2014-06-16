/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>

#include "ureskeep.h"


UKeepItem::UKeepItem()
{
  //cnn = NULL;
  tickCnt = 0;
  eventOn = new UServerPush();
  eventOff = new UServerPush();
  core = NULL;
  logfile = NULL;
}

UKeepItem::~UKeepItem()
{
  if (eventOff != NULL)
    delete eventOff;
  if (eventOn != NULL)
    delete eventOn;
  //if (cnn != NULL)
  //  delete cnn;
}

////////////////////////////////////////////////////

bool UKeepItem::tick()
{ //
  double d, ti;
  bool result = false;
  bool nowON = false;
  bool nowOff = false;
  bool nowUnknown = false;
  int oldState;
  int newState;
  const char * p1 = NULL;
  //
  if (not isWait())
  {
    d = tickTime.getTimePassed();
    oldState = varRunningOK->getValueInt();
    // wait a normal tick time
    ti = varTickInterval->getValued();
    if (d > ti)
    {
      newState = oldState;
      if (oldState != 1)
        nowON = monitorTest(1, NULL);
      if (oldState != 0)
        nowOff = monitorTest(0, NULL);
      if (oldState != -1)
        nowUnknown = monitorTest(-1, NULL);
      if (oldState == 0 and not (nowON or nowUnknown))
        // from off detected allow this time before new off event is triggered
        ti = varAllow->getValued();
      d = offTime.getTimePassed();
      if (d > ti)
      {
        if (nowUnknown)
          newState = -1;
        else if (nowON)
          newState = 1;
        else if (nowOff)
          newState = 0;
        if (oldState != newState or (newState==0 and not varAllowOff->getBool()))
        { // there is a change - off is continued passed allowed time
          if (newState == 1)
            // turned on
            eventOn->setUpdated("1");
          else if (newState == 0)
          { // turned off
            eventOff->setUpdated("0");
            offTime.now();
            varOffCnt->add(1.0);
          }
          varRunningOK->setInt(newState);
          // log
          switch (newState)
          {
            case -1: p1 = varUnknownExpr->getValues(); break;
            case 0:  p1 = varOffExpr->getValues(); break;
            case 1:  p1 = varOnExpr->getValues(); break;
            default: p1 = "error"; break;
          }
          logfile->toLog(varKeepName->getValues(),oldState, newState,p1);
        }
        tickCnt++;
        varMonitorCnt->add(1.0);
        result = true;
        tickTime.now();
      }
    }
  }
  return result;
}

/////////////////////////////////////////

bool UKeepItem::createBaseVar()
{
  varKeepName = addVarA("name", "noname", "s", "(r/w) Name of the keep process");
  varOnExpr  = addVarA("onExpr", "false", "s", "(r/w) Expression that should "
                                               "evaluate to true if process is running.");
  varOffExpr  = addVarA("offExpr", "false", "s", "(r/w) Expression that should "
      "evaluate to true if process is running.");
  varUnknownExpr  = addVarA("unknownExpr", "false", "s", "(r/w) Expression that should "
      "evaluate to true if process is running.");
  varWait  = addVar("wait", 1.0, "d", "(r/w) Wait flag. When waiting no monitoring takes place"
      "and no actions are taken. when wait=false, monitoring is restarted.");
  varAllow = addVar("allow", 15.5, "d", "(r/w) Allow this many seconds after start before monitoring starts.");
  varAllowOff = addVar("allowOff", 0.0, "d", "(r/w) Allow off to be permanent (no new off-events).");
  varRunningOK = addVar("runningOK", -1.0, "d", "(r/w) Is the process running at last monitoring");
  varMonitorCnt = addVar("monitorCnt", 0.0, "d", "(r) number of monitor scans.");
  varOffCnt = addVar("offCnt", 0.0, "d", "(r) number of off detections.");
//  varStopCmd = addVarA("stopCmd", "q", "s", "(r/w) The command that will stop the process");
//  varStartCmd = addVarA("startCmd", "do ./noname >null", "s", "(r/w) The command thad will start the process to keep");
  varTickInterval = addVar("monitorInterval", 1.0, "d", "(r/w) the time between monitoring actions - in seconds");
  return true;
}

///////////////////////////////////////////

bool UKeepItem::isA(const char * name)
{
  bool result = false;
  //
  if (varKeepName != NULL)
  {
    result = (strcasecmp(varKeepName->getValues(0), name) == 0);
  }
  return result;
}

///////////////////////////////////////////

void UKeepItem::setName(const char * name)
{
  if (varKeepName != NULL)
  {
    varKeepName->setValues(name, 0, true);
  }
}


///////////////////////////////////////////

void UKeepItem::setExpr(int expFor, const char * expr)
{
  switch (expFor)
  {
    case -1:
      if (varUnknownExpr != NULL)
        varUnknownExpr->setValues(expr, 0, true);
      break;
    case 0: // off expression
      if (varOffExpr != NULL)
        varOffExpr->setValues(expr, 0, true);
      break;
    case 1: // on expression
      if (varOnExpr != NULL)
        varOnExpr->setValues(expr, 0, true);
      break;
    default: break;
  }
}

///////////////////////////////////////////

void UKeepItem::setAllow(double secs)
{
  if (varAllow != NULL)
  {
    varAllow->setDouble(secs, 0);
  }
}

///////////////////////////////////////////

void UKeepItem::setAllowOff(bool value)
{
  if (varAllowOff != NULL)
  {
    varAllowOff->setBool(value, 0);
  }
}

///////////////////////////////////////////

void UKeepItem::setWait(bool value)
{
  if (varWait != NULL)
  {
    varWait->setBool(value);
  }
}

///////////////////////////////////////////

void UKeepItem::setCore(UCmdExe * pCore)
{
  setImplementor(pCore);
  eventOff->setImplementor(pCore);
  eventOn->setImplementor(pCore);
  core = pCore;
}

///////////////////////////////////////////

bool UKeepItem::monitorTest(int exprFor, bool * evaluated)
{
  bool result = false;
  const char * expr;
  char * p1;
  const char * p2;
  int n;
  double d;
  UDataBase * reply = NULL;
  UDataString * rs;
  //
  switch (exprFor)
  {
    case -1:
       expr = (char*)varUnknownExpr->getValues();
       break;
    case 0:
      expr = (char*)varOffExpr->getValues();
      break;
    case 1:
      expr = (char*)varOnExpr->getValues();
      break;
    default:
      expr = "false";
  }
  p2 = expr;
  while (isspace(*p2))
    p2++;
  p2 = strchr(p2, ' ');
  if (p2 > NULL)
    n = p2 - expr;
  else
    n = strlen(expr);
  if (n > 0)
  {
    if (strncmp(expr, "bash", n) == 0)
    { // a bash command
      rs = new UDataString();
      reply = rs;
      n = 1; // class count
      p1 = (char *) &expr[5];
      result = callGlobal("core.bash", "s", &p1, NULL, &d, &reply, &n);
      // debug
      // printf("Result is (%d) returns %i and string='%s'\n", result, roundi(d), ((UDataString*)reply)->getStr());
      // debug end
      result = (roundi(d) == 0);
    }
    else
    {
      if (evaluated != NULL)
        printf("Trying to evaluate '%s' ...\n", expr);
      d = evaluateD(expr, expr, NULL, evaluated, false);
      if (evaluated != NULL)
      { // syntax check requested
        if (not *evaluated)
          printf("... expression failed: %s\n", errorTxt);
        else
          // evaluated OK
          printf("... result is %g\n", d);
      }
      result = (d > 0.5);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

const char * UKeepItem::print(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt, "%s name %s (waiting %s)\n", preStr, varKeepName->getValues(), bool2str(varWait->getBool()));
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "     - ONexpr='%s'\n", varOnExpr->getValues());
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "     - OFFexpr='%s'\n", varOffExpr->getValues());
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "     - UNKNOWNexpr='%s'\n", varUnknownExpr->getValues());
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "     - allow=%gs every=%gs state=%d\n",
           varAllow->getValued(), varTickInterval->getValued(),
           varRunningOK->getInt());
  //
  return buff;
}

///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////


void UResKeep::UResKeepInit()
{
  setLogName("keep");
  openLog();
  // create status variables
  createBaseVar();
  threadRunning = false;
  verbose = false;
  eventOff = NULL;
  keepsCnt = 0;
  // start monitor thread
  start();
}

///////////////////////////////////////////

UResKeep::~UResKeep()
{
  // stop thread
  stop(true);
  if (eventOff != NULL)
    delete eventOff;
}

///////////////////////////////////////////

void UResKeep::createResources()
{
  UCmdExe * core;
  //
  core = getCorePointer();
  if (eventOff == NULL and core != NULL)
  {
    eventOff = new UServerPush();
    eventOff->setCmdExe(core);
  }
}

///////////////////////////////////////////

const char * UResKeep::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  //
  getKeepList(preString, buff, buffCnt);
  return buff;
}

///////////////////////////////////////////

void UResKeep::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    varIdleCnt  = addVar("idleCnt",  0.0, "d", "(r) idle loops with no commands");
    varErrCnt   = addVar("errCnt",   0.0, "d", "(r) number of commands returned non success");
    varKeepTime = addVar("interval", 0.2, "d", "(r/w) Time between keep tests in seconds");
    varWait = addVar("wait", 0.0, "d", "(r/w) Global pause flag - should all keep items be paused");
  }
}

//////////////////////////////////////////////

bool UResKeep::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  // evaluate standard functions
/*  if ((strcasecmp(name, "heading") == 0) and (strlen(paramOrder) == 0))
  {
    // calculate result
    h = getHeading();
    // return result - if a location is provided
    if (value != NULL)
      *value = h;
      // it is goot praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "distance") == 0) and (strlen(paramOrder) == 0))
  {
    // calculate result
    d = getDistance();
    // return result - if a location is provided
    if (value != NULL)
      *value = d;
      // it is goot praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else
    // call name is unknown
    result = false;
  */
  return result;
}

//////////////////////////////////////////////////

void UResKeep::run()
{ //
  threadRunning = true;
  double sampleTime;
  UTime t;
  int i, n = 0;
  UKeepItem ** ki;
  // wait to allow init script to finish
  Wait(1.2);
  while (not threadStop)
  {
    t.Now();
    lock();
    // add some functionality
    if (not varWait->getBool())
    {
      ki = keeps;
      for (i = 0; i < keepsCnt; i++)
      {
        (*ki)->tick();
        ki++;
      }
    }
    unlock();
    { // wait a sample time
      sampleTime = varKeepTime->getValued();
      // do not allow faster than 10ms
      sampleTime = mind(sampleTime, 0.01);
      // wait until next sample time
      Wait(sampleTime);
    }
    n++;
  }
  threadRunning = false;
}

///////////////////////////////////////////////////

void * startKeepLoopThread(void * obj)
{ // call the hadling function in provided object
  UResKeep * ce = (UResKeep *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResKeep::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
              &startKeepLoopThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UResKeep::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////

UServerPush * UResKeep::getPushHandle(const char * process, bool eventOn)
{
  UServerPush * result = NULL;
  //
  if (strlen(process) == 0)
  {
    if (eventOn)
      result = this;
    else
      result = eventOff;
  }
  else
  { // an active process
  }
  return result;
}

///////////////////////////////////////////////////////////

UKeepItem * UResKeep::add(const char * name)
{
  UKeepItem * result = NULL;
  UVarPool * varPool, *vp;
  //
  result = getKeepItem(name);
  if (result == NULL and keepsCnt < MAX_KEEPS_CNT)
  { // create new keep
    varPool = getVarPool();
    // test if there already ...
    vp = varPool->getStruct(name);
    if (vp != NULL)
      // delete if there already
      varPool->deleteStruct(name);
    //
    // make new keep item - is also a varPool
    result = new UKeepItem();
    // add a new substructure to keep with parameters to theis keep
    varPool->addStruct(name, result);
    // set who to inform about events
    result->setCore(getCorePointer());
    // create keep parameters
    result->createBaseVar();
    // fill in the new values
    result->setName(name);
    // set logfile to this resource logfile
    result->setLogFile(this);
    //result->setExpr(expr);
    // save in list of keeps
    keeps[keepsCnt++] = result;
  }
  else
    result = NULL;
  //
  return result;
}

//////////////////////////////////////////////

UKeepItem * UResKeep::getKeepItem(const char * name)
{
  int i;
  UKeepItem ** ki = keeps;
  UKeepItem * result = NULL;
  //
  for (i = 0; i < keepsCnt; i++)
  {
    if ((*ki)->isA(name))
    {
      result = *ki;
      break;
    }
    ki++;
  }
  return result;
}

//////////////////////////////////////////////

bool UResKeep::deleteKeep(const char * name)
{
  UVarPool * vp;
  int i;
  UKeepItem ** kis = keeps;
  UKeepItem * ki = NULL;
  //
  lock();
  for (i = 0; i < keepsCnt; i++)
  {
    if ((*kis)->isA(name))
    {
      ki = *kis;
    }
    if (ki != NULL and i < keepsCnt - 1)
      // move item one down
      kis[0] = kis[1];
    kis++;
  }
  if (ki != NULL)
    keepsCnt--;
  unlock();
  if (ki != NULL)
  {
    delete ki;
    // get var pool, with the parameters for the deleted keep
    vp = getVarPool();
    // delete the variables for the keep
    vp->deleteStruct(name);
  }
  return false;
}

/////////////////////////////////////////////

const char * UResKeep::getKeepList(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  int i;
  UKeepItem ** ki = keeps;
  //
  snprintf(p1, buffCnt, "%s - %d items (global wait %s)\n", preStr, keepsCnt, bool2str(varWait->getBool()));
  n += strlen(p1);
  p1 = &buff[n];
  for (i = 0; i < keepsCnt; i++)
  {
    (*ki)->print("  - ", p1, buffCnt - n);
    n += strlen(p1);
    p1 = &buff[n];
    ki++;
  }
  return buff;
}

////////////////////////////////////////////

void UResKeep::setWait(bool value)
{
  if (varWait != NULL)
  {
    varWait->setBool(value);
  }
}

