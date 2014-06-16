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

#include "urescron.h"

///////////////////////////////////////////

UCronJob::~UCronJob()
{
  if (pf != NULL)
    pclose(pf);
}

////////////////////////////////////////////

int UCronJob::runJob()
{
  const int MSL = 500;
  char s[MSL];
  int i = 0, n;
  char * p1, *p2;
  const int MRL = MAX_RES_STR_SIZE - 1;
  bool buffFull = false;
  //
  tStart.now();
  pf = popen(cmdStr, "r");
  resStr[MRL] = '\0';
  if (pf != NULL)
  {
    active = true;
    n = 0;
    p2 = resStr;
    stopJob = false;
    while (not feof(pf) and not stopJob)
    {
      p1 = fgets(s, MSL, pf);
      if (p1 != NULL and strlen(s) > 0 and not buffFull)
      { // copy result to buffer - up to max length
        strncpy(p2, p1, MRL - n);
        n += strlen(p1);
        if (n >= MRL - 18)
        { // truncate if needed
          n = MRL - 18;
          p2 = &resStr[n];
          strncpy(p2, " ... (truncated)", 18);
          // drop the rest
          buffFull = true;;
        }
        resStr[n] = '\0';
        p2 = &resStr[n];
      }
      i++;
    }
    resInt = pclose(pf);
    pf = NULL;
    reuseCnt++;
    if (resInt != 0)
      errCnt++;
  }
  else
    resInt = -1;
  pipeTime = tStart.getTimePassed();
  if (jobLog != NULL)
  { // put into logfile
    snprintf(s, MSL, "took %.3f ms, result (%d) $ %s:\n", pipeTime * 1000.0, resInt, cmdStr);
    jobLog->toLog(s, resStr);
  }
  active = false;
  return (resInt);
}

/////////////////////////////////////////////

const char * UCronJob::getResultStr(char * buff, const int buffCnt)
{
  int n;
  char * p1 = buff;
  //
  *p1 = '\0';
  if (buffCnt > 12)
    tStart.getTimeAsString(p1, true);
  n = strlen(p1);
  p1 = &buff[n];
  if (n > 4)
  {
    if (active)
    {
      snprintf(p1, buffCnt - n, "  is active: %s\n", cmdStr);
    }
    else
    {
      snprintf(p1, buffCnt - n, " took %.3f ms (%d) $ %s:\n",
            pipeTime * 1000.0, resInt, cmdStr);
      n += strlen(p1);
      p1 = &buff[n];
      strncpy(p1, resStr, buffCnt - n);
    }
  }
  return buff;
}

/////////////////////////////////////////////

void * startCronJobThread(void * obj)
{ // call the hadling function in provided object
  UCronJob * ce = (UCronJob *)obj;
  ce->runJob();
  pthread_exit((void*)NULL);
  return NULL;
}

//---------------------------------------------

bool UCronJob::start()
{
  pthread_attr_t  thAttr;
  //
  if (not active)
  {
    pthread_attr_init(&thAttr);
    //
    // create socket server thread
    active = (pthread_create(&threadHandle, &thAttr,
              &startCronJobThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return active;
}

/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

void UResCron::UResCronInit()
{
  int i;
  // to save the ID and version number
  setLogName("cron");
  openLog();
  // allow individual jobs to write in the log too
  for (i = 0; i < MAX_ACTIVE_CRON_JOBS; i++)
    job[i].jobLog = this;
  // create status variables
  createBaseVar();
  // the this class execute the push jobs
  cmds.setCmdExe(this);
  jobCnt = 0;
  jobLast = -1;
  threadRunning = false;
  verbose = false;
  // start job starter thread
  start();
}

///////////////////////////////////////////

UResCron::~UResCron()
{
  // stop job starter thread
  stop(true);
  // the jobs are terminated in the job destructor
}

///////////////////////////////////////////

const char * UResCron::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int n = 0;
  char * p1 = buff;
  const int MSL = 50;
  char s[MSL];
  UTime lastTime; // should be replaced by something more appropriate
  //
  snprintf(p1, buffCnt - n, "%s fast scheduled commands - @todo more verbose\n", preString);
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "    Last command run at at %s\n", s);
  return buff;
}

///////////////////////////////////////////

void UResCron::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    varIdleCnt  = addVar("idleCnt",  0.0, "d", "(r) idle loops with no commands");
    varJobCnt   = addVar("jobCnt",   0.0, "d", "(r) number of commands run");
    varErrCnt   = addVar("errCnt",   0.0, "d", "(r) number of commands returned non success");
    varLastTime = addVar("exeTime",  0.0, "d", "(r) execution time of last command");
    varCronTime = addVar("cronTime", 1.0, "d", "(r/w) idle count interval (maximum) when testing scheduled tasks");
  }
}

//////////////////////////////////////////////

bool UResCron::methodCall(const char * name, const char * paramOrder,
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

void UResCron::run()
{ //
  bool isOK;
  threadRunning = true;
  double sampleTime;
  UTime t, tn;
  double wt;
  int ej, n = 0;
  // wait to allow init script to finish
  Wait(1.2);
  // debug
  // printf("Cron job starter started\n");
  // debug end
  while (not threadStop)
  {
    t.Now();
    isOK = handleOnePushCmd();
    if (isOK)
    { // just finished one job, but do not occupy all cpu resources
      varJobCnt->add(1.0);
      Wait(0.01);
    }
    else
    { // idle, so update local variables if needed
      ej = getDoneJobsErrCnt();
      varErrCnt->setInt(ej, 0);
      //
      sampleTime = varCronTime->getValued();
      t.now();
      tn = cmds.getPushQueue()->getNextExeTime();
      // get time to next scheduled event - may be negative or wery long
      wt = tn - t;
      // limit to between 10 ms and sample time (default is 1 sec)
      wt = fmax(0.01, fmin(sampleTime, wt));
      varIdleCnt->add(1.0, 0);
      // wait to next sample time has passed
      Wait(wt);
    }
    n++;
    // debug
    if (verbose)
      printf("Cron job %d, iteration %d - commands %d active %d!!\n",
             varJobCnt->getInt(), n,
             cmds.getPushCmdCnt(NULL, NULL), getActiveCnt());
    // debug end
  }
  threadRunning = false;
  // debug
  // printf("Cron job starter terminated!!\n");
  // debug end
}

///////////////////////////////////////////////////

void * startCronLoopThread(void * obj)
{ // call the hadling function in provided object
  UResCron * ce = (UResCron *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResCron::start()
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
              &startCronLoopThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UResCron::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////

int UResCron::findFunctionOwner(const char * tagName)
{ // one function owner only (me)
  return 1;
}

//////////////////////////////////////////////////////

int UResCron::getResultStringsCnt()
{
  int i, n;
  UCronJob * cj;
  //
  n = 0;
  cj = job;
  for (i = 0; i < jobCnt; i++)
  {
    if (cj->reuseCnt > 0 and not cj->active)
      n++;
    cj++;
  }
  return n;
}

//////////////////////////////////////////////////////

// bool UResCron::executePushFunction(int functionIndex, UServerInMsg * msg, void * extra)
// {
//   UCronJob * cj;
//   //
//   cj = getFreeJob();
//   if (cj != NULL)
//   {
//     cj->active = true;
//     cj->cmdStr = msg->message;
//     startCronJobThread(cj);
//   }
//   return (cj != NULL);
// }

//////////////////////////////////////////////////////////

UCronJob * UResCron::getFreeJob()
{
  UCronJob * result = NULL;
  int i, n = 0;
  //
  for (i = 1; i <= MAX_ACTIVE_CRON_JOBS; i++)
  {
    n = (jobLast + i) % MAX_ACTIVE_CRON_JOBS;
    if (n >= jobCnt)
      break;
    if (not job[n].active)
    {
      result = &job[n];
      jobLast = n;
      break;
    }
  }
  if (result == NULL and n < MAX_ACTIVE_CRON_JOBS)
  {
    result = &job[n];
    jobLast = n;
    jobCnt++;
  }
  return result;
}

/////////////////////////////////////////////////////////

int UResCron::getDoneJobsCnt()
{
  int result = 0;
  int i;
  UCronJob * cj = job;
  //
  for (i = 0; i < jobCnt; i++)
  {
    result += cj->reuseCnt;
    cj++;
  }
  return result;
}

/////////////////////////////////////////////////////////

int UResCron::getDoneJobsErrCnt()
{
  int result = 0;
  int i;
  UCronJob * cj = job;
  //
  for (i = 0; i < jobCnt; i++)
  {
    result += cj->errCnt;
    cj++;
  }
  return result;
}

/////////////////////////////////////////////////////////

const char * UResCron::getResultStr(int prev, char * buff, const int buffCnt)
{
  int i;
  UCronJob * cj;
  int n = 0, k, m;
  char * p1 = buff;
  //
  *p1 = '\0';
  if (jobCnt == 0)
  { // no data
    snprintf(p1, buffCnt - n, "0/0 No cron jobs performed yet");
  }
  else
  { // get number of jobs to list
    m = mini(jobCnt, maxi(1, prev));
    // get the oldest
    k = jobLast - m + 1;
    if (k < 0)
      k += MAX_ACTIVE_CRON_JOBS;
    for (i = 0; i < jobCnt; i++)
    { // do at maximum 'jobCnt' strings
      cj = &job[k];
      // advance to end of current string
      n += strlen(p1);
      p1 = &buff[n];
      if (buffCnt - n < 40)
      { // mark as continued
        n = mini(buffCnt - 18, n);
        p1 = &buff[n];
        strncpy(p1, " ... (truncated)", buffCnt - n);
        break;
      }
      // print job number
      snprintf(p1, buffCnt - n, "%d/%d ", i, jobCnt);
      n += strlen(p1);
      p1 = &buff[n];
      // append result string
      cj->getResultStr(p1, buffCnt - n);
      // finished when last job is reached
      if (k == jobLast)
        break;
      // advance to next
      k++;
      if (k >= MAX_ACTIVE_CRON_JOBS)
        k = 0;
    }
  }
  return buff;
}

///////////////////////////////////////////////////////////////

bool UResCron::handleOnePushCmd()
{
  bool result = false;
  UServerPushElement * qe;
  UCronJob * cj;
  UTime t;
  int na, nj;
  bool isBusy;
  //
  // if there is more active jobs than push elements
  // then the jobs take longer than the test intervel, make
  // an error and do not start the new job
  na = getActiveCnt();
  nj = cmds.getPushCmdCnt(NULL, NULL);
  isBusy = (na >= nj and na > 0);
  if (isBusy)
  {
    if (verbose)
      printf("More jobs active (%d) than push commands (%d) - refuse to start another!\n", na, nj);
    toLog("# too busy", na, nj, "dooing active jobs relative to number of push jobs");
  }
  //
  if (not isBusy and nj > 0)
  {
    qe = cmds.getPushQueue()->getNextTimedPushElement();
    if (qe != NULL)
    {
      if (isClientAlive(qe->toDo.client, 0))
      { // alive, so start job
        cj = getFreeJob();
        cj->cmdStr = qe->toDo.message;
        if (cj != NULL)
        { // start job in own thread
          cj->start();
          // update the push queue status
          qe->update(true);
          // debug
          if (verbose)
            printf("cron just started job: %s\n", cj->cmdStr);
          // debug end
          // set time of last start of command
          t.now();
          varLastTime->setTime(t);
        }
      }
      else
      { // client is dead - flush the pending commands
        qe->activeCmd = false;
        qe->activeCall = false;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////////////

int UResCron::getActiveCnt()
{
  int result = 0;
  int i;
  UCronJob * cj = job;
  //
  for (i = 0; i < jobCnt; i++)
  {
    if (cj->active)
      result++;
    cj++;
  }
  return result;
}


