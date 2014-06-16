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

//#include <urob4/ucmdexe.h>

#include "useqloop.h"
//#include "srmain.h"

void *startSequencer(void *ptr)
{
  USeqLoop * obj;
  // convert pointer to planner object
  obj = (USeqLoop *) ptr;
  // run sequencer loop
  obj->run();
  //
  return NULL;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

USeqLoop::USeqLoop()
        : USequencer()
{
  const int MSL = 500;
  char s[MSL];
  //
  terminate = false;
  running = false;
  sampleTime = 0.125;
  snprintf(s, MSL, "%s/seq.log", dataPath);
  logSeq = fopen(s, "w");
  //cmdexe = NULL;
  reset();
  start();
}

////////////////////////////////////////////////////

USeqLoop::~USeqLoop()
{
  if (running)
  { // stop and wait for loop
    terminate = true;
    pthread_join(threadLoop, NULL);
  }
  if (logSeq != NULL)
    fclose(logSeq);

}

////////////////////////////////////////////////////

void USeqLoop::reset()
{
  USequencer::reset();
  lineParsed = -1;
  idle = true;
}

/////////////////////////////////////////////////////

void USeqLoop::clear()
{
  seqLock.lock();
  reset();
  seqLock.unlock();
}

////////////////////////////////////////////////////

// bool USeqLoop::sendToAll(const char * message, int lockedUser)
// {
//   bool result = false;
//   if (cmdexe != NULL)
//     result = cmdexe->sendMsgAll(message, lockedUser);
//   return result;
// }

////////////////////////////////////////////////////

// bool USeqLoop::sendInfoToAll(const char * message, int lockedUser)
// {
//   const int MRL = 500;
//   char buff[MRL];
//   char reply[MRL];
//   // pack as XML
//   str2xml(buff, MRL, message);
//   snprintf(reply, MRL, "<push text=\"%s\"/>\n", buff);
//   // send
//   return sendToAll(reply, lockedUser);
// }

////////////////////////////////////////////////////

bool USeqLoop::start()
{
  bool result = true;
  pthread_attr_t  thConAttr;
  if (not running)
  { // Starts socket server thread 'runSockServer'
    pthread_attr_init(&thConAttr);
    // disable stop flag
    terminate = false;
    // create socket server thread
    pthread_create(&threadLoop, &thConAttr, &startSequencer, (void *)this);
    pthread_attr_destroy(&thConAttr);
  }
  return result;
}

////////////////////////////////////////////////////

void USeqLoop::run()
{
  UTime t;
  double dt;
  const int MSL = 40;
  char st[MSL];
  char sd[MSL];
  const int MRL = 200;
  char reply[MRL];
  int nextLine;
  USeqLine * sl = NULL;
  bool isOK;
  bool outOfLines = false;
//  bool smrCLsyntaxErr = false;
  //
  loopStartTime.Now();
  running = true;
  if (logSeq != NULL)
  {
    t.Now();
    t.getTimeAsString(st, true);
    t.getDateString(sd, true);
    fprintf(logSeq,"%lu.%03lu (%s %s) seqLoop started\n",
            t.getSec(), t.getMicrosec(), sd, st);
  }
  //
  setActiveLine(0); // reset inDriveCmd
  nextLine = 0; // start with line 0
  idle = false;
  // wait for system to be available
  while (calc != NULL and not calc->isSysVarsDefined())
    Wait(0.1);
  //
  while(not terminate)
  { // run main loop once
    seqLock.lock();
    sl = getLine(lineActive);
    nextLine = lineActive;
    skipLine.clear();
    if (idle)
    { // a new sample time
      loopStartTime.Now();
      // make all scheduled watches
      if (not simulated)
        doWatches(loopStartTime);
    }
    if (idle and (skipLine.isASkipLine()))
    { // execute the waiting skip line
      if (parseSkipStatement(&skipLine, -2, logSeq, false, &nextLine))
      { // set the skip-to line as the new active line
        if (nextLine != lineActive)
        {
          setActiveLine(nextLine);
          calc->setDriveMode(DRIVE_MODE_IDLE);
        }
        // get the line
        sl = getLine(lineActive);
        // finish the idle state
        idle = false;
      }
      else
      {
        if (logSeq != NULL)
          fprintf(logSeq, "*** %d error: parsing (skip) failed '%s'\n", lineActive, sl->getCmdLine());
        printf("USeqLoop::run: parsing skip failed line %d - '%s'\n", lineActive, sl->getCmdLine());
      }
    }
    if (simulated)
    {
      nextLine = getSimLine(calc->getReplayTime());
      if (nextLine != lineActive)
        setActiveLine(nextLine);
      sl = getLine(lineActive);
    }
    // parse if not parsed already
    if (lineActive < linesCnt)
    { // parse next statement - no logfile
      if (lineParsed != lineActive)
      { // is a new line, so make a statement
        // debug
        printf("USeqLoop::run: %d Parsing '%s'\n", lineActive, sl->getCmdLine());
        // debug end
        lineParsed = lineActive;
        outOfLines = false;
      }
      isOK = parseStatement(sl, lineActive, logSeq, false, &nextLine);
      if (not isOK)
      {
        nextLine = lineActive + 1;
        snprintf(reply, MRL, "*** %d error: parsing failed, skipping '%s'"
                           , lineActive, sl->getCmdLine());
        if (logSeq != NULL)
        {
          t.Now();
          fprintf(logSeq, "%lu.%03lu %s\n", t.getSec(), t.getMilisec(), reply);
        }
        printf("USeqLoop::run: %s\n", reply);
        sendInfoToAll(reply, -1);
      }
    }
    else if ((lineActive >= linesCnt) and not outOfLines)
    { // end of program stop robot
      calc->setDriveMode(DRIVE_MODE_IDLE);
      snprintf(reply, MRL, "** %d warning: sequencer out of mission lines "
                           "(try \"seq help\")", lineActive);
      printf("USeqLoop::run: %s\n", reply);
      if (logSeq != NULL)
      {
        t.Now();
        fprintf(logSeq, "%lu.%03lu %s\n", t.getSec(), t.getMilisec(), reply);
      }
      sendInfoToAll(reply, -1);
      outOfLines = true;
    }
    // if no advance, then just idle
    idle = (lineActive == nextLine);
    if (not idle and not simulated)
      setActiveLine(nextLine);
    seqLock.unlock();
    //
    if (terminate)
      break;
    // calculate suspend time
    if (idle)
    { // wait for sample time, but only if idle
      t.Now();
      dt = sampleTime - (t - loopStartTime);
      // ensure a minimum wait for others to cope
      dt = maxd(0.01, dt);
      Wait(dt);
    }
    if (not idle)
      // tell UResSeq (any higher level) that a new line is handled
      sequencerUpdate();
  }
  //
  // terminate log
  if (logSeq != NULL)
  {
    t.Now();
    t.getTimeAsString(st, true);
    //logCalcVars(logSeq);
    fprintf(logSeq,"%s terminated (by USeqLoop::run())\n", st);
  }
  running = false;
}

////////////////////////////////////////////////

// void USeqLoop::setSrObj(SRMain * mainObj)
// {
//   srObj = mainObj;
//   if (srObj->WPFD != NULL)
//     setLaserScanHist(&srObj->WPFD->scanSet);
// }

