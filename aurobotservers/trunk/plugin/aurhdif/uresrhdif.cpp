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

#include "uresrhdif.h"




void UResRhdIf::UResRhdIfInit()
{
  int i;
  // create status variables
  createBaseVar();
  threadRunning = false;
  verbose = false;
  varsCnt = 0;
  for (i = 0; i < MAX_VARS_CNT; i++)
    vars[i] = NULL;
  // start monitor thread
  start();
}

///////////////////////////////////////////

UResRhdIf::~UResRhdIf()
{
  tryConnect(false, false);
  // stop thread
  stop(true);
}

///////////////////////////////////////////

void UResRhdIf::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    varHost = addVar("host", DEFAULTHOST, "s", "(rw) host where the RHD is running");
    varPort = addVar("port", 24902.0, "d", "(rw) RHD port number");
    varConnected = addVar("connected", 0.0, "d", "(r) is connection to RHD established");
    varKeep = addVar("keep", 1.0, "d", "(rw) Should connection be reestablished after a disconnect");
    varSampleTime = addVar("sampleTime", -1.0, "d", "(r) Actual time between RHD updates");
    varWriteAccess = addVar("writeAccess", 0.0, "d", "(rw) controls - at connect time - if write variables should be in a writeable mode.");
    varWriteGranted = addVar("writeAccessGranted", 0.0, "d", "(r) is true (1) if write access is granted");
  }
}

//////////////////////////////////////////////////

void UResRhdIf::run()
{ //
  threadRunning = true;
  UTime t;
  int loops = 0;
  symTableElement *symTable;
  symTableElement *symTableW;
  double oldTime = 0, newTime = 0;
  int i, n, mr, mw;
  char c;
  int v, d;
  bool updated;
  //
  // wait to allow init script to finish
  Wait(1.2);
  t.now();
  while (not threadStop)
  {
    if (varConnected->getBool())
    {
      t.now();
      // request and wait for new data from RHD
      c = rhdSync();
      if (c <= 0)
      { // disconnect
        rhdDisconnect();
        // set as disconnected regardless of result
        varConnected->setBool(false);
      }
    }
    if (varConnected->getBool())
    {
      symTable = getSymbolTable('r');
      symTableW = getSymbolTable('w');
      newTime = (double)symTable[0].timestamp[0].tv_sec +
                (double)symTable[0].timestamp[0].tv_usec / 1000000.0;
      varSampleTime->setDouble(newTime - oldTime);
      oldTime = newTime;
      // create new symbol table if needed
      if (getSymbolTableSize('r') + getSymbolTableSize('w') > varsCnt)
      {
        mr = mini(getSymbolTableSize('r'), MAX_VARS_CNT);
        for(n = varsCnt; n < mr; n++)
        {
          if (vars[n] == NULL)
            vars[n] = addVar(symTable[n].name, "0 0", "d", "(r) first value is update flag");
          if (symTable[n].updated == 0)
            symTable[n].data[0] = 0;
          varsCnt = maxi(varsCnt, n + 1);
        }
        // add also optional write variables
        mw = mini(getSymbolTableSize('w'), MAX_VARS_CNT - varsCnt);
        for(n = 0; n < mw; n++)
        {
          if (vars[n + mr] == NULL)
          {
            if (varWriteGranted->getBool())
              vars[n + mr] = addVar(symTableW[n].name, "0.0 0.0", "d", "(w) writeable, first value unused");
            else
              vars[n + mr] = addVar(symTableW[n].name, "0.0 0.0", "d", "(r) writeable, first value is update flag");
            if (symTableW[n].updated == 0)
              symTableW[n].data[0] = 0;
            varsCnt = maxi(varsCnt, n + mr + 1);
          }
        }
      }
      mr = mini(getSymbolTableSize('r'), varsCnt);
      for (n = 0; n < mr; n++)
      { // just an update - all variables are created
        // update just values
        for (i = symTable[n].length - 1; i >= 0; i--)
        { // set values from highest index first to reduce
          // number of memory reallocations
          v = symTable[n].data[i];
          vars[n]->setInt(v, i + 1, true);
        }
        // use zero element as update flag
        vars[n]->setBool(symTable[n].updated, 0);
      }
      mw = mini(getSymbolTableSize('w'), varsCnt - mr);
      for (n = 0; n < mw; n++)
      { // just an update - all variables are created
        UVariable * wvar = vars[n + mr];
        updated = wvar->getInt(0);
        for (i = symTableW[n].length - 1; i >= 0; i--)
        { // set values from highest index first to reduce
          // number of memory reallocations
          v = symTableW[n].data[i];
          // get current value of global variable
          if (wvar->getElementCnt() > i + 1)
            d = wvar->getInt(i + 1);
          else
            d = v - 1;
          if (varWriteGranted->getBool() and updated)
          { // global variable marked for update - set write-symbol (overwrite)
            // debug
/*          printf("Updated %s, from %d to %d for index %d\n",
              symTableW[n].name, v, vars[n + mr]->getInt(i + 1), i);*/
            // debug end
            // update symbol table to overwrite the update from the RHD
            symTableW[n].data[i] = d;
          }
          else if (d != v)
            // update global write variable
            wvar->setInt(v, i + 1, true);
        }
        if (varWriteGranted->getBool())
        {
          if (updated)
          {
            symTableW[n].updated = 1;
            wvar->setBool(false, 0);
            // debug
            //printf("Updated %s \n", symTableW[n].name);
            // debug end;
          }
        }
        else
          // the zero element is the update flag - clear it
          wvar->setBool(0, 0);
      }
    }
    else
    {
      if (varKeep->getBool())
      {
        if (t.getTimePassed() > 1.0)
        {
          tryConnect(true, true);
          t.now();
        }
      }
      Wait(0.1);
    }
    loops++;
  }
  threadRunning = false;
}

///////////////////////////////////////////////////

void * startRhdLoopThread(void * obj)
{ // call the hadling function in provided object
  UResRhdIf * ce = (UResRhdIf *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResRhdIf::start()
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
              &startRhdLoopThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UResRhdIf::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////

bool UResRhdIf::tryConnect(bool aConnect, bool keep)
{
  bool result = true;
  char c, cw;
  //
  if (aConnect)
  {
    if (varWriteAccess->getBool())
      cw = 'w';
    else
      cw = 'r';
    if (not isConnected())
    {
      c = rhdConnect(cw, (char *) varHost->getValues(), varPort->getInt());
      result = (c == 'r' or c == 'w');
      varWriteGranted->setBool(c == 'w');
      if (result)
      { // set as connected
        varConnected->setBool(true, 0);
        // tell onConnect that we are now connected
        setUpdated("");
      }
    }
    if (keep != varKeep->getBool())
      // set to automatic reconnect
      varKeep->setBool(keep, 0);
  }
  else
  {
    c = rhdDisconnect();
    result = (c == 1);
    // set to no automatic reconnect
    varKeep->setBool(false);
    // set as disconnected regardless of result
    varConnected->setBool(false);
  }
  return result;
}

/////////////////////////////////////////////
