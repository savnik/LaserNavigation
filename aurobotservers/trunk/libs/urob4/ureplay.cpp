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


#include <stdio.h>
#include <ugen4/ucommon.h>

//#include "ucmdexe.h"
#include "ureplay.h"


UReplay::UReplay()
{
  replay = false;
  replayTimeAdvancePending = false;
  replayFile = NULL;
  replayFileName[0] = '\0';
  replayLine[0] = '\0';
  replayLogLine = -1;
  replayParent = NULL;
  childs = NULL;
  childsCnt = 0;
}

//////////////////////////////////////////////

UReplay::~UReplay()
{
  if (replayFile != NULL)
  {
    fclose(replayFile);
    replayFile = NULL;
  }
}

////////////////////////////////////////////

bool UReplay::replayToTime(UTime untilTime)
{
  bool result = false;
  //
  if (childsCnt > 0)
  {
    for (int i = 0; i < childsCnt; i++)
    {
      if (childs[i] != NULL)
        childs[i]->replayToTime(untilTime);
    }
  }
  else
    while (replay and untilTime > replayTimeNext)
    {
      result = replayStep();
      if (not result)
        break;
    }
  return result;
}

////////////////////////////////////////////

void UReplay::replayAdvanceTime(UTime untilTime)
{
  replayTimeNow = untilTime;
  replayTimeAdvancePending = true;
  if (replayParent != NULL)
    replayParent->replayAdvanceTime(untilTime);
}

//////////////////////////////////////////////////

char * UReplay::getReplayFileName(char * fn, const int fnCnt)
{
  snprintf(fn, fnCnt, "%s/%s", replayPath, replayFileName);
  return fn;
}

///////////////////////////////////////////////////

int UReplay::replayStep(int steps)
{
  bool result = true;
  int i = 0;
  //
  if (replay)
    for (i = 0; i < steps; i++)
    {
      result = replayStep();
      if (not result)
        break;
    }
  if (i > 0)
    replayAdvanceTime(replayTimeNow);
  return replayLogLine;
}

/////////////////////////////////////////////////

bool UReplay::replayStep()
{
  bool result = true;
  UTime t;
  char * p3;
  //
  if (replayFile == NULL and replay)
    // start replay if not started already
    setReplay(true);
  //
  if ((replayFile != NULL) and replay)
  {
    t.setTimeTod(replayLine);
    if (t.valid)
    { // this is a legal line - read the rest
      // format example
      //         time    scan  res start-angle range count range range range ....
      // 1175162437.565549 553237 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.667 1.623 1.682 1.880 ...
      // 1175162438.698901 553322 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.665 1.624 1.682 1.890 ...
      result = decodeReplayLine(replayLine);
      replayTimeNow = t;
    }
    //
    do
    { // read until next valid line (has a timestamp)
      p3 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
      if (p3 != NULL)
        t.setTimeTod(replayLine);
      replayLogLine++;
    } while (not t.valid and not feof(replayFile));
    //
    if (t.valid)
      replayTimeNext = t;
    if (feof(replayFile))
    {
      fclose(replayFile);
      replayFile = NULL;
/*      if (var.replay != NULL)
        var.replay->setValued(replay, 0, false);*/
      fprintf(stderr, "No more valid replay data in %s (after %d lines) - file restarts\n", replayFileName, replayLogLine);
    }
    updateReplayStatus();
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////////

bool UReplay::setReplay(bool value)
{
  bool result = false;
  const int MFL = 500;
  char fn[MFL];
  UTime t;
  char * p1;
  //
  // stop replay
  if (not value and (replayFile != NULL))
  { // stop replay and clear pose history
    fclose(replayFile);
    replayFile = NULL;
    result = true;
  }
  replay = value;
  if (value and (replayFile == NULL))
  { // start replay
    replayFile = fopen(getReplayFileName(fn, MFL), "r");
    result = (replayFile != NULL);
    replayLogLine = 0;
    if (result)
    { // read first line
      while (not t.valid and not feof(replayFile))
      { // read past lines not starting with a timestamp
        p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
        if (p1 == NULL)
          break;
        t.setTimeTod(replayLine);
        replayLogLine++;
      }
      result = t.valid;
      if (result)
        replayTimeNext = t;
      if (feof(replayFile))
      { // no valid data - close file
        fclose(replayFile);
        replayFile = NULL;
        //if (verbose)
        fprintf(stderr, "No valid replay data in '%s'\n", fn);
      }
      //
      updateReplayStatus();
    }
    else
      //if (verbose)
      fprintf(stderr, "Replay file not found '%s'\n", fn);
    result = true;
  }
  return result;
}

//////////////////////////////////////////////////////////

void UReplay::replaySetBaseFileName(const char * name, const char * preName)
{
  if (replayFile != NULL)
  {
    printf("UReplay::replaySetFileName: closing file %s before setting new name to %s\n", replayFileName, name);
    fclose(replayFile);
    replayFile = NULL;
  }
  snprintf(replayFileName, REPLAY_FILE_NAME_LENGTH, "%s%s.log", preName, name);
}

//////////////////////////////////////////////////////////

void UReplay::replaySetFileName(const char * name)
{
  if (replayFile != NULL)
  {
    printf("UReplay::replaySetFileName: closing file %s before setting new name to %s\n", replayFileName, name);
    fclose(replayFile);
    replayFile = NULL;
  }
  strncpy(replayFileName, name, REPLAY_FILE_NAME_LENGTH);
}

////////////////////////////////////////////////////////

void UReplay::setParent(UReplay * parent)
{
  replayParent = parent;
  if (replayParent != NULL)
    replayParent->addChild(this);
}

/////////////////////////////////////////////

void UReplay::addChild(UReplay * child)
{
  int i;
  for (i = 0; i < childsCnt; i++)
  {
    if (childs[i] == child)
      break;
  }
  if (i >= childsCnt)
  {
    childsCnt++;
    childs = (UReplay **) realloc(childs, sizeof(UReplay*) * childsCnt);
    if (childs != NULL)
      childs[i] = child;
    else
      printf("UReplay::addChild failed to allocate space for new child\n");
  }
}

/////////////////////////////////////////////

void UReplay::removeChild(UReplay * child)
{
  int i;
  for (i = 0; i < childsCnt; i++)
  {
    if (childs[i] == child)
      break;
  }
  if (i < childsCnt)
  {
    childs[i] = NULL;
    childsCnt--;
    if (i < childsCnt)
      memmove(&childs[i], &childs[i+1], sizeof(UReplay*) * (childsCnt - i));
  }
}
