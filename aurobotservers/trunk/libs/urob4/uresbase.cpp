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

#include "ucmdexe.h"
#include "uresbase.h"


UResBase::UResBase()
{
  resID[0] = '\0';
  resIsAlso[0] = '\0';
  setResID("resBase", 1514);
  core = NULL;
  var.line = NULL;
  var.time = NULL;
  var.replay = NULL;
}

//////////////////////////////////////////////

UResBase::~UResBase()
{
  if (replayFile != NULL)
  {
    fclose(replayFile);
    replayFile = NULL;
  }
}

//////////////////////////////////////////////

bool UResBase::isA(const char  * idStr)
{
  bool result;
  result = (strcasecmp(resID, idStr) == 0);
  return result;
}

//////////////////////////////////////////////

void UResBase::setResID(const char * id, const int version)
{
  int n;
  //
  if (id != resID)
    strncpy(resID, id, MAX_RESOURCE_ID_LENGTH);
  n = strlen(resIsAlso);
  if ((n + strlen(id) + 1) < MAX_RESOURCE_ISALSO_LENGTH)
  {
    resIsAlso[n++] = ' ';
    strncpy(&resIsAlso[n], id, MAX_RESOURCE_ISALSO_LENGTH - n);
  }
  // set also version
  if (version > 0)
    resVersion = version;
  snprintf(replayFileName, REPLAY_FILE_NAME_LENGTH, "%s.log", id);
}

///////////////////////////////////////////////

bool UResBase::isAlsoA(const char * id)
{
  return inThisStringList(id, resIsAlso);
}

///////////////////////////////////////////////////

void UResBase::print(const char * preString)
{
  const int MSL = 5000;
  char s[MSL];
  printf("%s", print(preString, s, MSL));
}

///////////////////////////////////////////////////

const char * UResBase::print(const char * preString, char * buff, int buffCnt)
{
  if ((buff != NULL) and (buffCnt > 0))
  {
    snprintf(buff, buffCnt, "%s (empty resource)\n", preString);
  }
  return buff;
}

/////////////////////////////////////////////////////////

UResBase * UResBase::getStaticResource(const char * resName, bool mayCreate, bool staticOnly)
{
  UResBase * res = NULL;
  if (core != NULL)
    res = core->getStaticResource(resName, mayCreate, staticOnly);
  return res;
}

////////////////////////////////////////////

// bool UResBase::replayToTime(UTime untilTime)
// {
//   bool result = false;
//   //
//   while (replay and untilTime > replayTimeNext)
//   {
//     result = replayStep();
//     if (not result)
//       break;
//   }
//   return result;
// }

////////////////////////////////////////////

// void UResBase::replayAdvanceTime(UTime untilTime)
// {
//   replayTimeNow = untilTime;
//   replayTimeAdvancePending = true;
// }

///////////////////////////////////////////////

// char * UResBase::getReplayFileName(char * fn, const int fnCnt)
// {
//   snprintf(fn, fnCnt, "%s/%s", replayPath, replayFileName);
//   return fn;
// }

///////////////////////////////////////////////

char * UResBase::getLogFileName(char * fn, const int fnCnt)
{
  snprintf(fn, fnCnt, "%s/%s.log", dataPath, resID);
  return fn;
}

///////////////////////////////////////////////////

// int UResBase::replayStep(int steps)
// {
//   bool result = true;
//   int i;
//   //
//   for (i = 0; i < steps; i++)
//   {
//     result = replayStep();
//     if (not result)
//       break;
//   }
//   if (i > 0)
//     replayAdvanceTime(replayTimeNow);
//   return replayLogLine;
// }

/////////////////////////////////////////////////

// bool UResBase::replayStep()
// {
//   bool result = true;
//   UTime t;
//   char * p3;
//   //
//   if (replayFile == NULL)
//     // start replay if not started already
//     setReplay(true);
//   //
//   if ((replayFile != NULL) and replay)
//   {
//     t.setTimeTod(replayLine);
//     if (t.valid)
//     { // this is a legal line - read the rest
//       // format - 1 (wpf.log)
//       // % laserscan save: time tilt odox odoy odoh planX planY ray1 ... ray181
//       // 1148031884.152597 1148031884.264499 1162.350 1200.926 0.82011 0.00 0.00 8.191 8.191 8.191 ...
//       // format - 2
//       //         time    scan  res start-angle range count range range range ....
//       // 1175162437.565549 553237 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.667 1.623 1.682 1.880 ...
//       // 1175162438.698901 553322 1 -90 181 1.891 1.891 1.881 1.770 1.686 1.665 1.624 1.682 1.890 ...
//       result = decodeReplayLine(replayLine);
//       replayTimeNow = t;
//     }
//     if (var.line != NULL)
//     {
//       var.line->setValued(replayLogLine, 0, false);
//       var.time->setTime(replayTimeNow);
//     }
//     do
//     {
//       p3 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
//       t.setTimeTod(replayLine);
//       replayLogLine++;
//     } while (not t.valid and not feof(replayFile));
//     if (t.valid)
//       replayTimeNext = t;
//     if (feof(replayFile))
//     {
//       fclose(replayFile);
//       replayFile = NULL;
//       if (var.replay != NULL)
//         var.replay->setValued(replay, 0, false);
//       fprintf(stderr, "No more valid replay data in %s (after %d lines) - file restarts\n", replayFileName, replayLogLine);
//     }
//   }
//   else
//     result = false;
//   return result;
// }

/////////////////////////////////////////////////////////

// bool UResBase::setReplay(bool value)
// {
//   bool result = false;
//   const int MFL = 500;
//   char fn[MFL];
//   UTime t;
//   char * p1;
//   //
//   // stop replay
//   if (not value and (replayFile != NULL))
//   { // stop replay and clear pose history
//     fclose(replayFile);
//     replayFile = NULL;
//     result = true;
//   }
//   replay = value;
//   if (value and (replayFile == NULL))
//   { // start replay
//     replayFile = fopen(getReplayFileName(fn, MFL), "r");
//     result = (replayFile != NULL);
//     replayLogLine = 0;
//     if (result)
//     { // read first line
//       while (not t.valid and not feof(replayFile))
//       { // read past lines not starting with a timestamp
//         p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
//         t.setTimeTod(replayLine);
//         replayLogLine++;
//       }
//       result = t.valid;
//       if (result)
//         replayTimeNext = t;
//       if (feof(replayFile))
//       { // no valid data - close file
//         fclose(replayFile);
//         replayFile = NULL;
//         //if (verbose)
//         fprintf(stderr, "No valid replay data in '%s'\n", fn);
//       }
//       if (var.line != NULL)
//       {
//         var.line->setValued(replayLogLine, 0, false);
//         var.time->setTime(replayTimeNext);
//         var.replay->setValued(replay, 0, false);
//       }
//     }
//     else
//       //if (verbose)
//       fprintf(stderr, "Replay file not found '%s'\n", fn);
//     result = true;
//   }
//   return result;
// }

//////////////////////////////////////////////////////////

// void UResBase::replaySetFileName(const char * name)
// {
//   if (replayFile != NULL)
//   {
//     printf("UResBase::replaySetFileName: closing file %s before setting new name to %s\n", replayFileName, name);
//     fclose(replayFile);
//     replayFile = NULL;
//   }
//   strncpy(replayFileName, name, REPLAY_FILE_NAME_LENGTH);
// }

////////////////////////////////////////////////////////

void UResBase::createReplayVar(UVarPool * pool)
{
  var.replay = pool->addVar("replay", 0.0, "d", "(r) is in replay mode");
  var.line = pool->addVar("replayline", 0.0, "d", "(r) current line number into replay file");
  var.time = pool->addVar("replayTime", 0.0, "t", "(r) time of last used replay line");
}

////////////////////////////////////////////////////

void UResBase::updateReplayStatus()
{
  if (var.line != NULL)
  {
    var.line->setValued(replayLogLine, 0, false);
    var.time->setTime(replayTimeNow);
    if (isReplay())
      var.replay->setBool(replayFile != NULL);
  }
}
