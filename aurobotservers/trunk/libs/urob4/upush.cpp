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
#include "upush.h"

//////////////////////////////////////////////

UPush::UPush()
        : UServerPushQueue()
{}

//////////////////////////////////////////////

UPush::~UPush()
{}

//////////////////////////////////////////////

void UPush::pushEvent(void * pushObj)
{
  UServerPushElement * qe;
  int i;
  bool success;
  //
  if (cmdExe != NULL)
    for (i = 0; i < getPushCmdCnt(); i++)
  {
    qe = get(i);
    if (qe->activeCmd)
    {
      if (cmdExe->isClientAlive(qe->toDo.client, 0.0))
      { // execute function
        success = cmdExe->executeFunction(qe->functionIndex, &qe->toDo, pushObj, true);
          // update push element with result
        qe->update(success);
      }
      else
        qe->activeCmd = false;
    }
  }
}

//////////////////////////////////////////////

int UPush::addPushCommand(UServerInMsg * msg)
{
  int result = 0;
  char attName[MAX_SML_NAME_LENGTH];
  const int MVL = MAX_MESSAGE_LENGTH_TO_CAM;
  char attValue[MVL];
  char pushCmd[MVL] = "";
  char pushCall[MVL] = "";
  double interval = 1.0;
  int countGood = -1;
  int countTotal = -1;
  bool isAFlush = false;
  UServerPushElement * qe;
  // reset tag to read all attributes (again)
  if ((cmdExe != NULL) and (msg != NULL))
  {
    msg->tag.reset();
    // read attributes (except device)
    while (msg->tag.getNextAttribute(attName, attValue, MVL))
    {
      if (strcasecmp(attName, "flush") == 0)
      { // flush
        isAFlush = true;
        break;
      }
      else if ((strcasecmp(attName, "good") == 0) or
                (strcasecmp(attName, "g") == 0))
        // position value
        ; //n = sscanf(attValue, "%d", &countGood);
      else if ((strcasecmp(attName, "total") == 0) or
                (strcasecmp(attName, "n") == 0))
        // position value
        ; //n = sscanf(attValue, "%d", &countTotal);
      else if ((strcasecmp(attName, "interval") == 0) or
                (strcasecmp(attName, "t") == 0))
        // position value
        ; // n = sscanf(attValue, "%lf", &interval);
      else if (strcasecmp(attName, "cmd") == 0)
        // the command to push
        strncpy(pushCmd, attValue, MVL);
      else if (strcasecmp(attName, "call") == 0)
        // the command to push
        strncpy(pushCall, attValue, MVL);
      // else ignore attribute
    }
    //
    if (isAFlush)
    { // flush
      result = systemQueueflush(msg->client, attValue);
    }
    else if (strlen(pushCmd) > 0)
    { // space for more?
      // get requested camera
      qe = getFreeQueueElement();
      if (qe != NULL)
      { // add to queue
        qe->clear();
        qe->interval = interval;
        qe->countGoodTarget = countGood;
        qe->countTotalTarget = countTotal;
        qe->toDo.serverPushCommand = true;
        qe->nextExeTime.Now();
        qe->nextExeTime += qe->interval;
        if (strlen(pushCmd) > 0)
        {
          qe->toDo.setMessage(msg->client, pushCmd, strlen(pushCmd), false);
          qe->functionIndex = cmdExe->getFunctionOwner(&qe->toDo);
          if (qe->functionIndex >= 0)
          { // must be a valid function
            qe->toDo.serverPushCommand = true;
            qe->activeCmd = true;
            result = 1;
          }
        }
        if (strlen(pushCall) > 0)
        {
          qe->setCall(msg->client, pushCall);
          qe->activeCall = true;
          result = 1;
        }
      }
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

void UPush::flushClientCmds(int clientIdx)
{
  systemQueueflush(clientIdx, "");
}

///////////////////////////////////////////////////////


