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

#include "ucampush.h"

UCamPush::UCamPush(UCamDevBase * dev) 
  : UCamMounted(dev)
{
  //cmdExe = NULL;
}

///////////////////////////////////////////////

UCamPush::~UCamPush()
{
}

///////////////////////////////////////////////

// void UCamPush::gotNewImage(UImage * raw)
// {
//   UServerPushElement * qe;
//   int i;
//   bool success;
//   //
//   if (cmdExe != NULL)
//     for (i = 0; i < push.getPushCmdCnt(); i++)
//     {
//       qe = push.get(i);
//       if (qe->active)
//       {
//         if (cmdExe->isClientAlive(qe->toDo.client))
//         { // execute function if time is right
//           if (((qe->interval > 0.0) and
//               ((qe->nextExeTime - raw->imgTime) < 0.0)) or
//               (qe->interval <= 0.0))
//           {
//             success = cmdExe->executeFunction(qe->functionIndex, &qe->toDo, raw);
//             // update push element with result
//             qe->update(success);
//             //raw->imgTime.print("img4");
//           }
//           //qe->nextExeTime.print("2exe");
//         }
//         else
//           qe->active = false;
//       }
//     }
// }
//
// //////////////////////////////////////////////
//
// int UCamPush::addCamPushCommand(UServerInMsg * msg)
// {
//   int result = 0;
//   char attName[MAX_SML_NAME_LENGTH];
//   const int VAL_BUFF_LNG = 100;
//   char attValue[VAL_BUFF_LNG];
//   char pushCmd[VAL_BUFF_LNG] = "";
//   int n;
//   double interval = -1.0;
//   int countGood = -1;
//   int countTotal = -1;
//   bool isAFlush = false;
// //  int poolImg = -1;
//   UServerPushElement * qe;
//   // reset tag to read all attributes (again)
//   if ((cmdExe != NULL) and (msg != NULL))
//   {
//     msg->tag.reset();
//     // read attributes (except device)
//     while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
//     {
//       if (strcasecmp(attName, "flush") == 0)
//       { // flush
//         isAFlush = true;
//         break;
//       }
//       else if ((strcasecmp(attName, "good") == 0) or
//                 (strcasecmp(attName, "g") == 0))
//         // stop after G good images
//         n = sscanf(attValue, "%d", &countGood);
//       else if ((strcasecmp(attName, "total") == 0) or
//                 (strcasecmp(attName, "n") == 0))
//         // stop aftet N images
//         n = sscanf(attValue, "%d", &countTotal);
//       else if ((strcasecmp(attName, "interval") == 0) or
//                 (strcasecmp(attName, "i") == 0) or
//                 (strcasecmp(attName, "t") == 0))
//         // wait I images before capture
//         n = sscanf(attValue, "%lf", &interval);
//       else if (strcasecmp(attName, "cmd") == 0)
//         // the command to push
//         strncpy(pushCmd, attValue, VAL_BUFF_LNG);
//       // else ignore attribute
//     }
//     //
//     if (isAFlush)
//     { // flush
//       result = push.systemQueueflush(msg->client, attValue);
//     }
//     else if (strlen(pushCmd) > 0)
//     { // space for more?
//       // get requested camera
//       qe = push.getFreeQueueElement();
//       if (qe != NULL)
//       { // add to queue
//         qe->clear();
//         qe->toDo.setMessage(msg->client, pushCmd, strlen(pushCmd));
//         qe->functionIndex = cmdExe->findFunctionOwner(&qe->toDo);
//         if (qe->functionIndex >= 0)
//         { // must be a valid function
//           qe->interval = interval;
//           qe->countGoodTarget = countGood;
//           qe->countTotalTarget = countTotal;
//           qe->toDo.serverPushCommand = true;
//           qe->nextExeTime.Now();
//           qe->nextExeTime += qe->interval;
//           qe->active = true;
//           result = 1;
//         }
//       }
//     }
//   }
//   //
//   return result;
// }
//
// ///////////////////////////////////////////////////////
//
// void UCamPush::flushClientCmds(int clientIdx)
// {
//   push.systemQueueflush(clientIdx, "");
// }

///////////////////////////////////////////////////////

void UCamPush::print(const char * preString)
{
  const int MSL = 2000;
  char s[MSL];
  //
  print(preString, s, MSL);
  printf("%s", s);
}

///////////////////////////////////////////////////////

const char * UCamPush::print(const char * preString, char * buff, int buffCnt)
{
  char * p1 = buff;
  int m;
  UCamDevBase * dev;
  UPosRot pr = getPosRot();
  //
  dev = getDev();
  snprintf(p1, buffCnt, "%s device %d (%s) open=%s %dx%d %dfps %s\n",
         preString,
         dev->getDeviceNumber(), getPosName(),
         bool2str(dev->isCameraOpen()),
         dev->getHeight(), dev->getWidth(),
         dev->getFrameRate(),
         dev->getCameraName());
  m = strlen(p1);
  p1 = &buff[m];
  pr.getPos()->snprint(    "      pos ", p1, buffCnt - m);
  m += strlen(p1);
  p1 = &buff[m];
  pr.getRot()->snprint(    "      rot ", true, p1, buffCnt - m);
  m += strlen(p1);
  p1 = &buff[m];
  UServerPush::print("      push:", p1, buffCnt - m);
  return buff;
}

///////////////////////////////////////////////////////
