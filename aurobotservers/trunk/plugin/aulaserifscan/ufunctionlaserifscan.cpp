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

#include <urob4/uvarcalc.h>

#include "ufunctionlaserifscan.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFunctionLaserIfScan' with your classname, as used in the headerfile */
  return new UFunctionLaserIfScan();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionLaserIfScan::UFunctionLaserIfScan()
{ // initialization of variables in class - as needed
  setCommand("laserscan", "scandataif", "handles control of laser scanner client-side data (by jca " __DATE__ " " __TIME__ ")");
  ifScan = NULL;
  //ifScanLocal = false;
  //strncpy(resList, UResLaserIfScan::getResClassID(), MAX_RESOURCE_LIST_SIZE);
}

///////////////////////////////////////////////////

UFunctionLaserIfScan::~UFunctionLaserIfScan()
{ // possibly remove allocated variables here - if needed
  if (ifScan != NULL)
    delete ifScan;
}

///////////////////////////////////////////////////

// const char * UFunctionLaserIfScan::resourceList()
// {
//   return resList;
// }

///////////////////////////////////////////////////

bool UFunctionLaserIfScan::setResource(UResBase * resource, bool remove)
{ // load resource as provided by the server (or other plugins)
  bool result;
  // test if the provided resource is relevant
  // for laserscan handler
  ifScan = (UResLaserIfScan *) setThisResource(UResLaserIfScan::getResClassID(), resource,
            remove, &result, (UResBase*) ifScan, NULL);
  // other resource types may be needed by base function.
  result |= UFunctionBase::setResource(resource, remove);
  return result;
}


////////////////////////////////////////////////////////

void UFunctionLaserIfScan::createResources()
{
  if (ifScan == NULL)
  { // no pool - so (try to) create one
    ifScan = new UResLaserIfScan();
    // ifScanLocal = (ifScan != NULL);
    addResource(ifScan, this);
  }
}

////////////////////////////////////////////////////////

// UResBase * UFunctionLaserIfScan::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResLaserIfScan::getResClassID()) == 0)
//   {
//     if (ifScan == NULL)
//     { // no pool - so (try to) create one
//       ifScan = new UResLaserIfScan();
//       ifScanLocal = (ifScan != NULL);
//     }
//     result = ifScan;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionLaserIfScan::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (ifScan != NULL);
//   if (not isOK)
//   { // missin this resource
//     if  (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResLaserIfScan::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   // ask if the function base is OK too
//   isOK = UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result and isOK;
// }

///////////////////////////////////////////////////

bool UFunctionLaserIfScan::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("laserscan"))
    result = handleLaserIf(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionLaserIfScan::handleLaserIf(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char attValue[VAL_BUFF_LNG];
  const int MCL = 500;
  char unused[MCL] = "";
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool aStatus = false;
  bool aVerbose = false;
  bool verbose = false;
  bool aClear = false;
  bool aUnused = false;
  bool result = false;
  const char *p3;
  bool replyOK = false;
  UVarPool * vp;
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "status") == 0)
      aStatus = true;
    else if (strcasecmp(attName, "clear") == 0)
    {
      aClear = true;
    }
    else
    { // unused options - it is assumed to be to laser server
      aUnused = true;
      // the rest of the options are assumed to be options to the command
      p3 = msg->tag.getNext();
      snprintf(unused, MCL, "%s %s\n", attName, p3);
      break;
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"LASERSCAN\">\n");
    sendText(msg, "----------- available LASERSCAN options\n");
    sendText(msg,        "status                        Status for interface\n");
    snprintf(reply, MRL, "verbose[=false]               Print more to server console (is %s)\n", bool2str(verbose));
    sendText(msg, reply);
    sendText(msg,        "clear                         Clear all data\n");
    sendText(msg,        "help                          This help tekst\n");
    sendText(msg, "-----\n");
    sendText(msg,        "See also 'LaserOnConnect'\n");
    sendMsg(msg, "</help>\n");
    replyOK = sendInfo(msg, "done");
  }
  else if (aUnused)
  {
    sendMsg(msg, "<help subject=\"LASERSCAN\">\n");
    snprintf(reply, MRL, "Unknown option starting at '%s'\n", unused);
    sendText(msg, reply);
    sendMsg(msg, "</help>\n");
    replyOK = sendInfo(msg, "done");
  }
  else if (ifScan != NULL)
  {
    if (aVerbose)
    {
      ifScan->setVerbose(verbose);
      ifScan->getVarPool()->setVerbose(verbose);
      replyOK = sendInfo(msg, "done");
    }
    if (aStatus)
    {
      sendMsg(msg, "<help subject=\"laser scan data status\">\n");
      vp = ifScan->getVarPool();
      snprintf(reply, MRL, " %s defines (%d variables %d structs %d methods)\n",
               ifScan->name(), vp->getVarsCnt(), vp->getStructCnt(),  vp->getMethodCnt());
      sendText( msg, reply);
        //
      ifScan->snprint(     " - ", reply, MRL);
      sendText(msg, reply);
      // finished
      sendMsg(msg, "</help>\n");
      replyOK = sendInfo(msg, "done");
    }
    if (aClear)
    {
      ifScan->getScanHist()->clear();
      replyOK = sendInfo(msg, "all scan data cleared");
    }
  }
  else
    replyOK = sendWarning(msg, "No laser scan interface resource");
  //
  if (not replyOK)
    sendWarning(msg, "Unknown subject");
  return result;
}

//////////////////////////////////////////////

const char * UFunctionLaserIfScan::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s scan data resource valid (%s)\n",
                  preString, bool2str(ifScan == NULL));
  return buff;
}

////////////////////////////////////////////////////


