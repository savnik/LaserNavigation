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

#include <urob4/uvarpool.h>
#include <urob4/ucmdexe.h>

#include "ureslaserifvar.h"
#include "ufunctionlaserif.h"
#include "ureslaserifroad.h"
#include "ureslaserifobst.h"
//#include "ureslaserifscan.h"
#include "uresnavifman.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionLaserIfData' with your classname, as used in the headerfile */
  return new UFunctionLaserIfData();
}
//
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionLaserIfData::UFunctionLaserIfData()
{ // initialization of variables in class - as needed
  setCommand("laserdata laserobst", "laserif", "control of client side laserdata (jca " __DATE__ " " __TIME__ ")");
  ifRoad = NULL;
  ifRoadLocal = false;
  ifObst = NULL;
  ifObstLocal = false;
  ifSf = NULL;
  ifSfLocal = false;
  ifMan = NULL;
  ifManLocal = false;
  // adds no resources by default
  resList[0] = '\0';
}

///////////////////////////////////////////////////

UFunctionLaserIfData::~UFunctionLaserIfData()
{ // possibly remove allocated variables here - if needed
  if ((ifRoad != NULL) and ifRoadLocal)
    delete ifRoad;
  if ((ifObst != NULL) and ifObstLocal)
    delete ifObst;
  if ((ifSf != NULL) and ifSfLocal)
    delete ifSf;
  if ((ifMan != NULL) and ifManLocal)
    delete ifMan;
}

///////////////////////////////////////////////////

// const char * UFunctionLaserIfData::resourceList()
// {
//   return resList;
// }

///////////////////////////////////////////////////

bool UFunctionLaserIfData::setResource(UResBase * resource, bool remove)
{ // load resource as provided by the server (or other plugins)
  bool result;
  // test if the provided resource is relevant
  // - for obstacle handler
  ifObst = (UResLaserIfObst *) setThisResource(UResLaserIfObst::getResClassID(),
            resource, remove, &result,
            (UResBase*) ifObst, &ifObstLocal);
  if (not result)
  { // for road handler
    ifRoad = (UResLaserIfRoad *) setThisResource(UResLaserIfRoad::getResClassID(),
              resource, remove, &result,
              (UResBase*) ifRoad, &ifRoadLocal);
  }
  if (not result)
  { // for scan feature handler
    ifSf = (UResLaserIfSf*) setThisResource(UResLaserIfSf::getResClassID(),
            resource, remove, &result,
              (UResBase*) ifSf, &ifSfLocal);
  }
  if (not result)
  { // for scan feature handler
    ifMan = (UResNavIfMan*) setThisResource(UResNavIfMan::getResClassID(),
            resource, remove, &result,
            (UResBase*) ifMan, &ifManLocal);
  }
  // other resource types may be needed by base function.
  result |= UFunctionBase::setResource(resource, remove);
  return result;
}


////////////////////////////////////////////////////////

// UResBase * UFunctionLaserIfData::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResLaserIfRoad::getResClassID()) == 0)
//   {
//     if (ifRoad == NULL)
//     { // no pool - so (try to) create one
//       ifRoad = new UResLaserIfRoad();
//       ifRoad->setVerbose(false);
//       ifRoadLocal = (ifRoad != NULL);
//     }
//     result = ifRoad;
//   }
//   else if (strcmp(resID, UResLaserIfObst::getResClassID()) == 0)
//   {
//     if (ifObst == NULL)
//     { // no pool - so (try to) create one
//       ifObst = new UResLaserIfObst();
//       ifObstLocal = (ifObst != NULL);
//     }
//     result = ifObst;
//   }
//   else if (strcmp(resID, UResLaserIfSf::getResClassID()) == 0)
//   {
//     if (ifSf == NULL)
//     { // no pool - so (try to) create one
//       ifSf = new UResLaserIfSf();
//       ifSfLocal = (ifSf != NULL);
//     }
//     result = ifSf;
//   }
//   else if (strcmp(resID, UResNavIfMan::getResClassID()) == 0)
//   {
//     if (ifMan == NULL)
//     { // no pool - so (try to) create one
//       ifMan = new UResNavIfMan();
//       ifManLocal = (ifMan != NULL);
//     }
//     result = ifMan;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionLaserIfData::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   // ask if the function base is OK too
//   result = UFunctionBase::gotAllResources(missingThese, missingTheseCnt);
//   return result;
// }

///////////////////////////////////////////////////

bool UFunctionLaserIfData::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("laserdata"))
    result = handleLaserIf(msg);
  else if (msg->tag.isTagA("laserobst"))
    result = handleLaserObst(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionLaserIfData::handleLaserIf(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char attValue[VAL_BUFF_LNG];
  const int MCL = 500;
  char unused[MCL] = "";
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool anAdd = false;
  char addName[MAX_RESOURCE_ID_LENGTH];
  bool anUnused = false;
  bool aStatus = false;
  bool aClearObst = false;
  bool aClearSf = false;
  bool aClearRoad = false;
  bool result = false;
  const char *p3;
  bool replyOK = false;
  int obstGrpCnt = 0;
  int roadCnt = 0;
  int sfGrpCnt = 0;
  //
  if (ifObst != NULL)
    obstGrpCnt = ifObst->getGroupsCnt();
  if (ifSf != NULL)
    sfGrpCnt = ifSf->getSfPool()->getScansCnt();
  if (ifRoad != NULL)
    roadCnt = ifRoad->getRoadLinesCnt();
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "status") == 0)
      aStatus = true;
    else if (strcasecmp(attName, "add") == 0)
    {
      strncpy(addName, attValue, MAX_RESOURCE_ID_LENGTH);
      anAdd = true;
    }
    // clearObst
    else if (strcasecmp(attName, "clearObst") == 0)
      aClearObst = true;
    else if (strcasecmp(attName, "clearSf") == 0)
      aClearSf = true;
    else if (strcasecmp(attName, "clearRoad") == 0)
      aClearRoad = true;
    else if (strcasecmp(attName, "clear") == 0)
    {
      aClearRoad = true;
      aClearObst = true;
      aClearSf = true;
    }
    else
    { // unused options - it is assumed to be to laser server
      anUnused = true;
      // the rest of the options are assumed to be options to the command
      p3 = msg->tag.getNext();
      snprintf(unused, MCL, "%s=\"%s\" %s\n", attName, attValue, p3);
      break;
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"LASERDATA\">\n");
    sendText(msg, "----------- available LASER Data options\n");
    sendText(msg,        "status                        Status for data handlers\n");
    snprintf(reply, MRL, "add=obst                      Add obstacle data handler handler (added=%s)\n", bool2str(ifObst!=NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=sf                        Add scan-feature data handler client side (added=%s)\n", bool2str(ifSf!=NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=road                      Add road data handler client side (added=%s)\n", bool2str(ifRoad!=NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=man                       Add manoeuvre data handler client side (added=%s)\n", bool2str(ifMan!=NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "clearObst                     Remove all obstacle data (has %d groups)\n", obstGrpCnt);
    sendText(msg, reply);
    snprintf(reply, MRL, "clearSf                       Remove all (scan) grouped features (has %d groups)\n", sfGrpCnt);
    sendText(msg, reply);
    snprintf(reply, MRL, "clearRoad                     Remove all road data (has %d roads lines)\n", roadCnt);
    sendText(msg, reply);
    sendText(msg,        "clear                         Clear all (above) data sets\n");
    sendText(msg,        "help                          This help tekst\n");
    sendText(msg, "-----\n");
    sendText(msg,        "A data handler is no interface, accepts data from an interface - see 'module help'\n");
    sendText(msg,        "see 'module help' on how to load an interface module.\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (anUnused)
  {
    snprintf(reply, MRL, "%s  - unused attributes '%s'", msg->tag.getTagName(), unused);
    sendWarning(msg, reply);
  }
  else
  {
    if (anAdd)
    {
      result = true;
      if (strcasecmp(addName, "road") == 0)
      {
        //result = addResource( UResLaserIfRoad::getResClassID());
        if (ifRoad == NULL)
        {
          ifRoad = new UResLaserIfRoad();
          ifRoad->setVerbose(false);
          ifRoadLocal = (ifRoad != NULL);
          addResource(ifRoad, this);
        }
      }
      else if (strcasecmp(addName, "sf") == 0)
      {
        //result = addResource( UResLaserIfSf::getResClassID());
        if (ifSf == NULL)
        { // no pool - so (try to) create one
          ifSf = new UResLaserIfSf();
          ifSfLocal = (ifSf != NULL);
          addResource(ifSf, this);
        }
      }
      else if (strcasecmp(addName, "obst") == 0)
      {
        //result = addResource( UResLaserIfObst::getResClassID());
        if (ifObst == NULL)
        { // no pool - so (try to) create one
          ifObst = new UResLaserIfObst();
          ifObstLocal = (ifObst != NULL);
          addResource(ifObst, this);
        }
      }
      else if (strcasecmp(addName, "man") == 0)
      {
        //result = addResource( UResNavIfMan::getResClassID());
        if (ifMan == NULL)
        { // no pool - so (try to) create one
          ifMan = new UResNavIfMan();
          ifManLocal = (ifMan != NULL);
          addResource(ifMan, this);
        }
      }
      else
        result = false;
      if (result)
        replyOK = sendInfo(msg, "done");
      else
        replyOK = sendWarning(msg, "Unknown or already loaded data handler");
    }
    if (aStatus)
    {
      sendMsg(msg, "<help subject=\"LASERDATA status\">\n");
      if (ifRoad == NULL)
        sendText(msg, " road  No road line data handler (try add=road)\n");
      else
      {
        ifRoad->snprint(" road  ", reply, MRL);
        sendText(msg, reply);
      }
      if (ifObst == NULL)
        sendText(msg, " obst  No obstacle data handler (try add=obst)\n");
      else
      {
        ifObst->snprint(" obst  ", reply, MRL);
        sendText(msg, reply);
      }
      if (ifSf == NULL)
        sendText(msg, " sf    No scan feature data handler (try add=sf)\n");
      else
      {
        ifSf->snprint(" sf    ", reply, MRL);
        sendText(msg, reply);
      }
      sendMsg(msg, "</help>\n");
      replyOK = sendInfo(msg, "done");
    }
    if (aClearObst)
    {
      if (ifObst != NULL)
        ifObst->clear();
      replyOK = sendInfo(msg, "done");
    }
    if (aClearSf)
    {
      if (ifSf != NULL)
        ifSf->clear();
      if (not replyOK)
        replyOK = sendInfo(msg, "done");
    }
    if (aClearRoad)
    {
      if (ifRoad != NULL)
        ifRoad->clear();
      if (not replyOK)
        replyOK = sendInfo(msg, "done");
    }
    if (not replyOK)
      sendWarning(msg, "Unknown subject");
  }
  return result;
}

///////////////////////////////////////////////////

bool UFunctionLaserIfData::handleLaserObst(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 6000;
  char reply[MRL];
  bool ask4help;
  bool anAdd;
  bool aStatus;
  bool aClearObst;
  bool aList;
  bool aGroup;
  bool aLog, aLogValue;
  bool result = false;
  bool replyOK = false;
  int obstGrpCnt = 0;
//   int roadCnt = 0;
//   int sfGrpCnt = 0;
  int listGroup = 0;
  //
  if (ifObst != NULL)
    obstGrpCnt = ifObst->getGroupsCnt();
//   if (ifSf != NULL)
//     sfGrpCnt = ifSf->getSfPool()->getScansCnt();
//   if (ifRoad != NULL)
//     roadCnt = ifRoad->getRoadLinesCnt();

  ask4help = msg->tag.getAttBool("help", NULL, 0);
  aStatus = msg->tag.getAttBool("status", NULL, 0);
  anAdd = msg->tag.getAttBool("add", NULL, 0);
  aClearObst = msg->tag.getAttBool("clear", NULL, 0);
  aList = msg->tag.getAttInteger("list", NULL, 0);
  aGroup = msg->tag.getAttInteger("group", &listGroup, 0);
  aLog = msg->tag.getAttBool("log", &aLogValue, true);
  if (ask4help)
  {
    sendHelpStart("LASEROBST");
    sendText(            "----------- available laser obstacle data handler options\n");
    sendText(            "status      Status for laser data handlers\n");
    snprintf(reply, MRL, "add         Add obstacle data handler handler (added=%s)\n", bool2str(ifObst!=NULL));
    sendText(reply);
    snprintf(reply, MRL, "clear       Remove all obstacle data (has %d groups)\n", obstGrpCnt);
    sendText( reply);
    snprintf(reply, MRL, "list        list obstaclegroups (has %d)\n", obstGrpCnt);
    sendText( reply);
    sendText(            "group=K     list obstacles for group K (0=newest (default))\n");
    if (ifObst != NULL)
    {
      snprintf(reply, MRL,"log[=false] Open or close logfile, open=%s to %s\n", bool2str(ifObst->olog.isOpen()), ifObst->olog.getLogFileName());
      sendText(reply);
    }
    sendText(            "help        This help tekst\n");
    sendText(            "-----\n");
    sendText(            "A data handler accepts data from an established interface\n");
    sendText(            "see 'module help' on how to load an interface module.\n");
    sendHelpDone();
  }
  else
  {
    if (anAdd)
    {
      result = false;
      //result = addResource( UResLaserIfObst::getResClassID());
      if (ifObst == NULL)
      { // no pool - so (try to) create one
        ifObst = new UResLaserIfObst();
        ifObstLocal = (ifObst != NULL);
        result = addResource(ifObst, this);
      }
      if (result)
        replyOK = sendInfo(msg, "done");
      else
        replyOK = sendWarning(msg, "data handler already loaded");
    }
    if (aStatus)
    {
      sendMsg(msg, "<help subject=\"LASERDATA status\">\n");
      if (ifRoad == NULL)
        sendText(msg, " road  No road line data handler (try add=road)\n");
      else
      {
        ifRoad->snprint(" road  ", reply, MRL);
        sendText(msg, reply);
      }
      if (ifObst == NULL)
        sendText(msg, " obst  No obstacle data handler (try add=obst)\n");
      else
      {
        ifObst->snprint(" obst  ", reply, MRL);
        sendText(msg, reply);
      }
      if (ifSf == NULL)
        sendText(msg, " sf    No scan feature data handler (try add=sf)\n");
      else
      {
        ifSf->snprint(" sf    ", reply, MRL);
        sendText(msg, reply);
      }
      sendMsg(msg, "</help>\n");
      replyOK = sendInfo(msg, "done");
    }
    if (aClearObst)
    {
      if (ifObst != NULL)
        ifObst->clear();
      replyOK = sendInfo(msg, "done");
    }
    if (aList and ifObst != NULL)
    {
      ifObst->listGroups(reply, MRL);
      sendHelpStart("Obstacle group list");
      sendText(reply);
      sendHelpDone();
      replyOK = true;
    }
    if (aGroup and ifObst != NULL)
    {
      ifObst->listGroup(listGroup, reply, MRL);
      sendHelpStart("Obstacle list");
      sendText(reply);
      sendHelpDone();
      replyOK = true;
    }
    if (aLog and ifObst != NULL)
    {
      ifObst->setLog(aLogValue);
      if (not replyOK)
        replyOK = sendInfo("done");
    }
    if (not replyOK)
      sendWarning(msg, "Unknown subject");
  }
  return result;
}

//////////////////////////////////////////////

const char * UFunctionLaserIfData::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s data handler interface for road (%s), obstacle (%s) and scan-features (%s)\n",
           preString, bool2str(ifRoad == NULL), bool2str(ifObst == NULL), bool2str(ifSf == NULL));
  return buff;
}

////////////////////////////////////////////////////

// bool UFunctionLaserIfData::addResource(const char * resName)
// {
//   bool result = false;
//   //char * p1, * p2 = resList;
//   int n, m;
//   bool found = false;
//   //
//   m = strlen(resName);
//   n = strlen(resList);
//   found = inThisStringList(resName, resList);
//   if (not found)
//     // add
//     result = (n < ( MAX_RESOURCE_LIST_SIZE - m - 1));
//   if (result)
//   {
//     strncat(resList, " ", MAX_RESOURCE_LIST_SIZE);
//     strncat(resList, resName, MAX_RESOURCE_LIST_SIZE);
//     cmdHandler->addNewRessources(this);
//   }
//   //
//   return result;
// }

////////////////////////////////////////////////////

