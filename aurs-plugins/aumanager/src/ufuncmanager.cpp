/** \file ufuncmanager.cpp
 *  \ingroup Mission manager
 *  \brief UFunction class for Robot Mission manager
 *
 *  Robot mission manager plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 54 $
 *  $Date: 2009-03-30 17:15:21 +0200 (Mon, 30 Mar 2009) $
 */
/***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck, DTU                *
 *                       anders.beck@get2net.dk                            *
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
/*********************** Version control information ***********************/
 #define VERSION     "1.0"
 #define REVISION    "$Rev: 54 $:"
 #define DATE        "$Date: 2009-03-30 17:15:21 +0200 (Mon, 30 Mar 2009) $:"
/***************************************************************************/

#include <string>
#include <vector>

#include "ufuncmanager.h"
#include "robotmanager.h"
#include "smrclgenerator.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncManager' with your classname, as used in the headerfile */
  return new UFuncManager();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncManager::~UFuncManager()
{ // possibly remove allocated variables here - if needed
  if (manager != NULL)
    delete manager;
}

///////////////////////////////////////////////////

bool UFuncManager::handleCommand(UServerInMsg * msg, void * extra)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 10000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool replySend = false;
  bool gotStartPos;
  char  startpos[MRL];
  char  startorient[MRL];
  bool gotAddWaypoint;
  char waypoint[MRL];
  bool gotPlanning;
  bool gotExecute, gotExeState;
  bool gotShowRoute;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotStart, gotStop;
  //bool result;
  char planToString[256], planFromString[256];


  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = strtol(val, NULL, 0);
    gotStartPos = msg->tag.getAttValue("startpos", val, MVL);
    if (gotStartPos) {
        strncpy(startpos,val,MVL);
        msg->tag.getAttValue("orient", val, MVL);
        strncpy(startorient,val,MVL);
    }
    gotAddWaypoint = msg->tag.getAttValue("addwaypoint", val, MVL);
    if (gotAddWaypoint) {
        strncpy(waypoint,val,MVL);
    }
    gotShowRoute = msg->tag.getAttValue("exportmission", val, MVL);
    gotPlanning = msg->tag.getAttValue("plan", val, MVL);
    if (gotPlanning) {
        msg->tag.getAttValue("from", val, MVL);
        strncpy(planFromString,val,256);
        msg->tag.getAttValue("to", val, MVL);
        strncpy(planToString,val,256);
    }
    gotExecute = msg->tag.getAttValue("execute", val, MVL);
    if (gotExecute) {
        gotExeState = str2bool2(val, true);
    }
    gotStart = msg->tag.getAttValue("start", val, MVL);
    gotStop = msg->tag.getAttValue("stop", val, MVL);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart( msg, "manager");
    sendText( msg, "--- available manager options\n");
    if (manager == NULL)
    {
      sendText( msg, "*** The needed MANAGER resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "log[=false]           Open or close manager logfile 'manager.log'"
             " - is open (%s)\n", bool2str(manager->isLogOpen()));
      sendText(msg, reply);
      snprintf(reply, MRL, "loadmap = 'filename'  Load XML map into graph-database"
             " - is loaded (%s)\n", bool2str(manager->isLogOpen()));
      sendText(msg, reply);
      snprintf(reply, MRL, "verbose[=false]       Output debug messages if true (is %s)\n",
               bool2str(manager->verbose));
      sendText(msg, reply);
      sendText(msg,        "help                  This message\n");
      sendText(msg,        "--------\n");
      sendText(msg,        "see also              var manager    for status and settings\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (manager == NULL)
  {
    sendWarning(msg, "no MANAGER resource to do that - try 'module list' for help");
    replySend = true;
  }
  else
  { // manager resource is available, so make a reply
     if (gotStart) {
       if(manager->rManager.start()) {
           sendInfo(msg, "Started robot mission manager control successfully");
       } else {
           sendWarning(msg,"Could not start robot mission manager");
       }
       replySend = true;
    }
    if (gotStop) {
       if(manager->rManager.stop(true)) {
           sendInfo(msg, "Successfully terminated robot mission manager control");
       } else {
           sendWarning(msg,"Could not stop robot mission manager");
       }
       replySend = true;
    }
    if (gotPlanning) {

       if(manager->planRoute(planFromString,planToString) > 0) {
           sendInfo(msg, "Planning successfully completed");
       } else {
           sendWarning(msg,"Planner could not find route to destination");
       }
       replySend = true;
    }
    if (gotExecute) {
       if (manager->rManager.setExecute(gotExeState)) {
           if (gotExeState) sendInfo(msg,"Starting execution of plan...");
           else sendInfo(msg,"Halted mission execution");
       }
       replySend = true;
    }
    if (gotStartPos) {
        if (strcmp(startorient,"in") == 0) {
            if (manager->rManager.setStartPose(startpos,robotManager::INBOUND) < 0) {
                snprintf(reply,MVL,"Startposition: %s was not found!",startpos);
                sendWarning(msg,reply);
            } else {
                snprintf(reply,MVL,"Start position set to: %s - INBOUND",startpos);
                sendInfo(msg,reply);
            }
        } else if (strcmp(startorient,"out") == 0) {
            if (manager->rManager.setStartPose(startpos,robotManager::OUTBOUND) < 0) {
                snprintf(reply,MVL,"Startposition: %s was not found!",startpos);
                sendWarning(msg,reply);
            } else {
                snprintf(reply,MVL,"Start position set to: %s - OUTBOUND",startpos);
                sendInfo(msg,reply);
            }
            
        } else {
            snprintf(reply,MVL,"Invalid orientation paramenter: %s",startorient);
            sendWarning(msg,reply);
        }
        replySend = true;
        
    }
    if (gotAddWaypoint) {
        if (manager->rManager.addWaypoint(waypoint) > 0) {
            snprintf(reply,MVL,"Waypoint: %s added to mission queue",waypoint);
            sendInfo(msg,reply);
        } else {
            snprintf(reply,MVL,"No route to waypoint: %s was found",waypoint);
            sendWarning(msg,reply);
        }
        replySend = true;

    }
    if (gotShowRoute) {
        string tempStr = manager->rManager.getMissonXML();
        if (tempStr.empty()) {
            snprintf(reply,MVL,"No waypoints added to mission queue yet");
            sendInfo(msg,reply);
        } else {
            sendText(msg,tempStr.c_str());
            snprintf(reply,MVL,"Done printing misson");
            sendInfo(msg,reply);
        }
        replySend = true;

    }
    if (gotVerbose)
    {
      manager->verbose = gotVerboseValue;
      sendInfo(msg, "done");
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo(msg, "no action performed");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////

void UFuncManager::createResources()
{
  manager = new UResManager();
  addResource(manager, this);
}
