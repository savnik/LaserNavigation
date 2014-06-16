/** \file ufuncplanner.cpp
 *  \ingroup Mission planner
 *  \brief UFunction class for Robot Mission planner
 *
 *  Robot mission planner plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1764 $
 *  $Date: 2011-12-08 11:59:40 +0100 (Thu, 08 Dec 2011) $
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
 #define REVISION    "$Rev: 1764 $:"
 #define DATE        "$Date: 2011-12-08 11:59:40 +0100 (Thu, 08 Dec 2011) $:"
/***************************************************************************/

#include <string>
#include <vector>

#include "ufuncplanner.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncPlanner' with your classname, as used in the headerfile */
  return new UFuncPlanner();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncPlanner::UFuncPlanner()
{ 
  // command list and version text
  setCommand(commandList(), "AuPlanner", "Au automatic navigation planner");
  // initialization of variables in class - as needed
  planner = NULL;       // initially the resource is not created
}

///////////////////////////////////////////////////


UFuncPlanner::~UFuncPlanner()
{ // possibly remove allocated variables here - if needed
  if (planner != NULL)
    delete planner;
}

///////////////////////////////////////////////////

const char * UFuncPlanner::name()
{
  //Find revision number from SVN Revision
	char *cPtr, versionString[20] = REVISION, tempString[10];
	cPtr = strrchr(versionString,'$');
	strncpy(tempString,versionString+6,(cPtr-versionString-6));
	tempString[(cPtr-versionString-6)] = 0;
	snprintf(revStr,64,"v%s.%s",VERSION,tempString);
	snprintf(nameStr,256,"planner (%s) (%s %s by Anders B. Beck)",revStr,__DATE__,__TIME__);

	return (const char *)nameStr;
}

///////////////////////////////////////////////////

const char * UFuncPlanner::commandList()
{ // space separated list og command keywords handled by this plugin
  return "planner";
}

////////////////////////////////////////////////////////////

void UFuncPlanner::createResources()
{
  planner = new UResPlanner();
  addResource(planner, this);
  return;
}

////////////////////////////////////////////////////////////

const char * UFuncPlanner::print(const char * preString, char * buff, int buffCnt) {

	return "AuPlanner status is sooo busy!";


}

///////////////////////////////////////////////////

bool UFuncPlanner::handleCommand(UServerInMsg * msg, void * extra)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 10000;
  const int MVL = 1000;
  char reply[MRL];
  bool ask4help;
  char val[MVL];
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool replySend = false;
  bool gotStartPos;
  char  startpos[MVL];
  char  startorient[MVL];
  char  laserif[MVL];
  bool gotAddWaypoint;
  char waypoint[MVL];
  bool gotPlanning;
  bool gotExecute, gotExeState;
  bool gotShowRoute;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotStart, gotStop, gotClear = false;
  //bool result;
  char planToString[MVL], planFromString[MVL];


  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (!ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = strtol(val, NULL, 0);
    gotStartPos = msg->tag.getAttValue("startpos", val, MVL);
    if (gotStartPos) {
        strncpy(startpos,val,MVL);
        msg->tag.getAttValue("orient", val, MVL);
        strncpy(startorient,val,MVL);
		  //Get laser interface name
		  if (msg->tag.getAttValue("laserif", val, MVL)) {
				strncpy(laserif,val,MVL);
		  } else strncpy(laserif,"",MVL);
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
	 gotClear = msg->tag.getAttValue("clearmission", val, MVL);
    gotStart = msg->tag.getAttValue("start", val, MVL);
    gotStop = msg->tag.getAttValue("stop", val, MVL);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
	 snprintf(reply,MRL,"planner %s",revStr);
    sendHelpStart( msg, reply);
    if (planner == NULL)
    {
      sendText( msg, "*** The needed planner resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
	snprintf(reply, MRL, "--- Planner resource ready, version %d\n", planner->getResVersion());
	sendText(msg, reply);
	if (!planner->startConn.empty()) snprintf(reply, MRL, "*** Startconnector: %s - dir: %d\n",planner->startConn.c_str(),planner->currDir);
	else snprintf(reply, MRL, "*** No startposition defined\n");
	sendText(msg, reply);
	sendText( msg, "--- available planner options:\n");
	snprintf(reply, MRL, "log[=false]           Open or close planner logfile 'planner.log'"
             " - is open (%s)\n", bool2str(planner->isLogOpen()));
	sendText(msg, reply);

	snprintf(reply, MRL, "startpos=[startconn] oriet=[in/out] laserif=[optional]\n"
	                     "                      Set robot start connector and orientation, and\n"
		             "                      optional prefix for if to set position in localizer\n");
	sendText(msg, reply);
	snprintf(reply, MRL, "addwaypoint=[name]    Load and plan a waypoint node or connecor for the mission\n");
	sendText(msg, reply);
	snprintf(reply, MRL, "clearmission          Clear and reset planned mission\n");
	sendText(msg, reply);
	snprintf(reply, MRL, "exportmission         Export the robot mission plan in XML\n");
	sendText(msg, reply);
	snprintf(reply, MRL, "verbose[=false]       Output debug messages if true (is %s)\n",bool2str(planner->verbose));
	sendText(msg, reply);
	snprintf(reply, MRL, "execute[=stop,pause]  Start mission execution, or stop/pause it"
			     " - is executing (No status preview yet...)\n");
	sendText(msg, reply);
	sendText(msg,        "help                  This message\n");
	sendText(msg,        "--------\n");
	sendText(msg,        "see also              var planner    for status and settings\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (planner == NULL)
  {
    sendWarning(msg, "no planner resource to do that - try 'module list' for help");
    replySend = true;
  }
  else
  { // planner resource is available, so make a reply
     if (gotStart) {
       if(planner->start()) {
           sendInfo(msg, "Started robot mission manager control successfully");
       } else {
           sendWarning(msg,"Could not start robot mission manager");
       }
       replySend = true;
    }
    if (gotStop) {
		 planner->stop(true);
       if(true) {
           sendInfo(msg, "Successfully terminated robot mission manager control");
       } else {
           sendWarning(msg,"Could not stop robot mission manager");
       }
       replySend = true;
    }
	  if (gotClear) {
		 if(planner->clearMission()) {
           sendInfo(msg, "Mission queue has been cleared, ready for a new one");
       } else {
           sendWarning(msg,"Could not clear mission queue");
       }
       replySend = true;
	  }
    if (gotPlanning) {

       if(planner->planRoute(planFromString,planToString) > 0) {
           sendInfo(msg, "Planning successfully completed");
       } else {
           sendWarning(msg,"Planner could not find route to destination");
       }
       replySend = true;
    }
    if (gotExecute) {
       if (planner->setExecute(gotExeState)) {
           if (gotExeState) sendInfo(msg,"Starting execution of plan...");
           else sendInfo(msg,"Halted mission execution");
       }
       replySend = true;
    }
    if (gotStartPos) {
        if (strcmp(startorient,"in") == 0) {
            if (planner->setStartPose(startpos,UResPlanner::INBOUND,laserif) < 0) {
                snprintf(reply,MVL,"Startposition: %s was not found!",startpos);
                sendWarning(msg,reply);
            } else {
                snprintf(reply,MVL,"Start position set to: %s - INBOUND",startpos);
                sendInfo(msg,reply);
            }
        } else if (strcmp(startorient,"out") == 0) {
            if (planner->setStartPose(startpos,UResPlanner::OUTBOUND,laserif) < 0) {
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
        if (planner->addWaypoint(waypoint) > 0) {
            snprintf(reply,MVL,"Waypoint: %s added to mission queue",waypoint);
            sendInfo(msg,reply);
        } else {
            snprintf(reply,MVL,"No route to waypoint: %s was found",waypoint);
            sendWarning(msg,reply);
        }
        replySend = true;

    }
    if (gotShowRoute) {
        string tempStr = planner->getMissonXML();
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
    if (gotOpenLog) {
      planner->openLog(gotOpenLogValue);
      snprintf(reply, MRL, "done, open=%s file=%s",
               bool2str(planner->isLogOpen()), planner->ULogFile::getLogFileName());
      sendInfo(reply);
      replySend = true;
      
    }
    if (gotVerbose)
    {
      planner->verbose = gotVerboseValue;
      sendInfo(msg, "done");
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo(msg, "no action performed");
  // return true if the function is handled with a positive result
  return true;
}

