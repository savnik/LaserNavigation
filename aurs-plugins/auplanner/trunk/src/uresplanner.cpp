/** \file uresplanner.cpp
 *  \ingroup Mission manager
 *  \brief UResource class for Robot Mission manager
 *
 *  Robot mission planner plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1786 $
 *  $Date: 2012-01-13 15:28:44 +0100 (Fri, 13 Jan 2012) $
 */
/***************************************************************************
 *                  Copyright 2010 Anders Billesø Beck, DTU                *
 *                       anbb@teknologisk.dk                            *
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
 #define REVISION    "$Rev: 1786 $:"
 #define DATE        "$Date: 2012-01-13 15:28:44 +0100 (Fri, 13 Jan 2012) $:"
/***************************************************************************/

#include <termios.h>
#include <stdio.h>
#include <math.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>
#include <vector>

#include "uresplanner.h"

//Constructor
UResPlanner::UResPlanner() { // set name and version

	//Find revision number from SVN Revision
	char *cPtr,versionString[20] = REVISION, tempString[10];
	cPtr = strrchr(versionString,'$');
	strncpy(tempString,versionString+6,(cPtr-versionString-6));
	tempString[(cPtr-versionString-6)] = 0;
	int resIdNr = atoi(tempString);

	 setResID("planner", resIdNr);
    setDescription("Robot Mission Planner and navigation executor");
    UResPlannerInit();
        
}

///////////////////////////////////////////

UResPlanner::~UResPlanner()
{
  // stop job starter thread
  stop(true);
  // the jobs are terminated in the job destructor
}

///////////////////////////////////////////

void UResPlanner::UResPlannerInit()
{
  // to save the ID and version number
  setLogName("planner");
  openLog();

  // create status variables
  createBaseVar();
  // the this class execute the push jobs
  threadRunning = false;
  verbose = false;
  executing = false;

    doubleRange = false;
	detectRange = false;
	stopRange = false;
  
  // start job starter thread
  start();
}

////////////////////////////////////////////////////////////////////////////////

const char* UResPlanner::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int n = 0;
  char * p1 = buff;
  const int MSL = 50;
  char s[MSL];
  UTime lastTime; // should be replaced by something more appropriate
  //
  snprintf(p1, buffCnt - n, "%s fast scheduled commands - @todo more verbose\n", preString);
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "    Last command run at at %s\n", s);

  return buff;
}



///////////////////////////////////////////

void UResPlanner::createBaseVar()
{

    /*varIdleCnt  = addVar("idleCnt",  0.0, "d", "(r) idle loops with no commands");
    varJobCnt   = addVar("jobCnt",   0.0, "d", "(r) number of commands run");
    varErrCnt   = addVar("errCnt",   0.0, "d", "(r) number of commands returned non success");
    varLastTime = addVar("exeTime",  0.0, "d", "(r) execution time of last command");
    varCronTime = addVar("cronTime", 1.0, "d", "(r/w) idle count interval (maximum) when testing scheduled tasks");*/
    addMethod("startnode" ,"s", "Get feature value from [s] extracted from the current start-node");
    addMethod("endnode"   ,"s", "Get feature value from [s] extracted from the current end-node");
    addMethod("startconn" ,"s", "Get feature value from [s] extracted from the current start-connector");
    addMethod("endconn"   ,"s", "Get feature value from [s] extracted from the current end-connector");
    addMethod("edge"      ,"s", "Get feature value from [s] extracted from the current edge");
    addMethod("nextedge"  ,"s", "Get feature value from [s] extracted from the next edge");
    addMethod("nextnode"  ,"s", "Get feature value from [s] extracted from the next end-node");
    addMethod("nextconn"  ,"s", "Get feature value from [s] extracted from the next end-connector");

    addMethod("test"        ,"", "Test various functions");
    addMethod("start"       ,"", "Start the manager robot control");
    addMethod("stop"        ,"", "Stop the manager robot control");

    addMethod("clearmission","", "Clear the planned mission and reset mission manager");
    addMethod("addwaypoint" ,"s", "Add a waypoint to the robot mission");
    addMethod("startpos"    ,"sss","Set the robot start possition and pose, and interface to set initpose to localizer");

	 //Use pointer to base resource to create variables
    period      = addVar("period",         0.1, "d", "Period in sec. to call the movement planner");
    wpreached   = addVar("wpReached",      0.6, "d", "Distance from waypoint before it is detected as reached");
    endreached  = addVar("endReached",     0.1, "d", "Distance from mission endpoint before it is detected as reached");
    nextWp      = addVar("nextWaypoint",   0.0, "d", "Next target waypoint in the mission");
    totalWp     = addVar("totalWaypoints", 0.0, "d", "Total number of waypoints");
    totalRe     = addVar("totalRoute",     0.0, "d", "Total number of route elements");
    currRe      = addVar("currentRoute",   0.0, "d", "Current route element");
    currWpRe    = addVar("currentWpRoute", 0.0, "d", "Current route element in waypoint route");
    exe         = addVar("execute",        0.0, "d", "Start the execution of mission");
    reverse     = addVar("reverse",        0.0, "d", "Should the robot change direction to follow the current edge");
    nextReverse = addVar("nextReverse",    0.0, "d", "Should the robot change direction to follow the next edge");
    dMethod     = addVar("driveMethod",    2.0, "d", "Method to navigate (1=drivePos, 2=driveonw directly to MRC)");
    defaultSpeed = addVar("speed",    	   0.2, "d", "Default speed for the robot"); 
    defaultAccel = addVar("accel",    	   0.075, "d", "Default accelleration for the robot");
	 //currentConn = addVar("currentConn",    "void", "s", "Current connector (or last visited)");
	 //currentDir  = addVar("currentDir",     "nowhere","s","Current robot direction in the connector");

}

/////////////////////////////////////////////////////

int UResPlanner::findFunctionOwner(const char * tagName)
{ // one function owner only (me)
  return 1;
}

////////////////////////////////////////////////////////////////////////////////

bool UResPlanner::setResource(UResBase * resource, bool remove)
{
  bool result = false;

  if (resource->isA(UResVarPool::getResID()))
  {
    result = true;
    if (remove) {
      varGlobal = NULL;
		varPool = NULL;
	 } else if (varGlobal != resource) {
      varGlobal = (UResVarPool *)resource;
	 } else
      // not used
      result = false;
  }
  else if (resource->isA(UResMapbase::getResClassID())) {
      result = true;
      if (remove) mapbase = NULL;
      else if (mapbase != resource) {
          mapbase = (UResMapbase*)resource;
			 mapbase->graphmapper.loadClipper(this);
      } else result = false;
  }
  else if (resource->isA(UResPoseHist::getMapPoseID()))
  { // pointer to server the resource that this plugin can provide too
    // but as there might be more plugins that can provide the same resource
    // use the provided
    if (remove)
      // the resource is unloaded, so reference must be removed
      mappose = NULL;
    else if (mappose != (UResPoseHist *)resource)
      // resource is new or is moved, save the new reference
      mappose = (UResPoseHist *)resource;
    else
      // reference is not used
      result = false;
  }
  else if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // pointer to server the resource that this plugin can provide too
    // but as there might be more plugins that can provide the same resource
    // use the provided
    if (remove)
      // the resource is unloaded, so reference must be removed
      odopose = NULL;
    else if (odopose != (UResPoseHist *)resource)
      // resource is new or is moved, save the new reference
      odopose = (UResPoseHist *)resource;
    else
      // reference is not used
      result = false;
  }
  else if (resource->isA(UResSmrIf::getResClassID())) {
	  if (remove)
      // the resource is unloaded, so reference must be removed
      smrIf = NULL;
	  else if (smrIf != (UResSmrIf *)resource)
      // resource is new or is moved, save the new reference
      smrIf = (UResSmrIf *)resource;
    else
      // reference is not used
      result = false;

  }
  else
    result = UResVarPool::setResource(resource, remove);
  return result;
}

//////////////////////////////////////////////

bool UResPlanner::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  double retValue = 0;

  if ((strcasecmp(name, "test") == 0))
  {
				  retValue = 1;
				  test();
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  //Start the manager control thread
  else if ((strcasecmp(name, "start") == 0))
  {
      //Call the start function
      retValue = start();
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "stop") == 0))
  {

      retValue = 1;
		stop(true);
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "addwaypoint") == 0))
  {
      retValue = addWaypoint(strings[0]);
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "startpos") == 0))
  {

	  dirType stDir;
	  if (strcmp(strings[1],"in") == 0) stDir = INBOUND;
	  else stDir = OUTBOUND;

	  retValue = setStartPose(strings[0],stDir,strings[2]);

    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "clearmission") == 0))
  {
	  retValue = clearMission();
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else
    // call name is unknown
    result = false;
  return result;
}


/**
 * Finish initialization by creating and adding resources
 **/
void UResPlanner::createResources(void) {
    if (varGlobal != NULL) {
        varPool = (UVarPool *)(((UVarCalc*) (varGlobal->getVarPool()))->getRootVarPool()); //Magic syntax!
		  printf("Created varPool! %p\n",(void *)varPool);
    }

    //Load the mapbase into robot state
    if (mapbase != NULL) {
        //rManager.loadMapbase(mapbase);
    }

}

void UResPlanner::test(void) {
	
	updateDestination();
	destinationReached();
	testVar = true;
	// double posDst[3] = {dstOdo.x,dstOdo.y,dstOdo.h};
	// double retVal;
	// int count = 0;         //"send","s",&txStr,NULL,&retVal,NULL,&count
	// varGlobal->callGlobal("drivepos.odo","ddd",(char **)NULL,(const double *)&posDst,&retVal,(UDataBase **)NULL,(int *)&count);


}


 /*  Activate mission execution */
 bool UResPlanner::setExecute(bool go) {

     if (go) {
		  if (!rPlan.empty()) {
			 exe->setValued(1,0);
			 currRe->setValued(0.0,0);
		  }
		  else printf("No route has been planned yet... Execution halted\n");
	  } else {
		  exe->setValued(-1,0);
	  }
     return true;

 }

 //////////////////////////////////////////////////////////////////////////////

 /******************************************************************************
  *                       MAIN EXECUTION THREAD
  ******************************************************************************/
void UResPlanner::run()
{ //
  threadRunning = true;
  bool executing = false;
  UTime t;
  int	lastRe = 0;
  char cmdStr[1024];
  string cmd;
  cmd.resize(1024); //Reserve a bit of memory for the string
  //UVarPool *vp;

  // wait to allow init script to finish
  Wait(5.0);

  //Main control loop
  while (not threadStop)
  {
    t.Now();

	 //Are we executing the plan
	 if ((exe->getValued() == 1.0) || (testVar == true)) {
		//Call housekeeper to maintain direction infomation variables
		if (executing) updateRoute(); //Start by detecting if a waypoint has been reached
		housekeeper();
		updateDestination(); //Update pose variables and destination pose

		if (!executing) lastRe = -1; //Reset last route element when execution is started

		//Test if the robot must be turned before starting route
		if (reverse->getValued() && !executing) {
		  //Turn the robot, if it must
			printf("Turning robot before starting mission\n");
			smrIf->doSmrDrive("flushcmds\n",0);
		   smrIf->doSmrDrive("turn 180 @v0.1 @a0.1\n",0);
			smrIf->sendUserEvent("ev",99);
			smrIf->doSmrDrive("idle\n",0);
			for (int i = 0; i < 1000; i++) {
				if (smrIf->testEvent(99,NULL,true)) {
					printf("Recieved event, done turning\n");
					break;
				}
				Wait(0.05);
			}
		}
		

	  //Activate drivepos
	  if (dMethod->getValued() <= 1) {
			double posDst[3] = {dstOdo.x,dstOdo.y,dstOdo.h};
			double retVal;
			int count = 0;         //"send","s",&txStr,NULL,&retVal,NULL,&count
			varGlobal->callGlobal("drivepos.odo","ddd",(char **)NULL,(const double *)&posDst,&retVal,(UDataBase **)NULL,(int *)&count);
	  } else if ((dMethod->getValued() == 2.0) && (lastRe != roundi(currRe->getValued()))) {
			//Generate command string
	          double stopdist = 0.05; //Distance to break robot
		  cmd.clear();
		  snprintf(cmdStr,1024,"flushcmds\n ");
		  cmd += cmdStr;
		  snprintf(cmdStr,1024,"driveonw %f %f %f \"rad\" @v%4.3f @a%5.4f :($targetdist < %f)\n ",dstMap.x, dstMap.y, dstMap.h,currentSpeed,currentAccel,2*wpreached->getValued());
		  cmd += cmdStr;
		  smrIf->sendString(cmd.c_str());
		  smrIf->sendUserEvent("ev",90);
		  snprintf(cmdStr,1024,"driveonw %f %f %f \"rad\" @v%4.3f @a%5.4f :($targetdist < %f)\n ",dstMap.x, dstMap.y, dstMap.h,currentSpeed,currentAccel,wpreached->getValued());
		  smrIf->sendString(cmdStr);
		  smrIf->sendUserEvent("ev",91);
		  snprintf(cmdStr,1024,"driveonw %f %f %f \"rad\" @v%4.3f @a%5.4f :($targetdist < %f)\n ",dstMap.x, dstMap.y, dstMap.h,currentSpeed,currentAccel,stopdist);
		  smrIf->sendString(cmdStr);
		  smrIf->sendUserEvent("ev",92);
		  snprintf(cmdStr,1024,"fwd %f @v%4.3f @a0.4\n idle\n ",stopdist,currentSpeed); //Gently stop the robot
		  smrIf->sendString(cmdStr);
		  smrIf->sendUserEvent("ev",98);
		  lastRe = roundi(currRe->getValued());
		  if(verbose) printf ("Plan towards: %s transmitted\n",rPlan[(int)currRe->getValued()].endName.c_str());
		  
		  //Clear waiting events
		smrIf->testEvent(98,NULL,true);	  	//Destination reached
		smrIf->testEvent(90,NULL,true);
		smrIf->testEvent(91,NULL,true); //Within detection range
		smrIf->testEvent(92,NULL,true); //Last stopping range
		doubleRange = false;
		detectRange = false;
		stopRange = false;

	  }

		executing = true;
		testVar = false;
	  
	 } else if (exe->getValued() > 1.0) { //Second execution level
		 executing = false;

	 }  else {
		 executing = false;
		 t.Now();

	 }

	 if (verbose && executing) printf("---======================= Period done =======================--- Re %2.1f\n",currRe->getValued());

	 //Wait a bit
     Wait(period->getValued());
  }
  threadRunning = false;

}



void * startManagerThread(void * obj)
{ // call the hadling function in provided object
  UResPlanner * ce = (UResPlanner *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResPlanner::start()
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
              &startManagerThread, (void *)this) == 0);
  }
  return result;
}

///////////////////////////////////////////////////

void UResPlanner::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    //pthread_join(threadHandle, NULL);
//TODO Re-enable thread-join..
  }
}

/*******************************************************************************
 *                     PLANNING FUNCTIONS
 * Functions in this section are related to motion plannig using the graph based
 * map
 *******************************************************************************/

/** Planning clipper function
 *
 * This function is used to interface the planning clipper in the
 * shortest path calculations.
 *
 * If the function returns true, the edge currEdge is not accepted for
 * the route planning.
 */
bool UResPlanner::shortestPathClipper (edge* currEdge, edge *prevEdge) {

    //Cancel blocked edges
    if (currEdge->blocked) return true;

    return false;

}

//////////////////////////////////////////////////

/**
  * Plan a route between two map location
  **/
int UResPlanner::planRoute(const char* fromPos, const char* toPos) {

      string from   = fromPos;
      string to     = toPos;
      int   retVal = 0;

      //Execute planner
      if (mapbase->graphmapper.calculateRoute(from,to,&rPlan) < 0) {
          retVal =  -1;
      } else {
          printf("Planning successfull - Route:\n");
          for (size_t i = 0; i < rPlan.size(); i++) {
              printf("   %s -> %s   (length: %d)\n",rPlan[i].startName.c_str(),rPlan[i].endName.c_str(),rPlan[i].edgeLength);
          }
          retVal = 1;
      }

      return retVal;
}

/** Clear any planned mission elements */
int UResPlanner::clearMission(void) {

	//Make sure that house-keeper has everything updated
	if (totalRe->getValued() > 0.0) housekeeper();

	//Clear any planned elements
	wpPlan.clear();
	rPlan.clear();

	//Reset state variables
	currRe->setValued(0.0,0);
	nextWp->setValued(0.0,0);
   totalWp->setValued(0.0,0);
   totalRe->setValued(0.0,0);
   currRe->setValued(0.0,0);
   currWpRe->setValued(0.0,0);
   exe->setValued(0.0,0);
   reverse->setValued(0.0,0);
   nextReverse->setValued(0.0,0);
   doubleRange = false;
	detectRange = false;
	stopRange = false;

	return 1;
}

/** Add waypoint to the mission queue */
int UResPlanner::addWaypoint(char *wpName) {

    waypoint    *tempWp = new waypoint;
    string      startPoint;
    string      endPoint = wpName;

    //First waypoint takes starting point in the starting point, that has been defined
    //The following waypoints starts in the endpoint of the previous
    if (wpPlan.empty()) {
        startPoint = startConn;
    } else {
        startPoint = wpPlan.back().plan.back().endName;
    }

    //Execute route-planner on startPoint -> wp
    tempWp->routeLength = mapbase->graphmapper.calculateRoute(startPoint,endPoint,&(tempWp->plan));

    //Check if a route was found
    if (tempWp->routeLength <= 0) {
        printf("Failed to add waypoint %s to waypoint queue\n",wpName);
        return -1;
    }

    tempWp->startConn = startPoint;
    tempWp->endConn   = tempWp->plan.back().endName;

    //Add the waypoint to the waypoint vector
    wpPlan.push_back(*tempWp);

    //Add the calculated route to the full route plan
    for (size_t i = 0; i < tempWp->plan.size();  i++) {
        rPlan.push_back(tempWp->plan[i]);
    }

    delete tempWp;

    //Update varpool variables
    totalWp->setValued(wpPlan.size(),0);
    totalRe->setValued(rPlan.size(),0);


    return 1;

}


/** Set robot startpose before planning
 */
int UResPlanner::setStartPose(char *startC, dirType dir, char *ifname) {

    //Find start connector
	 double poseAngleRad;
    connector *sConn = mapbase->graphmapper.getConnector(startC);

    if (sConn == NULL) return -1;

    //Assign startConnector and currentConnector
    startConn   = startC;
    //currentConn->setValues(startC,0,true);

	 printf("Start pose set to (%f,%f,%f)\n",sConn->poseMap.x,sConn->poseMap.y,sConn->poseMap.thr);

    //Dir is 0 if robot points inward into the node
    if (dir == INBOUND) {
		  //currentDir->setValues("in",0, true);
        currDir = INBOUND;
        poseAngleRad = sConn->features["thr"] + M_PI;
        if (poseAngleRad > M_PI) poseAngleRad -= 2*M_PI;
    } else {
        currDir = OUTBOUND;
		  //currentDir->setValues("out",0,true);
        poseAngleRad = sConn->features["thr"];
    }
	 
	 printf("Start pose set to (%f,%f,%f)\n",sConn->poseMap.x,sConn->poseMap.y,poseAngleRad);

	 //Get core pointer and send startposition to localizer
	 UCmdExe *cp;
	 char cmdstr[1024];
	 cp = getCorePointer();
	 snprintf(cmdstr,256,"%s setinitpose x=%f y=%f th=%f\n",ifname,sConn->poseMap.x,sConn->poseMap.y,poseAngleRad);
	 printf("Send to core: %s",cmdstr);
	 if (cp != NULL) cp->postCommand(-1,cmdstr);
	 snprintf(cmdstr,256,"%s setinitcov Cx=0.05 Cy=0.05 Cth=0.05\n",ifname);
	 if (cp != NULL) cp->postCommand(-1,cmdstr); //Set a relatively high covariance
    return 1;
}

/** Generate a misson XML string */
string UResPlanner::getMissonXML(void) {

    string  xmlString;
    char    tempStr[1024];

    if (wpPlan.empty()) {
        xmlString.clear();
        return xmlString;
    } else {
        sprintf(tempStr,"<misson from=\"%s\" to=\"%s\">\n",
                wpPlan.front().startConn.c_str(),wpPlan.back().endConn.c_str());
        xmlString = tempStr;
        for(size_t i = 0; i < wpPlan.size(); i++) {
            sprintf(tempStr,"  <waypoint from=\"%s\" to=\"%s\" length=\"%f\">\n",
                wpPlan[i].startConn.c_str(),wpPlan[i].endConn.c_str(),(double)wpPlan[i].routeLength/1000.0);
            xmlString += tempStr;
            for (size_t j = 0; j < wpPlan[i].plan.size(); j++) {
                sprintf(tempStr,"    <edge from=\"%s\" to=\"%s\" length=\"%f\"/>\n",
                    wpPlan[i].plan[j].startName.c_str(),wpPlan[i].plan[j].endName.c_str(),
                    (double)(wpPlan[i].plan[j].edgeLength)/1000.0);
                xmlString += tempStr;
            }
            xmlString += "  </waypoint>\n";
        }
        xmlString += "</misson>\n";
    }
    return xmlString;

}

/* Transmit a string to the SMR */
bool UResPlanner::sendSmr(string cmdStr) {

      const size_t      MAXTX       = 250; //Maximum length of accepted SMR-CL string
      size_t            found = 0;
      string            sendString;
      char              *txStr = new char[MAXTX+1];
      double            retVal;
      int               count = 0;

      //Check if the SMR variable pool is properly initialized
      if ((smrPool == NULL) | (smrConnected == NULL)) return false;

      //Check if the SMR is connected
      if (smrConnected->getValued() <= 0)   return false;

      //Break the string into one-liners and send them to mrc
      while (found != cmdStr.npos) {
          found = cmdStr.find_first_of('\n');
          sendString = cmdStr.substr(0,found+1);
          cmdStr = cmdStr.substr(found+1);
          strncpy(txStr,sendString.c_str(),MAXTX+1);
          if (sendString.length() > 0) {
            smrPool->callLocal("send","s",&txStr,NULL,&retVal,NULL,&count);
          }
      }
      return true;

}

/* Check for a usereven in the SMR Pool */
bool UResPlanner::checkSmrEvent(int event) {

      double            retVal;
		double				dEvent = (double)event;
		int					count;

      //Check if the SMR variable pool is properly initialized
      if ((smrPool == NULL) | (smrConnected == NULL)) return false;

      //Check if the SMR is connected
      if (smrConnected->getValued() <= 0)   return false;

      //Break the string into one-liners and send them to mrc
       smrPool->callLocal("gotEvent","d",NULL,&dEvent,&retVal,NULL,&count);

		if (retVal != 0) return true;
		else return false;
}



//////////////////////////////////////////////////////////////////////////////

bool UResPlanner::housekeeper(void) {

    //Update orientation
    if(currRe->getValued() > 0) {
        currDir = getDir(&(rPlan[(int)currRe->getValued()-1].routeEdge));
		  //if (currDir == INBOUND)	currentDir->setValues("in",0,true);
		  //else							currentDir->setValued("out",0,true);
    }
    //Get resulting robot move-type from current edge
    currMove = getMove(&(rPlan[(int)currRe->getValued()].routeEdge));

    //Update next orientation and move (if possible)
    if(currRe->getValued() <= totalRe->getValued()) {
        nextDir  = getDir(&(rPlan[(int)currRe->getValued()].routeEdge));
        nextMove = getMove(&(rPlan[(int)currRe->getValued() + 1].routeEdge));
    }

    //Some paths are in the 180 degrees opposite direction of the robot.
    //Set the reverse flag if this is true
    if (((currDir == INBOUND) && ((currMove == EXTNODE)||(currMove == PARENTNODE))) ||
        ((currDir == OUTBOUND) && ((currMove == INTNODE)||(currMove == OWNNODE)))) {
        //Set the reverse-uvariable
        reverse->setValued(1.0,0);
    } else {
        reverse->setValued(0.0,0);
    }

	 //Update the current connector
	 startConn = rPlan[(int)currRe->getValued()].startName;
	 //currentConn->setValues(rPlan[(int)currRe->getValued()].startName.c_str(),0,true);


    //Update the next-reverse as well
    //Set the reverse flag if this is true
    if (((nextDir == INBOUND) && ((nextMove == EXTNODE)||(nextMove == PARENTNODE))) ||
        ((nextDir == OUTBOUND) && ((nextMove == INTNODE)||(nextMove == OWNNODE)))) {
        //Set the reverse-uvariable
        nextReverse->setValued(1.0,0);
    } else {
        //Set the reverse-uvariable
        nextReverse->setValued(0.0,0);
    }
    
    //Detect the speed for the next edge
    currentSpeed = (double)defaultSpeed->getValued();
    currentAccel = (double)defaultAccel->getValued();

	 //Print status
	 if (verbose) {
		 printf("*** House keeper ***\nDirection:");
		if (currDir == INBOUND) printf("Current: inbound,");
		else printf("Current: outbound,");
		if (nextDir == INBOUND) printf(" Next: inbound\n");
		else printf(" Next: outbound\n");
		printf("Moves: current %d, next: %d\n",currMove,nextMove);
		printf("Reverse: current %2.1f, next: %2.1f\n",reverse->getValued(),nextReverse->getValued());
	 }


    loopCounter++;

    return true;
}


/** Function to calculate if robot is in or outbound when finishing an edge
 */
UResPlanner::dirType UResPlanner::getDir(edge *dEdge) {

    //If the robot drives between two connectors in the same node, it goes from
    //beeing inbound to beeing outbound
    if (dEdge->startNodeIndex == dEdge->endNodeIndex) {
        return OUTBOUND; //Return outbound
    }
    //If the robot drives from a parent node to an internal node, it goes from
    //beeing inbound to beeing inbound
    else if (!strncmp(dEdge->startNodeName,dEdge->endNodeName,strlen(dEdge->startNodeName))) {
        return INBOUND;
    }
    //If the robot drives from an internal node to a parent node, it goes from beeing
    //outbound to beeing outbound
    else if (!strncmp(dEdge->endNodeName,dEdge->startNodeName,strlen(dEdge->endNodeName))) {
        return OUTBOUND;
    }
    //Finally if the robot goes from one node to another in the same context, it goes from
    //beeing outbound to beeing inbound
    else {
        return INBOUND;
    }

}

/** Function to calculate what type of move the robot is doing
 */
UResPlanner::moveType UResPlanner::getMove(edge *dEdge) {

    //If the robot drives between two connectors in the same node, it goes from
    //beeing inbound to beeing outbound
    if (dEdge->startNodeIndex == dEdge->endNodeIndex) {
        return OWNNODE; //Return outbound
    }
    //If the robot drives from a parent node to an internal node, it goes from
    //beeing inbound to beeing inbound
    else if (!strncmp(dEdge->startNodeName,dEdge->endNodeName,strlen(dEdge->startNodeName))) {
        return INTNODE;
    }
    //If the robot drives from an internal node to a parent node, it goes from beeing
    //outbound to beeing outbound
    else if (!strncmp(dEdge->endNodeName,dEdge->startNodeName,strlen(dEdge->endNodeName))) {
        return PARENTNODE;
    }
    //Finally if the robot goes from one node to another in the same context, it goes from
    //beeing outbound to beeing inbound
    else {
        return EXTNODE;
    }

}


/*******************************************************************************
 *                     MOVEMENT FUNCTIONS
 * Function in this section are related to movement and manipulation with
 * coordinate systems.
 *******************************************************************************/

int UResPlanner::updateDestination(void) {



	 lastMap = mappose->getNewest();
	 lastOdo = odopose->getNewest();
	 int cRoute = (int)currRe->getValued();

	 //Select the reference pose
	 refPose = lastMap;


	 if (verbose) {
		 printf("Mappose %f,%f,%f\n",lastMap.x,lastMap.y,lastMap.h);
		 printf("Odopose %f,%f,%f\n",lastOdo.x,lastOdo.y,lastOdo.h);
		 printf("Reference pose %f,%f,%f\n",refPose.x,refPose.y,refPose.h);
	 }


	 if (rPlan.size()) {
		dstMap.x = rPlan[cRoute].endConn.poseMap.x;
		dstMap.y = rPlan[cRoute].endConn.poseMap.y;
	   dstMap.h = rPlan[cRoute].endConn.poseMap.thr;

		//Evaluate if the robot is inbound to the next connector and rotate 180deg if it is the case
		if (nextDir == INBOUND) {
			dstMap.h += M_PI; //Rotate
			if (dstMap.h > M_PI) dstMap.h -= 2 * M_PI; //Keep within +/- M_PI
			if (verbose) printf("Robot is inbound next time, rotating endpose\n");
		}

		if (verbose) printf("Destination pose: %f,%f,%f\n",dstMap.x,dstMap.y,dstMap.h);

	 } else if (verbose) printf("ROute plan is empty\n");

	 dstMapRelative = lastMap.getMapToPosePose(&dstMap);
	 if (verbose) printf("Relative to mappose: %f,%f,%f\n",dstMapRelative.x,dstMapRelative.y,dstMapRelative.h);
	 dstRefRelative = refPose.getMapToPosePose(&dstMap);
	 if (verbose) printf("Relative to refernce: %f,%f,%f\n",dstRefRelative.x,dstRefRelative.y,dstRefRelative.h);
	 dstOdo = lastOdo.getPoseToMapPose(&dstRefRelative);
	 if (verbose) printf("Odometry destination to mappose: %f,%f,%f\n",dstOdo.x,dstOdo.y,dstOdo.h);
	 dstOdoRelative = lastOdo.getMapToPosePose(&dstOdo);
	 if (verbose) printf("destination relative to odopose: %f,%f,%f\n",dstOdoRelative.x,dstOdoRelative.y,dstOdoRelative.h);

	 return 1;
}

////////////////////////////////////////////////////////////////////////////////

double UResPlanner::destinationReached(void) {

	//Calculate distance to endpoin
	double a,aInv, bInv, lDist, dist;

	//Calculate the distance from odopose to endpoint
	dist = sqrt(pow((dstOdo.x - lastOdo.x),2) + pow((dstOdo.y - lastOdo.y),2));

	//Calculate line slope between points
	//As pose is relative, one point is (0,0)
	if (lastOdo.x != dstOdo.x) {
		a = (dstOdo.y - lastOdo.y) / (dstOdo.x - lastOdo.x);
		aInv = - 1.00 / a;
		bInv = dstOdo.y - aInv * dstOdo.x;

		//printf("Perpendicular line is y = %f x + %f\n",aInv,bInv);

		lDist = (aInv * lastOdo.x + bInv - lastOdo.y) / sqrt(aInv * aInv + 1);
		//printf("Distance to line %f\n",lDist);


	} else { //Vertical line cannot use slope line notation

		return 1.0;
	}

	if (verbose) printf("Distance to endpoint %f (lDist : %f)\n",dist, lDist);
	return dist;

}

/** Update route elements and switch to next route elements when needed */
int UResPlanner::updateRoute(void) {

	bool reChanged = false;
	bool evt98 = false;
	
	destinationReached(); //Check distance

	//Has event 98 been published
	evt98 = smrIf->testEvent(98,NULL,true);	  	//Destination reached
	if (!doubleRange) doubleRange = smrIf->testEvent(90,NULL,true); //Double detection range
	if (!detectRange) detectRange = smrIf->testEvent(91,NULL,true); //Within detection range
 	if (!stopRange) stopRange = smrIf->testEvent(92,NULL,true); //Last stopping range
	
	if (verbose && evt98) printf("Event 98 caught\n");

	  //Detect if destination is reached
	  if ((int)currRe->getValued() == (int)rPlan.size()-1) {
		  //if (destinationReached() < endreached->getValued()) {
		  if (evt98) { //Check when commands run out
			  if(verbose) printf ("Mission destination %s reached\n",rPlan[(int)currRe->getValued()].endName.c_str());

			  //Make sure that housekeeper has everything updated
			  housekeeper();
			  //currentConn->setValues((rPlan[(int)currRe->getValued()].endName.c_str()),0,true);
			  startConn = rPlan[(int)currRe->getValued()].endName;

			  exe->setValued(0.0,0); //Execution stopped!
		  }
	  } 
	  else if (evt98 && nextReverse->getValued()) {

		  printf("Turn point reached, rotating robot and proceeding...\n");

		  //Turn the robot, if it must
		  smrIf->doSmrDrive("turn 180 @v0.1 @a0.1\n",0);
		  smrIf->sendUserEvent("ev",99);
		  smrIf->doSmrDrive("idle\n",0);
		  for (int i = 0; i < 1000; i++) {
				if (smrIf->testEvent(99,NULL,true)) {
					printf("Recieved event, done turning\n");
					break;
				}
				Wait(0.05);
			}

		  //Increment current route element counter
		  currRe->setValued(currRe->getValued() + 1.0);
		  reChanged = true;

	  }
	  else if (evt98) { //Commands has run out

		  printf("Commands has run out, destination reached\n");

		  //Increment current route element counter
		  currRe->setValued(currRe->getValued() + 1.0);
		  reChanged = true;

	  }
	 //Otherwise check if a waypoint is reached
	  else if ((detectRange) && !nextReverse->getValued()) {

		  reChanged = true;
		  //Print status
		  if (verbose) printf("Destination: %s reached\n",rPlan[(int)currRe->getValued()].endName.c_str());

		  //Increment current route element counter
		  currRe->setValued(currRe->getValued() + 1.0);

	  }
	  //Detect if the next path is short, then target is shiftet to the next
	  else if (doubleRange) {
		  if (verbose) printf("Into double detection range (%d, %f, %d)\n",rPlan.size(),currRe->getValued()+1.0,(rPlan.size() <= currRe->getValued()+1.0));
		  //Is there a next waypoint
		  if (rPlan.size() > currRe->getValued()+1.0) {
			  if (verbose) printf("Checking if there is a short path in next waypoint\n");
			  //If the length is shorter than half of detection range, then select next waypoint
			  if (rPlan[(int)currRe->getValued()+1].routeEdge.length < wpreached->getValued()) {
				  currRe->setValued(currRe->getValued() + 1.0);
				  reChanged = true;
			  }
		  }
	  }

	//Update current node infomation
	if (reChanged) {
		//currentConn->setValues(rPlan[(int)currRe->getValued()].startName.c_str(),0,true);
		startConn = rPlan[(int)currRe->getValued()].startName;
		doubleRange = false; //Double detection range
		detectRange = false; //Within detection range
		stopRange = false; //Last stopping range
	}
	 
	return 1;


}


/** Pose transformation function
 *
 * Returned pose is the local pose projected into the
 * global pose coordinate system
 */
UPose UResPlanner::poseTransform(UPose local, UPose global) {

	UPose trans;

	//Transform coordinates
	trans.x = cos(global.h)*local.x - sin(global.h)*local.y + global.x;
	trans.y = sin(global.h)*local.x + cos(global.h)*local.y + global.y;
	trans.h = local.h + global.h;

	//Limit angle to +/-Pi
	if      (trans.h > M_PI)  trans.h -= 2*M_PI;
	else if (trans.h < -M_PI) trans.h += 2*M_PI;

	//Return the new pose
	return trans;

}




