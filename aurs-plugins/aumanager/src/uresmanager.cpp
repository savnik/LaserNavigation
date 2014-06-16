/** \file uresmanager.cpp
 *  \ingroup Mission manager
 *  \brief UResource class for Robot Mission manager
 *
 *  Robot mission manager plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 55 $
 *  $Date: 2009-04-01 00:02:17 +0200 (Wed, 01 Apr 2009) $
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
 #define VERSION     "3.0"
 #define REVISION    "$Rev: 55 $:"
 #define DATE        "$Date: 2009-04-01 00:02:17 +0200 (Wed, 01 Apr 2009) $:"
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

#include "uresmanager.h"

void UResManager::UResManagerInit()
{
  // to save the ID and version number
  setLogName("manager");
  openLog();

  // create status variables
  createBaseVar();
  // the this class execute the push jobs
  threadRunning = false;
  verbose = false;
  executing = false;

  // start job starter thread
  start();
}

///////////////////////////////////////////

UResManager::~UResManager()
{
  // stop job starter thread
  stop(true);
  // the jobs are terminated in the job destructor
}

///////////////////////////////////////////

const char* UResManager::print(const char * preString, char * buff, int buffCnt)
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

void UResManager::createBaseVar()
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

    addMethod("addwaypoint" ,"s", "Add a waypoint to the robot mission");
    addMethod("startpos"    ,"ss","Set the robot start possition and pose");

    //Pass call to robot manager, so it can create it's variables
    rManager.createBase(this);

}

//////////////////////////////////////////////

bool UResManager::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  double retValue = 0;
  // evaluate standard functions
  if ((strcasecmp(name, "startnode") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from startnode
    if (rManager.rPlan.empty() ||
       (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement())) ||
        rManager.rPlan[rManager.currentRouteElement()].startNode.features.count(strings[0]) == 0) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()].startNode.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "endnode") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from endnode
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement())) ||
          rManager.rPlan[rManager.currentRouteElement()].endNode.features.count(strings[0]) == 0) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()].endNode.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "nextnode") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from next end-node
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement()+1)) ||
         (rManager.rPlan[rManager.currentRouteElement()+1].endNode.features.count(strings[0]) == 0)) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()+1].endNode.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "startconn") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from startnode
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement())) ||
          rManager.rPlan[rManager.currentRouteElement()].startConn.features.count(strings[0]) == 0) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()].startConn.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "endconn") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from startnode
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement())) ||
          rManager.rPlan[rManager.currentRouteElement()].endConn.features.count(strings[0]) == 0) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()].endConn.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "nextconn") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement()+1)) ||
         (rManager.rPlan[rManager.currentRouteElement()+1].endConn.features.count(strings[0]) == 0)) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()+1].endConn.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "edge") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from edge
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement())) ||
          rManager.rPlan[rManager.currentRouteElement()].routeEdge.features.count(strings[0]) == 0) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()].routeEdge.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "nextedge") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    // Get value from startnode
      if (rManager.rPlan.empty() ||
         (rManager.rPlan.size() <= ((size_t)rManager.currentRouteElement()+1)) ||
         (rManager.rPlan[rManager.currentRouteElement()+1].routeEdge.features.count(strings[0]) == 0)) {
          retValue = -1.0;
      } else {
          retValue = rManager.rPlan[rManager.currentRouteElement()+1].routeEdge.features[strings[0]];
      }
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }

  else if ((strcasecmp(name, "test") == 0))
  {
      retValue = rManager.updateRuleTest();
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
      retValue = rManager.start();
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "stop") == 0))
  {

      retValue = rManager.stop(true);
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "addwaypoint") == 0))
  {
      retValue = rManager.addWaypoint(strings[0]);
    // return result - if a location is provided
    if (value != NULL)
      *value = retValue;
      // it is good practice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "startpos") == 0))
  {
      printf("Not implemented yet...\n");
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

bool UResManager::setResource(UResBase * resource, bool remove)
{
  bool result = false;

  if (resource->isA(UResVarPool::getResID()))
  {
    result = true;
    if (remove)
      varGlobal = NULL;
    else if (varGlobal != resource)
      varGlobal = (UResVarPool *)resource;
    else
      // not used
      result = false;
  }
  else if (resource->isA(UResMapbase::getResClassID())) {
      result = true;
      if (remove) mapbase = NULL;
      else if (mapbase != resource) {
          mapbase = (UResMapbase*)resource;
      } else result = false;
  }
  else
    result = UResVarPool::setResource(resource, remove);
  return result;
}

/**
 * Finish initialization by creating and adding resources
 **/
void UResManager::createResources(void) {
    if (varGlobal != NULL) {
        rManager.loadVarpool(varGlobal->getVarPool());
    }

    //Load the mapbase into robot state
    if (mapbase != NULL) {
        rManager.loadMapbase(mapbase);
    }

}

//////////////////////////////////////////////////

/**
  * Plan a route between two map location
  **/
int UResManager::planRoute(const char* fromPos, const char* toPos) {

      string from   = fromPos;
      string to     = toPos;
      int   retVal = 0;

      //Execute planner
      if (mapbase->graphmapper.calculateRoute(from,to,&rManager.rPlan) < 0) {
          retVal =  -1;
      } else {
          printf("Planning successfull - Route:\n");
          for (size_t i = 0; i < rManager.rPlan.size(); i++) {
              printf("   %s -> %s   (length: %d)\n",rManager.rPlan[i].startName.c_str(),rManager.rPlan[i].endName.c_str(),rManager.rPlan[i].edgeLength);
          }
          retVal = 1;
      }
      
      return retVal;
}

  /**
   *Execute a previously planned route
   */
int   UResManager::executePlan(void) {
    
    if (!rManager.rPlan.empty()) {
        executing = true;
        return true;
    } else {
        return false;
    }

}

void UResManager::run()
{ //
  threadRunning = true;
  UTime t;
  /*int           count = 0, exeCmd = -1;
  double        retVal, tempVal;
  rManager.currentRouteElement() = 0;
  int           sentCount = -1;
  string        execStr;*/

  // wait to allow init script to finish
  Wait(5.0);

  while (not threadStop)
  {
    t.Now();

    //Debug...
    if (verbose)
      printf("Something testing stuff..\n");

         
/*    if (!executing) {
        Wait(1.0);
    } else {
        //GEnerate and send the first plan
        if(sentCount < 0) { //First plan
            execStr = generator.initializeMission(rManager.rPlan);
            sendSmr(execStr);
            execStr = generator.generateSwitch(rManager.rPlan,rManager.currentRoute);
            sendSmr(execStr);
            sentCount++;
        }

        if (rManager.currentRoute < rManager.rPlan.size()) {
            for (size_t i = 0; i < generator.cmds.size(); i++) {
                tempVal = generator.cmds[i].eventId;
                callGlobal("smr.gotEvent","d",NULL,&tempVal,&retVal,NULL,&count);
                if (retVal > 0.0) {
                    exeCmd = i;
                    //Generate a new switch
                    execStr = generator.generateSwitch(rManager.rPlan,rManager.currentRoute);
                    sendSmr(execStr);
                    sentCount++;
                }
            }
            if (exeCmd >= 0) {
                printf("Executing[%d] %s\n",sentCount,generator.cmds[exeCmd].command.c_str());
            }
            //Test for success
            tempVal = generator.nextCmds[0].eventId;
            callGlobal("smr.gotEvent","d",NULL,&tempVal,&retVal,NULL,&count);
            if (retVal > 0.0) {
                rManager.currentRoute++;
                //Generate a new switch
                execStr = generator.generateSwitch(rManager.rPlan,rManager.currentRoute);
                sendSmr(execStr);
                sentCount++;
                printf("** Switching to mission element %d\n",rManager.currentRoute);
            }
         } else {
             rManager.rPlan.clear();
             sentCount = -1;
             exeCmd = -1;
             executing = false;
             printf("MISSION FINISHED!\n");
         }
     } */
     Wait(5.0);
  }
  threadRunning = false;

}


bool UResManager::sendSmr(string cmdStr) {

      const size_t      MAXTX       = 250; //Maximum length of accepted SMR-CL string
      size_t            found = 0;
      string            sendString;
      char              *txStr = new char[MAXTX+1];
      double            retVal;
      int               count = 0;

      while (found != cmdStr.npos) {
          found = cmdStr.find_first_of('\n');
          sendString = cmdStr.substr(0,found+1);
          cmdStr = cmdStr.substr(found+1);
          strncpy(txStr,sendString.c_str(),MAXTX+1);
          if (sendString.length() > 0) {
            callGlobal("smr.send","s",&txStr,NULL,&retVal,NULL,&count);
            printf("%s",txStr);

          }  
      }
      return true;

}
///////////////////////////////////////////////////

void * startManagerThread(void * obj)
{ // call the hadling function in provided object
  UResManager * ce = (UResManager *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResManager::start()
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

void UResManager::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    //pthread_join(threadHandle, NULL);
//TODO Re-enable thread-join..
  }
}

/////////////////////////////////////////////////////

int UResManager::findFunctionOwner(const char * tagName)
{ // one function owner only (me)
  return 1;
}


//////////////////////////////////////////////////////

// bool UResManager::executePushFunction(int functionIndex, UServerInMsg * msg, void * extra)
// {
//   UCronJob * cj;
//   //
//   cj = getFreeJob();
//   if (cj != NULL)
//   {
//     cj->active = true;
//     cj->cmdStr = msg->message;
//     startCronJobThread(cj);
//   }
//   return (cj != NULL);
// }



