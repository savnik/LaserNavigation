/** \file robotManager.cpp
 *  \ingroup Robot
 *  \brief Robot state monitor class
 *
 * This lib monitors and controls the robot-state
 *
 *  \author Anders Billesø Beck
 *  $Rev: 46 $
 *  $Date: 2009-03-03 17:33:41 +0100 (Tue, 03 Mar 2009) $
 */

#include "smrclgenerator.h"

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
 #define VERSION  "1.0"
 #define REVISION "$Rev: 46 $:"
 #define DATE     "$Date: 2009-03-03 17:33:41 +0100 (Tue, 03 Mar 2009) $:"
/***************************************************************************/

#include <pthread.h>
#include <string>
#include <string.h>
#include <math.h>

#include "robotmanager.h"

robotManager::robotManager() {

    //Default values
    minTurnRadius   = 0.10;
    maxReverseDist  = 0.30;
    loopCounter     = 0;

    //Init thread variables
    threadRunning   = false;
    threadStop      = false;

}

robotManager::robotManager(const robotManager& orig) {

}

robotManager::~robotManager() {
      stop(1); //Kill the robot execution thread  
}

/** Load the mapbase into the robot state module */
int robotManager::loadMapbase(UResMapbase *loadMapbase) {

    mapbase = loadMapbase;

    //Load the robot state clipper into the mapbase
    if (mapbase != NULL) {
        mapbase->graphmapper.loadClipper(this);
        return 1;
    } else {
        return -1;
    }
}

/** Load the variable pool into the robot state module */
bool robotManager::loadVarpool(UVarPool *loadVarPool) {

    if (loadVarPool != NULL) {
        varPool = loadVarPool->getRootVarPool();
        generator.setVarpool(varPool);
        return true;
    } else
        return false;
}

/* Create system variables in the local varpool */
bool robotManager::createBase(UResVarPool *manager) {

    if (manager == NULL) return false;

    //Use pointer to base resource to create variables
    nextWp      = manager->addVar("nextWaypoint",   0.0, "d", "Next target waypoint in the mission");
    totalWp     = manager->addVar("totalWaypoints", 0.0, "d", "Total number of waypoints");
    totalRe     = manager->addVar("totalRoute",     0.0, "d", "Total number of route elements");
    currRe      = manager->addVar("currentRoute",   0.0, "d", "Current route element");
    currWpRe    = manager->addVar("currentWpRoute", 0.0, "d", "Current route element in waypoint route");
    exe         = manager->addVar("execute",        0.0, "d", "Start the execution of mission");
    reverse     = manager->addVar("reverse",        0.0, "d", "Should the robot change direction to follow the current edge");
    nextReverse = manager->addVar("nextReverse",    0.0, "d", "Should the robot change direction to follow the next edge");

    return true;

}

/** Thread gateway for the Robot execution */
void * startRobotThread(void * obj)
{ // call the hadling function in provided object
  robotManager * rs = (robotManager *)obj;
  rs->robotThread();
  pthread_exit((void*)NULL);
  return NULL;
}

bool robotManager::start()
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
              &startRobotThread, (void *)this) == 0);
  }
  return result;
}

///////////////////////////////////////////////////

bool robotManager::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;

    //Wait loop, to avoid deadlocks
    if (andWait) {
        for (int i = 0; threadRunning; i++) {
            usleep(1000); //Wait for thread to exit
            if (i > 100) break;
        }
        //Detect if thread has terminated (and join it, if it has)
        if (!threadRunning) {
            //pthread_join(threadHandle, NULL);
            return true;
        } else {
            return false;
        }
    }
  }
  //Do not wait for thread to stop here
  return true; 
}

/** Add waypoint to the mission queue */
int robotManager::addWaypoint(char *wpName) {
    
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

 /*  Activate mission execution */
 bool robotManager::setExecute(bool go) {

     if (go)    exe->setValued(1,0);
     else       exe->setValued(-1,0);

     return true;

 }

/** House keeping function
 *
 * This function must be called periodically to update status variables
 * and refresh the set of code generator rules
 */
bool robotManager::housekeeper(void) {

    //Update orientation
    if(currRe->getValued() > 0) {
        currDir = getDir(&(rPlan[(int)currRe->getValued()-1].routeEdge));
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

    //Run the testcode async
    if (test) {
        //generator.processCommandsFromRule();
        printf("INIT:\n%s\n",generator.initializeMission(rPlan).c_str());
        printf("SWITCH:\n%s\n",generator.generateSwitch(rPlan,currRe->getValued(),true).c_str());
        test = false;
        printf("Curr [%d,%d]=%g Next [%d,%d]=%g\n",currDir,currMove,reverse->getValued(),nextDir,nextMove,nextReverse->getValued());
    }

    loopCounter++;

    return true;
}


/** Robot execution thread
 *
 * This thread monitors the flow execution from the connected robot.
 * It maintains the switch-statement flow in MRC, handles events, and
 * controls variables from MRC
 */
bool robotManager::robotThread(void) {

    threadRunning   = true;
    double          testEvent, testResult;
    int             structCnt = 0; //No structs are ever returned from calls to SMR pool
    string          txString;
    int             idActive = 0, currentActive = 0;

    //Load the varpool from the SMR-interface
    smrPool  = varPool->getStructDeep("smr.connected",false,NULL,0);
    if (smrPool == NULL) {
        printf("Manager: The SMR interface plugin is not yet loaded - Start failed!\n");
        threadStop = true;
    } else {
        smrConnected = smrPool->getLocalVariable("connected",0);
        if (smrConnected == NULL) {
            printf("Failed getting the smr.connected Uvariable\n");
            threadStop = true;
        }
    }

    //Robot control thread
    while (!threadStop) {
        if (smrConnected->getValued() <= 0) {
            //printf("The SMR is NOT! connected\n");
            
        } else {
            //Idle state
            if (exe->getValued() < 0) {
            }
            //Execution is just resat. '
            //Stop the robot and move to idle state
            else if (exe->getValued() == 0) {
                //Idle the robot
                txString = "flushcmds\n stop\n wait 8\n idle\n";
                sendSmr(txString);
                exe->setValued(-1,0);
                generator.setExecution(false);
            }
            //Start by flushing buffers
            else if (exe->getValued() == 1) {
                //Only proceed, if there actually IS a plan...
                if (wpPlan.size() > 0) {
                    //Signal to the smrcl-generator that execution is started
                    generator.setExecution(true);

                    //Flush the robot queue, before starting
                    txString = "flushcmds\n";
                    sendSmr(txString);
                    //execute = 2;                //printf("SWITCH:\n %s\n",txString.c_str());

                    //Generate initialization string
                    txString = generator.initializeMission(rPlan);
                    sendSmr(txString);
                    //printf("INITSTRING:\n%s\n",txString.c_str());
                    //Generate first switch statement (don't wait for fresh data)
                    txString = generator.generateSwitch(rPlan,(int)currRe->getValued(),true);
                    sendSmr(txString);
                    //Generate first switch statement
                    //txString = generator.generateSwitch(rPlan,(int)currRe->getValued());
                    //sendSmr(txString);
                    exe->setValued(2,0);
                }
            }
            // Mission execution is running 
            else if (exe->getValued() > 1) {

                //Check for MRC events to know when to generate a new switch statement
                for (size_t i = 0; i < generator.cmds.size(); i++) {
                    testEvent = generator.cmds[i].id;
                    smrPool->callLocal("gotEvent","d",NULL,&testEvent,&testResult,NULL,&structCnt);
                    if (testResult > 0.0) {
                        printf("Got event %f (result %f)\n",testEvent,testResult);
                        idActive = generator.cmds[i].id;
                        printf("ID %d\n",idActive);
                        //Generate a new switch statement for the queue (don't wait for fresh data)
                        txString = generator.generateSwitch(rPlan,(int)currRe->getValued(),false);
                        sendSmr(txString);
                    }
                }

                //Test if route element has been successfully traversed
                testEvent = generator.successId;
                smrPool->callLocal("gotEvent","d",NULL,&testEvent,&testResult,NULL,&structCnt);
                if (testResult > 0.0) {

                    //Generate a new switch statement for the queue
                    currRe->setValued(currRe->getValued()+1,0); //Increment route counter
                    idActive = generator.successId;
                    //Stop execution if no more route elements are in the queue
                    if (currRe->getValued() >= rPlan.size()) {
                        exe->setValued(-1.0,0);
                        generator.setExecution(false);
                        printf("Finished executing mission\n");
                    } else {
                        printf("Next element! Location %s\n",rPlan[(int)currRe->getValued()].startName.c_str());
                        //Run the house-keeping functions to have system updated, before new code is generated
                        housekeeper();
                        //Create a new switch and wait for fresh data
                        txString = generator.generateSwitch(rPlan,(int)currRe->getValued(),true);
                        sendSmr(txString);
                        //printf("NEXTSWITCH:\n %s\n",txString.c_str());
                        //Reload the switch
                        //reloadSwitch(0);
                    }
                 }

                 //Set the current active command in the smr-cl generator
                if (idActive > 0) {
                    currentActive = idActive;
                    printf("Got active %d\n",idActive);
                    generator.setActive(idActive);
                    idActive = 0;
                }

            }

        }

        //Run the house-keeping functions
        housekeeper();

        usleep(50000);

    }

    threadRunning = false;
    return threadRunning;
}

/* Transmit a string to the SMR */
bool robotManager::sendSmr(string cmdStr) {

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

/* Reload the switch statement */
 bool robotManager::reloadSwitch(void) {

     string txStr;

     txStr = "setvar \"reload\" 1\n";
     //Send the reload command to MRC
     sendSmr(txStr);
     return true;

 }

 /* Set a command as failed */
 bool robotManager::failCommand(int newCond) {

     string txStr;

     txStr = "setvar \"reload\" 1\n";
     //Send the reload command to MRC
     sendSmr(txStr);
     return true;

 }


/** Function to calculate if robot is in or outbound when finishing an edge
 */
robotManager::dirType robotManager::getDir(edge *dEdge) {

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
robotManager::moveType robotManager::getMove(edge *dEdge) {

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

/** Set robot startpose before planning
 */
int robotManager::setStartPose(char *startC, dirType dir) {
    
    //Find start connector
    connector *sConn = mapbase->graphmapper.getConnector(startC);

    if (sConn == NULL) return -1;

    //Assign startConnector and currentConnector
    startConn   = startC;
    currentConn = startC;

    //Dir is 0 if robot points inward into the node
    if (dir == INBOUND) {
        currDir = INBOUND;
        poseAngle = sConn->features["th"] + 180.0;
        if (poseAngle > 360.0) poseAngle -= 360.0;
    } else {
        currDir = OUTBOUND;
        poseAngle = sConn->features["th"];
    }
    return 1;
}

/** Generate a misson XML string */
string robotManager::getMissonXML(void) {

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

/* *** Data export functions *** */

int robotManager::currentRouteElement(void) {

    //Return from the UVariable
    return (int)currRe->getValued();

}

int robotManager::totalRouteElements(void) {
    return (int)totalRe->getValued();
}

/** Planning clipper function
 *
 * This function is used to interface the planning clipper in the
 * shortest path calculations.
 *
 * If the function returns true, the edge currEdge is not accepted for
 * the route planning.
 */
bool robotManager::shortestPathClipper (edge* currEdge, edge *prevEdge) {

    //Outcommented 
    /*moveType edgeMove;
    dirType  dirIn;
    double angleIn, angleOut, turnAngle, circRadius, connDist;

    //Extract the angle beeing turned

    //Get resulting robot direction from previous edge
    if(prevEdge == NULL) {
        dirIn = INBOUND;
    } else {
        dirIn = getDir(prevEdge);
    }

    if (currEdge->startconnector->id == 30) {
        if (prevEdge == NULL)
            printf("   TESTING: %s -> %s (%d) \n",currEdge->start,currEdge->end,currEdge->endconnector->id);
        else printf("  TESTING: (%s ->) %s -> %s (%d)\n",prevEdge->start,currEdge->start,currEdge->end,currEdge->endconnector->id);
    }

    //Get resulting robot move from current edge
    edgeMove = getMove(currEdge);

    //Some paths are in the 180 degrees opposite direction of the robot.
    //Traversing these edges requires that the robot can perform a stationary turn
    //(turn radius = 0) or can reverse along the edge.
    if (((dirIn == INBOUND) && ((edgeMove == EXTNODE)||(edgeMove == PARENTNODE))) ||
        ((dirIn == OUTBOUND) && ((edgeMove == INTNODE)||(edgeMove == OWNNODE)))) {
        //Discard the edge if the robot has a turn radius and cannot reverse along the edge
        if (minTurnRadius > 0.0) {
            if (maxReverseDist < currEdge->length) {
                if (prevEdge != NULL)
                    printf("Discarded (%s ->) %s -> %s because of radius/reverse\n",prevEdge->start,currEdge->start,currEdge->end);
                else
                    printf("Discarded (START ->) %s -> %s because of radius/reverse\n",currEdge->start,currEdge->end);
                return true;
            }
        }

    }

    //Now check if the turn radius required to traverse the edge is allowed by the robot
    //TODO: Add node orientation in turnradius clipper
    angleIn  = currEdge->startconnector->features["th"];
    angleOut = currEdge->endconnector->features["th"];
    if (angleIn < angleOut)  angleIn += 360.0;
    turnAngle = fabs((angleOut - angleIn) + 180.0);
    //Calculate the distance between the connector coordinates sqrt((x2-x1)^2+(y2-y1)^2)
    connDist = sqrt(
        pow((currEdge->endconnector->features["x"]-currEdge->startconnector->features["x"]),2.0)
        +pow((currEdge->endconnector->features["y"]-currEdge->startconnector->features["y"]),2.0));

    //Calculate the radius of the circle spanning between the two points
    circRadius = (currEdge->length/2)/(sin((turnAngle/2)*(M_PI/180.0)));

    //Check if required circle radius between the two points is less than the robot turn radius
    if (circRadius < minTurnRadius) {
        printf("Discarded %s -> %s because of turnradius(%f) > circRadius(%f) \n",
                currEdge->start,currEdge->end,minTurnRadius,circRadius);
        return true;
    }

    if (prevEdge == NULL)// return true;
       printf("ACCEPTED (START ->) %s -> %s (n:%d) \n",currEdge->start,currEdge->end,currEdge->endconnector->id);
    else
        printf("ACCEPTED (%s ->) %s -> %s\n",prevEdge->start,currEdge->start,currEdge->end);*/

    //Cancel blocked edges
    if (currEdge->blocked) return true;

    return false;

}

