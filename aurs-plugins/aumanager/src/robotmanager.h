/** \file robotstate.cpp
 *  \ingroup Robot
 *  \brief Robot state monitor class
 *
 * This lib monitors and controls the robot-state
 *
 *  \author Anders Billesø Beck
 *  $Rev: 46 $
 *  $Date: 2009-03-03 17:33:41 +0100 (Tue, 03 Mar 2009) $
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

#include <vector>

#include <uresmapbase.h>

#include "smrclgenerator.h"

#ifndef _ROBOTMANAGER_H
#define	_ROBOTMANAGER_H

/** Class to contain information on route waypoints */
class waypoint {
public:
    vector<route> plan;         /**< Route elements in the waypoint route */
    string        startConn;    /**< Name of start connector */
    string        endConn;      /**< Name of end connector */
    int           startConnId;  /**< Id of start connector */
    int           endConnId;    /**< Id of end connector */
    int           routeLength;  /**< Length of the route from wp to wp */
};

/** Primary manager function base
 *
 *  This class manages the robot state, handles route planning and
 *  robot communication
 */
class robotManager : public dglClipperCallback {
public:
    robotManager();
    robotManager(const robotManager& orig);
    virtual ~robotManager();

public:
    // *** Public variables ***

    /** Enummeration types for robot orientation and move-type */
    enum    dirType  {INBOUND=0, OUTBOUND};
    enum    moveType {OWNNODE=0,EXTNODE, INTNODE,PARENTNODE};

    vector<route> rPlan;
    vector<waypoint> wpPlan;

    // *** Public functions ***

    /** Load the mapbase into the robot state module */
    int     loadMapbase(UResMapbase *);
    /** Load the root VarPool into the robot state module **/
    bool    loadVarpool(UVarPool *);
    /** Create resources in the local varpool */
    bool    createBase(UResVarPool *);
    /** Start the manager thread 
     *  Starting the thread automatically activates the robot control.
     *  Do not attempt to control the robot through other functions with
     *  this enabled */
    bool    start(void);
    /** Stop the manager thread 
     *  Stopping the thread, also terminates the manager control.*/
    bool    stop(bool);
    /** Thread entry-point for the robot control thread */
    bool robotThread(void);

    /** Assign a start pose
     *  Takes a map connector in string form and a direction type
     *  The direction type determines if the robot is facing in or out
     *  bound in the assigned connector
     */
    int     setStartPose(char*, dirType);

    /** Add a waypoint to the route planning queue */
    int     addWaypoint(char *);
    /** Activate mission execution */
    bool    setExecute(bool);

    /** Create a XML string containing the mission with all waypoints and route elements
     *  The function returns a c++ string object, if any waypoints are loaded.
     *  If no waypoints are loadted, an empty string is returned
     */
    string getMissonXML(void);
    /* Functions to export parameters */
    /** Return id of current edge, return -1 if no mission is loaded */
    int currentRouteElement(void);
    /** Returns the total number of route elements in the mission */
    int totalRouteElements(void);

    /* Test function to pass */
    int updateRuleTest (void) {
        int retval;
        retval = 0;
        test = true;
        return retval;
    }




protected:
    /** Interface function for callback for the route planner callback. 
     *  (overwriting dglClipperCallback function)
     *  This allows this class to handle clipping for the route
     *  planner.
     */
    virtual bool shortestPathClipper (edge*, edge *);

private:
    // ***  Private functions   ***
    dirType     getDir(edge *);
    moveType    getMove(edge *);
    bool        housekeeper(void);

    /** Transmit a string to the connected robot
     *  The function returns true, if transmission was successfull
     *  and false if the tx fails if the smr varpool is not found or transmission
     *  fails somehow
     */
    bool sendSmr(string);

    /** Reload the switch
     *
     * Send the reload-signal to the switch statement.
     * The second parameter indicates, if the condition value should be changed
     * at the reload. If it is different from 0, a new condition will be set
     */
    bool reloadSwitch(void);
    /** Set a command as failed
     *
     * Set a failed signal for a command in MRC
     */
    bool failCommand(int);

private:
    // *** Private variables ***
    /** Thread handle for the robot control thread. */
    pthread_t threadHandle;
    /** Is thread actually running */
    bool threadRunning;
    /** Should thread stop - terminate */
    bool threadStop;
   /** Pointer to global mapbase resource
    */
    UResMapbase *mapbase;
    /** SMR-CL Code generator lib
     */
    smrclGenerator generator;
    /** Pointer to global varpool resource
     */
    UVarPool    *varPool;
    /** Pointer to the SMR plugin varpool
     */
    UVarPool *smrPool;

    /** Pointer to the connected flag from the SMR variable pool
     */
    UVariable *smrConnected;
    /** Public pool variables for mission state */
    UVariable *nextWp;
    UVariable *totalWp;
    UVariable *totalRe;
    UVariable *currRe;
    UVariable *currWpRe;
    UVariable *exe;
    UVariable *reverse;
    UVariable *nextReverse;

    //
    double      poseAngle;
    double      minTurnRadius;
    double      maxReverseDist;
    dirType     currDir;
    dirType     nextDir;
    moveType    currMove;
    moveType    nextMove;
    string      startConn;
    string      currentConn;
    int         loopCounter;
    bool        test;

    int         activeId;

};

#endif	/* _ROBOTMANAGER_H */

