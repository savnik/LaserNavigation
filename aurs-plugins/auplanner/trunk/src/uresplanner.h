/** \file uresplanner.h
 *  \ingroup Mission Planner
 *  \brief UResource class for Robot Mission planner
 *
 *  Robot mission planner plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1786 $
 *  $Date: 2012-01-13 15:28:44 +0100 (Fri, 13 Jan 2012) $
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

#ifndef URESPLANNER_H
    #define URESPLANNER_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/ulogfile.h>
#include <ausmr/uressmr.h> //Smr plugin

//Mapbase plug-in header
#include <uresmapbase.h>

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

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Anders Billesø Beck
*/
class UResPlanner : public UResVarPool, public ULogFile, public dglClipperCallback
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResCron) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResPlanner();
  /**
  Destructor */
  virtual ~UResPlanner();
  /**
   * Initialize resource */
  void UResPlannerInit();

  /**
   print status to a string buffer */
  virtual const char* print(const char * preString, char * buff, int buffCnt);
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
  The server will offer a resource pointer by this call.
  If the resource is used, please return true. */
  bool setResource(UResBase * resource, bool remove);

  virtual void createResources();

// the above methods are used be the server core and should be present
// the print function is not strictly needed, but is nice.


public:
  // Public functions that this resource provides for all users.

 /** Enummeration types for robot orientation and move-type */
  enum    dirType  {INBOUND=0, OUTBOUND};
  enum    moveType {OWNNODE=0,EXTNODE, INTNODE,PARENTNODE};
  /**
  Run the time loop to test for jobs to be activated.
  This call do not return until the threadStop flag is set true. */
  void run();
  /**
   * Service function for push queue - to store owner of function */
  int findFunctionOwner(const char * tagName);
  /**
   * Plan a route between two map locations
   **/
  int   planRoute(const char*, const char*);
  /** Add a waypoint to the route planning queue */
  int     addWaypoint(char *);
  /** Assign a start pose
   *  Takes a map connector in string form and a direction type
   *  The direction type determines if the robot is facing in or out
   *  bound in the assigned connector
   */
  int     setStartPose(char*, dirType, char*);
  /** Create a XML string containing the mission with all waypoints and route elements
   *  The function returns a c++ string object, if any waypoints are loaded.
   *  If no waypoints are loadted, an empty string is returned
   */
  string getMissonXML(void);

 /** Activate mission execution */
  bool    setExecute(bool);

  /**
   *Start read thread
   * \return Returns true if the read thread started. */
  bool start();
  /**
   * Stop read thread - and wait for thread join 
   * \param andWait wait for thread to finish (ignored, waits always)
   */
  void stop(bool andWait);
  
  /** Clear the planned mission and prepare for a new one, current pose is preserved */
  int clearMission(void);

protected:
  /** Interface function for callback for the route planner callback.
   *  (overwriting dglClipperCallback function)
   *  This allows this class to handle clipping for the route
   *  planner.
   */
  virtual bool shortestPathClipper (edge*, edge *);
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();

  /**
   * Transform a UPose from local to global coordinates
  */
  UPose poseTransform(UPose local, UPose global);


  //TEst function
  void test(void);

  /************** Planning related function *********************************/
  /** House keeping function
  *
  * This function must be called periodically to update status variables
  * and refresh the set of code generator rules
  */
  bool housekeeper(void);
  /** Function to calculate if robot is in or outbound when finishing an edge
   */
  moveType getMove(edge *dEdge);
  /** Function to calculate what type of move the robot is doing
   */
  dirType  getDir(edge *dEdge);

  /** Send a command string to the SMR interface */
  bool sendSmr(string cmdStr);

  /** Check for a userevent through the SMR interface */
  bool checkSmrEvent(int event);


  /************** Movement related functions ********************************/
  /** Update the poses according to current odometry position and map poses
   */

  int     updateDestination(void);
  double  destinationReached(void);
  int     updateRoute(void);

public:
  /** Pointer to global mapbase resource
   */
  UResMapbase *mapbase;
  /** Pointer to global SMR interface resource */
  UResSmrIf *smrIf;
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

  vector<route> rPlan;
  vector<waypoint> wpPlan;

protected:
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;

  /**
   * Global variable pool
   */
  UResVarPool * varGlobal;

  bool  executing;

    /** Pointer to global varpool resource
     */
    UVarPool    *varPool;
    /** Pointer to the SMR plugin varpool
     */
    UVarPool *smrPool;
    /** Mappose element */
    UResPoseHist *mappose;
    /** Odopose element */
    UResPoseHist *odopose;
    /** Pointer to the connected flag from the SMR variable pool
     */
    UVariable *smrConnected;
    /** Public pool variables for mission state */
    UVariable *period;
    UVariable *endreached;
    UVariable *wpreached;
    UVariable *nextWp;
    UVariable *totalWp;
    UVariable *totalRe;
    UVariable *currRe;
    UVariable *currWpRe;
    UVariable *exe;
    UVariable *reverse;
    UVariable *nextReverse;
    UVariable *currentConn;
    UVariable *currentDir;
    UVariable *dMethod;
    UVariable *defaultSpeed;
    UVariable *defaultAccel;

    //Poses for calculation
    UPose dstMapRelative;
    UPose dstOdoRelative;
    UPose dstRefRelative;
    UPose dstOdo;
    UPose dstMap;
    UPose lastMap;
    UPose lastOdo;
    UPose refPose;

    //
    double      poseAngle;
    double      minTurnRadius;
    double      maxReverseDist;
    double	currentSpeed, currentAccel;
    
    dirType     nextDir;
    moveType    currMove;
    moveType    nextMove;
    int         loopCounter;
    bool        testVar;
    int         activeId;
    bool 	doubleRange;
    bool	detectRange;
    bool	stopRange;
public:
    string      startConn;
    dirType     currDir;
};

#endif

