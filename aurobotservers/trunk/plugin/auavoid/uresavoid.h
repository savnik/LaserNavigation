/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                   *
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

#ifndef URESAVOID_H
#define URESAVOID_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/uobstacle.h>

#include "ureactivepath.h"



/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResAvoid : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResAvoid) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResAvoid()
  { // set name and version
    setResID(getResClassID(), 200);
    UResAvoidInit();
  };
  /**
  Destructor */
  virtual ~UResAvoid();
  /**
   * Initialize class */
  void UResAvoidInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "avoid"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 169; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  print status to a string buffer */
  inline virtual const char * snprint(const char * preString, char * buff, int buffCnt)
  { return print(preString, buff, buffCnt); };
  /**
  The server will offer a resource pointer by this call.
  If the resource is used, please return true. */
  bool setResource(UResBase * resource, bool remove);
  /**
  Open logfile - the name is always avoid.log and is placed in the dataPath.
  The full name is available in 'logAvoidName'. */
  bool openLogAvoid();
  /**
  Is the logfile open */
  inline bool isLogAvoidOpen()
  {
    if (paths->par.logAvoid != NULL)
      return paths->par.logAvoid->isOpen();
    else
      return false;
  };
  /**
  Close the logfile */
  void closeLogAvoid();
  /**
  Get the logfile name */
  const char * getLogAvoidFileName()
  { return logAvoidName; };

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  /**
  * Find path to this destination - and this desired (maximum) end velocity.
  * Return the time of calculation (pose time) in 'tod'.
  * NB! remember to call result->unlock() when no manoeuvre is longer needed.
 * \param directTestOnly to test direct path
  * \returns man sequence if path is available and usable */
  UManSeq * findPathToHere(UPose exitPose, double endVel, bool exitPoseRel,
                           UTime * tod, bool directTestOnly);
  /**
  Get pointer to path pool */
  inline UAvoidPathPool * getPathPool()
  { return paths; };
  /**
  * The varPool has methods, and a call to one of these are needed.
  * Do the call now and return (a double sized) result in 'value'.
  * Return true if the method call is allowed.
  * "getAvoidPath", "ddd": Find path to exit position.
    The found route is returned in the 'returnStruct' pointer array.
    The pointers in the array will be redirected to the static set of
    manoeuvre buffers in the avoid function, i.e. the result should be
    used before the next call to 'getAvoidPath'. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
   * The varPool has methods, and a call to one of these are requested. The parameters
   * hold the requested method name, parameter list, parameters, and an array of pointers for the result. All parameters and results are of type UVariable.
   * \param name is the method name (within this variable pool)
   * \param paramOrder is the parameter sequence d=double structure or array
   * \param params is the array of UVariable pointers with the values
   * \param returnStruct is an array of pointers (to UDataBase structures,
   * either UVariable or another class based on UDataBase), may be NULL pointers
   * or pointers to actual result variables or classes - as specified in the method definition.
   * \param returnStructCnt is the actual number of returned structure pointers.
   * \return true if the method is handled (recognized) */
  virtual bool methodCallV(const char * name, const char * paramOrder,
                   UVariable * params[],
                   UDataBase ** returnStruct,
                   int * returnStructCnt);
  /**
  Is the obstacle avoidance a rev1 or rev 2 calculation */
  bool useRev2();
  /**
  Get current pose from pose-history */
  inline UPose getCurrentPose()
  {
    UPose pose;
    if (poseHist != NULL)
       pose = poseHist->getNewest(NULL);
    return pose;
  }
  /**
  code front-left and front-right points as an xmlPose
  \param buff is the buffer to deliver result into.
  \param buffCnt is the size of the bugger in bytes.
  \returns a pointer to the buffer. */
  const char * codeXmlRobotFront(char * buff, const int buffCnt);
  /**
   * Get last avoidance calculation time */
  UTime getCalcTime()
  { return varCalcTime->getTime(); };

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();

private:
  /**
  Copy footprint polygons to the polygon resource for debug purposes.
  \param resAP is the path definition with the footprint polygons.
  \param oldCnt is the prevoius number of polygons (that are to be deleted) */
  void copyFootprintPolys(UAvoidPath2 * resAP, int oldCnt);
  /**
  Copy cell decomposition cells as polygons in the polygon plugin using the name cellPoly.XXX
  \param svcg is the cell decomposition structure */
  void copyCellPolys(UAvoidCellGraph * avcg);
  /**
  Copy all grouped and added obstacles to polygon plugin,
  that is all obstacles in resAp.aogs with a point count > 2. */
  void copyAvoidObstacles(UAvoidPath2 * resAP);

protected:
  /**
  * Road lines to respect */
  UReacRoadLines roadLines;
  /**
  * Obstacles to avoid */
  UReacObstGrps obsts;
  /**
  Pool of alternative paths generated in responce to the obstacles and road lines */
  UAvoidPathPool * paths;
  /**
  Pose history */
  UResPoseHist * poseHist;
  /**
  Pose history */
  UResPoseHist * mapPose;
  /**
  Global variables to access obstacles and road lines */
  UResVarPool * globVar;
  /**
  Filename for obstacle avoidance logging */
  char logAvoidName[MAX_FILENAME_LENGTH];

protected:
  /**
  Index to variable with manoeuvre count */
  UVariable * varManCnt;
  /**
  Index to variable most problematic front left position */
  UVariable * varFrontLeft;
  /**
  Index to variable most problematic front right position */
  UVariable * varFrontRight;
  /**
  Index to boolean flag selecting wich obstacle avoidance method to use */
  UVariable * varRev2;
  /**
  Index to boolean flag selecting wich cell decomposition method */
  UVariable * varRev2cell;
  /**
  Index to boolean flag selecting wich obstacle avoidance method to use */
  UVariable * varSerial;
  /// (old method - (desired) minimum distance to obstacle
  UVariable * varObstMinDist;
  /// (old method - absolute minimum distance to obstacle
  UVariable * varObstMinMinDist;
  /// maximum lateral acceleration
  UVariable * varMaxAcc;  // maximum lateral acceleration
  /// maximum acceleration in turns
  UVariable * varMaxTurnAcc;
  /// minimum allowed turn radius
  UVariable * varMinTurnRadius;
  /// number of nested levels
  UVariable * varMaxNestedSpawns;
  /// maximum number of loops for midPose search
  UVariable * varMaxAvoidLoops;
  /// maximum spawn count limit
  UVariable * varMaxSpawnCount;
  /// minimum allowed clearence to obstacles (new method)
  UVariable * varClearenceMinimum;
  /// desired clearence to obstacles (new method)
  UVariable * varClearenceDesired;
  /// flag to set if obstacles (all) are to be ignored - path to bosition only)
  UVariable * varIgnoreObstacles;
  /// When set to true more debug info is returned to client and logged
  UVariable * varCrashTest;
  /// Use drivon manoeuver planning
  UVariable * varUseDriveon;
  /// drivon angle gain parameter
  UVariable * varDriveonGA;
  /// driveon distance gain parameter
  UVariable * varDriveonGD;
  /// allow forward solutions only - positive or negative (radians)
  UVariable * varForwardAngle;
  /// number of obstacle groups used for obstacle avoidance
  UVariable * varMaxOG;
  /// maximumber of tantents to search for the visibility graph
  UVariable * varMaxTangentDepth;
  /// Maximum number of tangent sequences testes, if first sequences fails in manoeuvre generation.
  UVariable * varMaxTangentToMan;
  /// debug maximum of allowed close obstacles that are to be ignored
  UVariable * varIgnoreCloseObst;
  /// accept mid-point solution after this number of calculations.
  UVariable * varAcceptSolution;
  /// should destination pose line be used in a line drive as soon as possible
  UVariable * varFollowLineOnLastPose;
  /// debug flag - use any solution
  UVariable * varUseAnyResult;
  /// debug flag - makeCellPolygon - 0 = no cells, 1= all cells 2=traversed cells
  UVariable * varMakeCellPolygon;
  /// debug limit of polygon size (for display only)
  UVariable * varMakeCellPolygonMaxY;
  /// debug flag - makeCellPolygon - boolean
  UVariable * varMakeFootprintPolygon;
  /// debug make also generated obstacles to polygons
  UVariable * varMakeAvoidObstPolygon;
  /// debug flag - make polyline from the cell path cost lines
  UVariable * varMakeCellCostLine;
  /// debug flag - make polyline from vertex-points from current to target position
  UVariable * varMakeCellVertexLine;
  /// debugDump flag, when true a full data dump (or as much as planned in code) is dumped to logfile.
  UVariable * varDebugDump;
  /// debugDump flag, when true a full data dump (or as much as planned in code) is dumped to logfile.
  UVariable * varCalcTime;
private:
  /** a common string buffer for logging */
  static const int MSL = 10000;
  char sBuff[MSL];
  
};

#endif

