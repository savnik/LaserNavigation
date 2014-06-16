/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#ifndef ULASERPATH_H
#define ULASERPATH_H

#include <ugen4/uline.h>
#include <urob4/uobstacle.h>
#include <umap4/uposev.h>
#include <umap4/umanseq.h>

#include "uavoidpath2.h"

/**
Maximum number of saved laser scans */
#define MAX_LASER_SCANS 60
/**
For navigation on on a road edge, the road edge must be
detected with some minimum length (away from newest
detected passable interval (in this path)) */
#define MINIMUM_ROAD_LENGTH 1.0
/*
Set to true for crash analysis.
It will allaw discarded paths to be reported for further analysis */
//#define CRASH_TEST true

/**
Road edge obstacle.
This obstacle is a line that must be passed on one side only., that is
if the closest point is an end there is no side limitation, if it is not an end,
then the path should be assumed to be non-valid. */
class UReacRoadLine
{
  public:
  /**
    Set road obstacle */
    void setLine(ULineSegment * firstSeg, UPolygon * lineHist)
    {
      lineSeg = firstSeg;
      polyLine = lineHist;
    }

  public:
  /**
    Poly-line obstacle */
    UPolygon * polyLine;
  /**
    Newest Line segment */
    ULineSegment * lineSeg;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Class with all relevant road lines */
class UReacRoadLines
{
  public:
  /**
    Constructor */
    UReacRoadLines();
  /**
    Clear the road lines */
    void clear();
  /**
    Add left line */
    void addLeftLine(ULineSegment * firstSeg, UPolygon * lineHist);
  /**
    Add left line */
    void addRightLine(ULineSegment * firstSeg, UPolygon * lineHist);

  public:
  /**
    Number of available segments */
    static const int MAX_ROAD_SEGS = 3;
  /**
    Left roadside obstacle */
    UReacRoadLine left[MAX_ROAD_SEGS];
  /**
    Count of used left lines */
    int leftCnt;
  /**
    Right roadside obstacle */
    UReacRoadLine right[MAX_ROAD_SEGS];
  /**
    Count of used right lines */
    int rightCnt;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Class for the relevant obstacles to avoid by the reactive behaviour */
class UReacObstGrps
{
  public:
  /**
   * Constructor */
    UReacObstGrps();
  /**
    Clear all obstacles */
    void clear();
  /**
    Add an obstacle group */
    void addObstGrp(UObstacleGroup * obstGrp);
  /**
    Get obstacle group */
    UObstacleGroup * getGroup(int idx);
  /**
    Get number of available groups */
    inline int getGroupsCnt()
    { return obstsCnt; };

  protected:
  /**
    Max number of obstacle groups to be stored */
    static const int MAX_OBST_GRPS = 5;
  /**
    Obstacle groups */
    UObstacleGroup * obsts[MAX_OBST_GRPS];
  /**
    Available number of obstacle groups */
    int obstsCnt;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


class UAvoidPathPool;

/**
Class to hold a set of obstacle pointers and an associated value.
The value could be the side by witch the obstacle has to be passed.*/
class UObstInfo
{
public:
  /**
  Constructor */
  UObstInfo();
  /**
  Destructor */
  ~UObstInfo();
  /**
  Remove all info */
  inline void clear()
  { obstCnt = 0; };
  /**
  Get obstacle info for this obstacle.
  Returns -1 if not found, otherwise returns stored info. */
  int getObstInfo(UObstacle * ob);
  /**
  Set obstacle info for this obstacle.
  Info should not be '-1', as this
  signals 'no info'. */
  bool setObstInfo(UObstacle * ob, int value);
  /**
  Get count of obstacle list */
  inline int getCnt()
  { return obstCnt; };
  /**
  Get obst.*/
  inline UObstacle * getObst(int index)
  { return obst[maxi(0, mini(index, obstCnt - 1))]; };
  /**
  Print status of obstacle info */
  void print(const char * prestring);
  /**
  Get max list size */
  inline int getMaxSize()
  { return listMax; };

protected:
  static const int LIST_SIZE = 100;
  /**
  Obstacle pointer */
  UObstacle * obst[LIST_SIZE];
  /**
  Number of pointers stored */
  int obstCnt;
  /**
  Value associated with pointer */
  int info[LIST_SIZE];
  /**
  Size of list */
  int listMax;
};


///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////


/**
A Laser path is a series of overlapping traversable segments
also called a corrodor.
The corridor is an array of pointers to UPassIntervals.
The first interval is the far from the robot. */
class UReactivePath
{
  public:
  /**
  Constructor */
  UReactivePath();
  /**
  Destructor */
  ~UReactivePath();
  /**
  Clear path */
  void clear();
  /**
  Make a copy of the source path into this path structure.
  Route statistics are not copied. and used flag is set to false.
  Road values and passable intervals are copied fully.
  Manoeuvre sequence is copied fully. */
  void copy(UReactivePath * source);
  /**
  Print some status for the path to this buffer */
  void print(const char * prestring, char * buff, const int buffCnt);
  /**
    Find route - using circular distance from objects.
  'edge' can have one of the following values:
    WPF_ROUTE_GO_RIGHT, WPF_ROUTE_GO_LEFT
    WPF_ROUTE_GO_TOP or WPF_ROUTE_GO_POSITION.
    'Destination' position is only relevant if edge == WPF_ROUTE_GO_POSITION.
    Dist from edge (distFromEdge) is desired distance for edge at exit
    of path (relevant for left and right edge only)
    Returns true if a route is found.
    The route is returned in the route array (in odometry map coordinates). */
/*    bool findRoute2(int edge, UPose destination,
                    UPoseV pose,
                    const double laserTilt,
                    const double distFromEdge,
                    double maxVel,
                    double maxAcc, double maxTurnAcc,
                    bool ignoreObstacles);*/
  /**
  Find route - using circular distance from objects.
  'edge' can have one of the following values:
  WPF_ROUTE_GO_RIGHT, WPF_ROUTE_GO_LEFT
  WPF_ROUTE_GO_TOP or WPF_ROUTE_GO_POSITION.
  'Destination' position is only relevant if edge == WPF_ROUTE_GO_POSITION.
  Dist from edge (distFromEdge) is desired distance for edge at exit
  of path (relevant for left and right edge only)
  Returns true if a route is found.
   * \param directTestOnly test direct path to destination only return true is possible
   * \param useDriveon replaces the pose destination with a line destination - as the drive and driveon commands.
   * \param turnRad is the turn radius to use when using drive-on command.
   * \param gD is the distance (from line) gain. approximated turn radius is PI/2 * gA / gD.
  The route is returned in the route array (in odometry map coordinates). */
  bool findObstAvoidRoute(UReacObstGrps * obsts,
                                         UReacRoadLines * roads,
                                         UAvoidPathPool * paths,
                                         bool ignoreObstacles,
                                         bool directTestOnly,
                         bool driveon, double turnRad);
/*      bool findObstAvoidRoute(int edge,
                          UPose destination, // if go-direct to pos only
                          UPoseV pose, // current robot pose
                          UObstacleHist * obsts,
                          const int obstsCnt,
                          UAvoidPathPool * pathPool,
                          const double laserTilt,
                          const double distFromEdge,
                          double maxVel,
                          double maxAcc, double maxTurnAcc,
                          bool ignoreObstacles);*/
  /**
  Test if the route ends in a wall.
  Test movement of passable interval relative to last scan.
  if less movement than a movementLimit, then wall counter is increased.
  Returns wall count. */
  /**
  Is a local route avoiding obstacle available.
  Returns true if a route point is available */
  inline bool isRouteFound()
  { return man->isValid() and valid; };
  /**
  Set path invalid and remove all existing manoeuvres; */
  inline void setRouteInvalid()
  {
    man->releaseAllMan();
    setValid(false);
  };
  /**
  Get number op points (poses) in route */
  inline int getRouteCnt()
  { return man->getP2PCnt(); };
  /**
  Is this path actually used for robot control */
  inline void setPathUsed(bool value)
  { pathUsed = value; };
  /**
  Is this path actually used for robot control */
  inline bool getPathUsed()
  { return pathUsed; };
  /**
  Get route SD relative to direct path */
  inline double getRouteSD()
  { return routeSD; };
  /**
  Get route distance or length from robot to exit point */
  inline double getRouteDist()
  { return routeDist; };
  /**
  Get route angle to first route point relative to present robot heading (in radians
  positive is CCV, zero is forward) */
  inline double getRouteAngleFirst()
  { return routeAngleFirst; };
  /**
  Get route angle to scan exit point relative to present robot heading (in radians
  positive is CCV, zero is forward) */
  inline double getRouteAngleExit()
  { return routeAngleExit; };
  /**
  Get direct distance to exit posture.
  Is calculated by setRouteStats(). */
  inline double getRouteDirectToExitDist()
  { return distExitDirect; };
  /**
  Calculate performance statistics for a route line.
  deviation (SD) from a direct line from robot to route
  endpoint, and angle offset relative to current
  robot position. */
  void setRouteStats(UPose odoPose);
  /**
  Get manoeuver sequence for planned drive path */
  inline UManSeq * getManSeq()
  { return man; };
  /**
  Get manoeuver sequence drive time */
  inline double getManTime()
  { return man->getTime(); };
  /**
  Is route valid */
  inline bool isValid()
  { return valid; };
  /**
  Is the route maintained for crash analysis */
  inline bool isACrash()
  { return crash; };
  /**
  Set route valid flag */
  inline void setValid(bool value)
  { valid = value; }
  /**
  set exit pose */
  inline void setExitPose(UPoseV pose)
  { exitPose = pose; };
  /**
  Get count of crash test mid-poses */
  inline int getMidPosesCnt()
  { return midPosesCnt; };
  /**
  Get a pointer to one of the crash test mid-poses */
  inline UPose getMidPose(int i)
  { return midPoses[i]; };

protected:
  /**
  Find in this path the best linefit of the road
  for this 'side' of the road.
  Side may be 0=left, 1=Right, 2=Center of road.
  Returns true if line segment is found, with optional variance
  returned at var pointer (if var != NULL). */

  /**
  Get a mid waypoint from this obstacle position.
  If tangent lines cross (before touching the circle),
  then a tangent point in the direction to point on the
  path closest to the obstacle, the 'offendedPoint', is used.*/
  UPoseV getTangentMidPose(int minIdx1, int minIdx2,
                            int side, UPosition minDistPos,
                            bool newEndPoint,
                            double atDistance);
  /**
  Expand this manoeuvre as needed to avoid the obstacles in the 'obsts' list,
  using the first 'obstsCnt' of the obstackle groups only.
  The obstacles are to be passed in the easiest way, unless the obstycle is
  noted in the 'obstSideList', where a specific side may be listed.
   * \param useDriveon replaces the pose destination with a line destination - as the drive and driveon commands.
   * \param turnRad is the turn radius to use.
   * \Returns true if at least one path is generated. More paths may be spawned if
  'obstSideSpawn' is true (and there is space in 'pathPool' for more paths. */
  bool avoidObst(
            UReacObstGrps * obsts,
            UAvoidPathPool * pathPool,
            UObstInfo obstSideList, bool obstSideSpawn,
            double maxVel,
            double maxAcc, double maxTurnAcc, double minTurnRad,
            int * spawnCnt, int spawnNestLevel, bool directTestOnly,
                bool useDriveon, double turnRad);
  /**
  Get position and heading for the best passage points between these
  two obstacles. Iterate the result from the midpoint between
  'minPos1' and 'minPos2'. Returns result in 'midPose'.
  'fromPose' must be set to a point before the new midPoint, to get
  the heading in the right main direction.
  The position is based on distance to the two involved obstacles, and
  heading is set in order to pass at a right angle to the line
  between the obstacles.
  Passage width is returned in 'width' (if not NULL).
  Returns true if possible, i.e. at least 'ROBOTWIDTH' of safe passage. */
  bool getMidPoseHere(UObstacle * obst1, UObstacle * obst2,
                      UPosition minPos1, UPosition minPos2, int side,
                      UPose * midPose, double * width,
                      UPose fromPose);
  /**
  Find the most distant vertex in this obstacle, starting at 'minVertex'
  the 'minVertex' segment is hit at the 'end' = 0=mid, 1=start i.e. the vertex,
  2= other end i.e. vertex+1. Return the distance (most likely negatve) in '*dist'
  and the new position in '*minDistPos'. The manoeuvre used for distance-testing
  is in 'mpp'. */
  bool getWorstVertex(UObstacle * obs, int minVertex, int end,
                                  int side, double * minDist,
                                  UPosition * minDistPos, UManPPSeq * mpp);
  /**
  Find the most distant vertex in this obstacle, signed relative tom
  manoeuvre part in 'mpp'. Side is the side that is counted positive 0=left, 1=right.
  Return the distance (most likely negatve) in '*dist'
  and the new position in '*minDistPos'.
  Returns true. */
  bool getWorstVertex2(UObstacle * obs,
                      int side, double * minDist,
                      UPosition * minDistPos, UManPPSeq * mpp);
  /**
  Get closest vertex to the position in 'minDistPos', then distance must be
  less than 'dist'. The closest vertex has number 'vertex' on exit.
  Returns NULL, if no obstacle if found near enough.
  Do not test obstacles in the 'exclude' list and not 'notThis' either. */
  UObstacle * getNearObstacle(UReacObstGrps * obsts,
                              int minVertexCnt,
                              double dist, UPosition pos,
                              int * vertex,
                              UObstInfo * exclude,
                              UObstacle * notThis);
  /**
  Test for near obstacle and move exit point away from
  obstacle (closer to other end), if possible.
  If not possible or no obstacle, then 'posExit' is unchanged.
  'n' is an iteration count, as the function may iterate a solution (max = 3).
  Returns 0 if position is OK as is.
  Returns 1 is position is changed OK.
  Returns 2 if no valid position is found. */
  int testForObstacleNearExit(UReacObstGrps * obsts,
                             const int usedEdge,
                             UPosition * posExit,
                             ULineSegment * seg,
                             double obstDist,
                             int n);


private:
  /**
  Series of passable intervals */
  //UPassInterval * intervals[MAX_LASER_SCANS];
  /**
  Number of scans in path */
  int intervalsCnt;
  /**
    Route points to travel along this path.
    The first point in route is */
  //UPosition route[MAX_ROUTE_POINTS];
  /**
    Count of points in route */
  //int routeCnt;
  /**
  New manoeuvre sequence */
  UManSeq * man;
  /**
  Were this ths path that actualy were used
  by the way-point-controller */
  bool pathUsed;
  /**
  Line deviation relative to a line from the robot
  to the route endpoint */
  double routeSD;
  /**
  Route orientation relative to the robot.
  I.e. angle to the line from the robot to
  the route endpoint (approx 2.5 meter ahead). */
  double routeAngleExit;
  /**
  Direct distance from robot to exit posture.
  Is used to evaluate best route distance */
  double distExitDirect;
  /**
  Route orientation relative to the robot.
  I.e. angle to the line from the robot to
  the first waypoint on route. */
  double routeAngleFirst;
  /**
  Route total distance from robot to exit-point */
  double routeDist;
  /**
  Is route valid - if not, should be deleted */
  bool valid;
  /**
  Is route not valid, but maintained for crash analysis */
  bool crash;
  /**
  Flag for desired exit point is in an obstacle.
  This may be the case in especially DIRECT type movement,
  and may require special treatmnet (tell calculator). */
  bool endsInObst;
  /**
  Position where route enters obstacle */
  UPosition endsInObstHere;
  /**
  Desired exit pose and speed */
  UPoseV exitPose;
  /**
  Size of crash test array with tested mid-points. */
  static const int MAX_MID_POSES = 50;
  /**
  Points used as mid-poses - deleted or not.
  Used for crash analysis only. */
  UPose midPoses[MAX_MID_POSES];
  /**
  Count of used mid-poses */
  int midPosesCnt;
};

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

/**
 * UAvoidPathPool holds an array of path alternatives and the functionality to
   select one of these as the best choice.
*/
class UAvoidPathPool : public ULock
{
public:
  /**
  Constructor */
  UAvoidPathPool();
  /**
  Destructor */
  ~UAvoidPathPool();
  /**
  Get the start pose for all paths */
  inline UPose getStartPose()
  { return par.startPose; };
  /**
  Set the start pose for all paths */
  inline void setStartPose(UPose pose, double velocity)
  {
    par.startPose.set(pose, velocity);
  };
  /**
  Set the start pose for all paths */
  inline void setAvoidSerial(int serial)
  { par.avoidSerial = serial; };
  /**
  Set safety distance limits */
  inline void setDistanceLimits(double minDist, double minMinDist)
  {
    par.obstMinDist = minDist;
    par.obstMinMinDist = minMinDist;
  };
  /**
  Get number of calculated paths */
  inline int getPathsCnt()
  { return pathsCnt; };
  /**
  Get number of calculated (rev2) paths */
  inline int getAvoidPathsCnt()
  { return avoidPathsCnt; };
  /**
  Get number valid paths. Invalid paths do not count. */
  int getValidPathsCnt();
  /**
  Get number valid paths of type UAvoidPath (rev2 path). Invalid paths do not count. */
  int getValidAvoidPathsCnt();
  /**
  Get specific path. (NB! no range check) */
  inline UReactivePath * getPath(int index)
  { return &paths[index]; };
  /**
  Get specific (rev2) path. (NB! no range check) */
  inline UAvoidPath2 * getAvoidPath(int index)
  { return &avoidPaths[index]; };
  /**
  Find path to this destination, avoiding obstacles and road edges
   * \param directTestOnly test only
   * \param driveon when true, replaces the pose destination with a line destination - as the drive and driveon commands.
   * \param turnRad is the drive-on turn radius
   * \returns pointer to OK path or NULL if none */
  UReactivePath * findPathToHere(UPoseV exitPose,
                                 UReacObstGrps * obsts,
                                 UReacRoadLines * roads,
                                 bool ignoreObstacles,
                                 bool directTestOnly,
                                 bool driveon, double turnRad);
  /**
  Find path to this destination, avoiding obstacles (and road edges)
  using a new method more directly based on visibility graph (simplified)
  \param debugDump is a flag, that will generate some data-dump to logfile (if open). */
  UAvoidPath2 * findPathToHere2(UPoseV exitPose,
                                UReacObstGrps * obsts,
                                UReacRoadLines * roads,
                                bool ignoreObstacles,
                                bool debugDump);
  /**
  Get start pose and velocity */
  UPoseV getStartPoseVel();
  /**
  Set other limiting parameters for the path evaluation.
  Units are in SI units (meter and sec) */
/*  void setParameters(double maxAcc,
                     double maxTurnAcc,
                     double minTurnRad,
                     bool crashTest,
                     int nestedLevels,
                     int maxAvoidLoops,
                     int maxSpawnCnt
                     );*/
  /**
  Get max lateral acceleration */
  inline double getMaxAcc()
  { return par.maxAcceleration; };
  /**
  Get max turn acceleration */
  inline double getMaxTurnAcc()
  { return par.maxTurnAcceleration; };
  /**
  Get minimum turn radius - fail manoeuvre if less is needed */
  inline double getMinTurnRad()
  { return par.minTurnRadius; };
  /**
  Get pointer to an unused path */
  UReactivePath * getNewPath();
  /**
  Get minimum distance to obstacle */
  inline double getObstMinDist()
  { return par.obstMinDist; };
  /**
  Get minimum distance to obstacle */
  inline double getObstMinMinDist()
  { return par.obstMinMinDist; };
  /**
  Is crashed paths to survive for analysis */
  inline bool getCrashTest()
  { return par.doCrashTest; };
  /**
  Get max nested levels when searching a path left and right around obstacles */
  inline int getMaxNestedLevels()
  { return par.maxNestedLevels; };
  /**
  Get max obstacle avoidance loops when finding new mid-poses */
  inline int getMaxAvoidLoops()
  { return par.maxAvoidLoops; };
  /**
  Get max spawnned paths when searching for an obstacle avoidance path */
  inline int getMaxSpawnCnt()
  { return par.maxSpawnCnt; };
  /**
  Get pointer to new unused UAvoidPathPool entry.
  A path may be used by e.g. implementation process. This
  function will then try the next available unlocked path entry.
  The path will be unlocked once implemented or discarded for other reasons.
  \returns an unlocked path - if anymore is available, else NULL. */
  UAvoidPath2 * getNewAvoidPath();
  /**
  Set most limiting front-left position */
  inline void setFrontLeft(UPosition value)
  {  par.frontLeft = value; };
  /**
  Set most limiting front-right position */
  inline void setFrontRight(UPosition value)
  {  par.frontRight = value; };
  /**
  Get pointer to obstacle avoidance parameters - to assign new values */
  inline UAvoidParams * getParamStruct()
  { return &par; };

protected:
  /**
  Number of alternative paths in path pool */
  static const int MAX_PATHS_IN_POOL = 20;
  /**
  Pool of alternative paths */
  UReactivePath paths[MAX_PATHS_IN_POOL];
  /**
  Number of used paths */
  int pathsCnt;
  /**
  Best path */
  int pathBest;
  /**
  New avoid path method */
  UAvoidPath2 avoidPaths[MAX_PATHS_IN_POOL];
  /**
  Number of used paths */
  int avoidPathsCnt;
public:
  /**
  Parameters for obstacle avoidance */
  UAvoidParams par;
};


#endif

