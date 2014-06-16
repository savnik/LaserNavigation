/** *************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UAVOIDPATH2_H
#define UAVOIDPATH2_H

#include <ugen4/u3d.h>
#include <urob4/uobstacle.h>
#include <umap4/umanseq.h>
#include <urob4/ulogfile.h>

#include "uavoidobst.h"
#include "uavoidlink.h"
#include "uavoidpoint.h"
#include "uavoidparams.h"
#include "uavoidcellgraph.h"

class UAvoidPathPool;
class UReacObstGrps;
class UReacRoadLines;


//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


class UAvoidMidPose
{
public:
  /**
  Constructor */
  UAvoidMidPose()
  { clear(); };
  /**
  Destructor */
  ~UAvoidMidPose();
  /**
  Clear mid-pose values */
  inline void clear()
  {
    angle = 0.0;
    obstDist = 0.0;
    nob = NULL;
  }

public:
  /**
  Mid pose to be passed */
  UPose pose;
  /**
  Angle from avoid point to this mid-pose */
  double angle;
  /**
  Path distance to offending obstacle (should be at least safety distance) */
  double obstDist;
  /**
  nearest obstacle (on manoeuvre just before or after) */
  UAvoidObst * nob;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


/**
One of a number of paths with a route from current pose to a specifix exit pose

	@author Christian <chrand@mail.dk>
*/
class UAvoidPath2
{
public:
  /**
  Constructor */
  UAvoidPath2();
  /**
  Destructor */
  ~UAvoidPath2();
  /**
  Clear path to a startup state */
  void clear();
  /**
  Is this path actually used for robot control */
  inline void setPathUsed(bool value)
  { pathUsed = value; };
  /**
  Is this path actually used for robot control */
  inline bool getPathUsed()
  { return pathUsed; };
  /**
  Is the route maintained for crash analysis */
  inline bool isACrash()
  { return crash; };
  /**
  Is route valid */
  inline bool isValid()
  { return valid; };
  /**
  Set route valid flag */
  inline void setValid(bool value)
  { valid = value; }
  /**
  Get manoeuver sequence for planned drive path */
  inline UManSeq * getManSeq()
  {
    if (manFull == NULL)
      manFull = new UManSeq();
    return manFull;
  };
  /**
  Set the start pose and velocity */
  inline void setPar(UAvoidParams * parameters)
  { par = parameters; };
  /**
  Find route to the already loaded exit point, obstacles and road-lines.
  If there is a need to spawn alternative routes then use the provided pointer to the path-pool
  to fetch more empty paths.
  \param debugDump will trigger some data dump to logfile (if open). */
  void findObstAvoidRoute(bool debugDump);
  /**
  Get the time to exit pose for this path */
  double getManTime();
  /**
  Get the obstacle groups with index idx */
  inline UAvoidObst * getAvoidObstGrp(int idx)
  { return aogs[idx]; };
  /**
  Get number of obstacle groups */
  inline int getAvoidObstGrpCnt()
  { return aogsCnt; };
  /**
  Get serial number for the obstacle calculation for this path.
  All paths generated in one calculateion has the same serial number. */
  inline int getSerial()
  { return avoidSerial; };
  /**
  Get number of available tangent sequences */
  inline int getLnkSeqsCnt()
  { return lnkSeqsCnt; };
  /**
  Get tangent sequence with this index number.
  NB! no range check. */
  inline UAvoidLnkSeq * getLnkSeq(int idx)
  { return lnkSeqs[idx]; };
  /**
  Get list of points to be avoided */
  inline UAvoidPoint * getPointList()
  { return pointListFinal; };
protected:
  /**
  Get an empty obstacle group entry for one obstcle. Either from the empty
  list or a newly created (and added to the empty list) */
  UAvoidObst * getEmptyAog();
  /**
  Get an empty obstacle link entry with 4 oblscle connection lines.
  Either from the empty list or a newly created (and added to the empty list) */
  UAvoidLink * getEmptyALnk(int serial);
  /**
  Group obstacles based on this minimal passable margin between any two obstacles. */
  void groupObstacles(double margin);
  /**
  Check for vertex positions that are equal in x-value. This may lead to
  cell generation problems.
  If a problem is found, then 1mm is aded to the x-position, and
  the polygon is further tested and corrected if not convex.
  \param obst is the obstacle polygon to test. */
  void removeEqualXvertices(UObstacle * obst);
  /**
  Create 4 tangent lines between all pairs of obstacles, and set verteces and edges
  that is not usable for visibility tangent lines when creating a path. They
  are not usable if they at any point are closer than margin to another obstacle
  in the same obstale group. */
  void createTangentLinks(double margin);
  /**
  Test tangent lines for visibility for all obstacles in all groups. */
  void testVisibility();
  /**
  Add two final obstacle groups for the start and end positions.
  NB! there must be space for these 2 extra obstacles in the aogs array. */
  void addStartAndExit();
  /**
  Test if start or exit pose is embedded in an obstacle, and
  if so invalidate these obstacles */
  void invalidateObstaclesAtStartAndExit();
  /**
  Test the tangent lines for this obstacle against the non-visibility lines
  defined for all obstacle groups. (n is a debug parameter)*/
  void testNoVisSegVisibility(UAvoidObst * aogb, int * n, int * m);
  /**
  Test this line against all non-visibility lines for all groups.
  Returns true if the line crosses a non-visibility line. */
  bool testNonVisibilityLineCross(ULineSegment * visLine);
  /**
  Count the number of valid tangent lines defined for all obstacles for all obstacle groups,
  including start and end point. */
  int countValidVisibilityLines();
  /**
  Count valid visibility lines in this series of link lines.
  Each link to other obstacles can have up to 4 links - two cross ond two outher.
  \param startLnk first link in the series.
  \returns count of valid lines - up to 4 for each link. */
  int countValidVisibilityLines(UAvoidLink * startLnk);
  
  /**
  Find routes in the set of obstacle link lines available from current position to target position.
  The target position is the last of the obstacle groups and the current position the one just before that.
  The path is searched in a depth first manner. */
  void findRoutes();
  /**
  Find routes (best route) using an A* method. */
  void findRoutesA();
  /**
  Get a new (empty) link sequence entry from the reuse pool. */
  UAvoidLnkSeq * getEmptyLnkSeq();
  /**
  Add this sequence of link sequence to the recycle list */
  void recycleLnkSeq(UAvoidLnkSeq * ls);
  /**
  Add this link sequence to the list of best tangent line sequences */
  void addToLnkSeqs(UAvoidLnkSeq * ls);
  /**
  Search the tangent lines from lsEnd to find an ordered tangent line sequence
  to advance from current position to the destination. No loops may occur with the already used tangents in the tangent line sequence from lsFirst.
  'n' is the number of tangent lines already in the list.
  The function tests all tangent lines for one obstacle and is called recursively
  until the destination "obstacle" is found.
  Returns true if the destination is found and false if no destination is found (a blind end) */
  void findRouteToDest(UAvoidLnkSeq * lsFirst, UAvoidLnkSeq * lsEnd, int n);
  /**
  Test the 'lsTest' tangent line. it may not end in an object already in the
  tangent line sequence if it would reuse any of the polygon
  index points already reserved.
  If edgeIndex1 and edgeIndex2 are different, then
  Alle the edge points from edgeIndex1 increasing to edgeIndex2
  (both inclusive) are tested*/
  bool findRouteInALoop(UAvoidLnkSeq * lsFirst,
                                     UAvoidObst * obstTest,
                                     int edgeIndex1,
                                     int edgeIndex2);
  /**
  Copy a sequence of tangent lines with root in 'source' in newly allocated
  UAvoidLnkSeq objects */
  UAvoidLnkSeq * copyAvoidLnkSeq(UAvoidLnkSeq * source);
  /**
  Invalidate the vertices between two obstacles in the same group */
  void invalidateConcavities(UAvoidLink * la, UAvoidLink  * lb);
  /**
  Remove tangents between group members that start in a voncavity
  vertex */
  void removeTangentInConcavities();
  /**
  Write obstacle avoidance status to logfile */
  void logObstacleGroups();
  /**
  debug log of tangent sequuences in each of the found paths. */
  void logPathSequences();
  /**
  Test if a tangent has an exit tangent to connect to without
  passing an edge that is too close to another obstacle (in the group) */
  void removeTangentIfNoExitTangent();
  /**
  Return this avoidance point to the recycle list.
  Recycles this point only. */
  void recyclePoint(UAvoidPoint * ap);
  /**
  Return this avoidance point and all connected points in the next-chain
  to the recycle list. */
  void recyclePoints(UAvoidPoint * first);
  /**
  Get an empti avoidance point from the recycle list.
  If recycle list is empty, then one is created on heap. */
  UAvoidPoint * getEmptyPoint();
  /**
  Returns a point list from link sequence.
  The list holds all endpoints of all links, including start and endpoint, and
  all polygon points between tangent lines.
  This is the basis for path planning. */
  UAvoidPoint * createPointList(UAvoidLnkSeq * source);
  /**
  Create sequence of points to avoid,
  The points are the sequence of vertex points along the visibility path.
  The link sequence is tanget lines only, so point list
  is supplemented with points along polygon obstacles.
  \param source is the sequence of tangent lines (links) between obstacles.
  \returns the point list in aPointList
  \Returns a pointer to the created list of points to be avoided. */
  UAvoidPoint * createAvoidPoinsPath(UAvoidLnkSeq * source);
  /**
  Convert a sequence of points from current position to destination
  passing a number 0 to N to a manoeuver sequence.
  \param ptLstBase is the list of points (UAvoidPoint) - first is current pose, last is destination
  \param ommitBase is the point ptLstBase surplus, then this flag is set true, and function returns false.
  \param movedBase is set true if mid-path point for the base avoid-point is moved.
  This may require the caller to redoo some previous calculations.
  \returns true if route is valid. false if no solution, or if ap2 is to be ommitted. */
  bool createMidPathPoints(UAvoidPoint * ptLstBase,
                           bool * ommitBase, bool * movedBase);
  /**
  Convert a sequence of points from current position to destination
  passing a number 0 to N to a manoeuver sequence.
  \param ptLst is the list of points (UAvoidPnt) - first is current pose, last is destination
  \param manSeq is the manoeuvre sequence for the result
  \param minTurnRadius is the allowed minimum turn radius for manoeuvre planning (not driveon)
  \param costAdd is list of path points to be punished in next search if manoeuvre is not possible.
  \returns true if route is valid. */
  bool convertToManSeq(UAvoidPoint * ptLst, UManSeq * manSeq, double minTurnRadius, UAvoidVertexCosts * costAdd);
  /**
  Set the turn centre to avoid the obstacle, the turn centre
  is planned so that the closest point in the manoeuvre is near the
  obstacle position, and any facing obstacle is avoided too.
  \param ap1 is the obstacle point, that needs new mid-pose.
  \param turnCentreRadius is the radius of turn arc circle,
  \param availableOuther is set to the available space for turning before hitting an opposing obstacle (including margin) - this may be used to plan an extra mid-point
  \returns false if too narrow for any turn radius, i.e. an extra mid-point is needed  */
  bool setTurnCentre(UAvoidPoint * ap1, double turnCentreRadius, double * availableOuther);
  /**
  Set new mid-pose for this obstacle point. That is a destination pose from previous mid-point.
  Assumes mCent is set correctly.
  \param ap1 is the obstacle point, that needs new mid-pose.
  \param manRad turn radius at the ap1 point.
  \param reversedAp2mCent set to true if reversed turn centre on previous point.
  \returns false if new midpoint can not set - i.e. manoeuvre is impossible (with current rules). */
  bool setMidPoint(UAvoidPoint * ap1, double manRad, bool * reversedAp2mCent = NULL);
  /**
  Get tangent point on c1 circle for the cross tangent
  between these two circles. where the tangent from c2 to c1 touches
  the c1 circle in a clockwise direction (if cv is true).
  \param c1 is the centre of the main circle.
  \param c2 is the centre of the 'from' circle
  \param cv is the tangent direction relative to circle 1
  \param manRad is the circle radius (both circles)
  \param tanpt is a pointer to a pose where the tangent point should
  be returned.
  \returns true if a cross tangent is possible (circles are not touching) */
  bool crossTangent(UPose c1, UPose c2,
                    bool cv, double manRad,
                    UPose * tanpt);
  /**
  Get tangent point on c1. where the outher tangent from c2 to c1 touches
  the c1 circle in a clockwise direction (if cv is true).
  \param c1 is the centre of the main circle.
  \param c2 is the centre of the 'from' circle
  \param cv is the tangent direction relative to circle 1
  \param manRad is the circle radius (both circles)
  \param tanpt is a pointer to a pose where the tangent point should
  be returned.
  \returns true if a tangent is possible (c1 and c2 are not equal) */
  bool outherTangent(UPose c1, UPose c2, bool cv, double manRad, UPose * tanpt);
  /**
  Get tangent point on c2. where the tangent
  in a clockwise direction (if cv is true) hits dest.
  \param dest is the destination point.
  \param c2 is the centre of the 'from' circle
  \param cv is the tangent direction from circle 2
  \param c2Rad is the circle radius
  \param tanpt is a pointer to a pose where the tangent point should
  be returned.
  \returns true if a destination is outside the circle (else tanpt is not changed) */
  bool pointTangent(UPose dest, UPose c2,
                    bool cv, double c2rad,
                    UPose * tanpt);
  /**
    get most offending point for this manoeuvre, i.e.
    will the manoeuvering hit anything on the way.
    \param manSeq is the manoeuvre sequence to test
    \param visLine is the visibility line we are trying to follow
    \param tight if true, then the tight values are used - rather than the desired clearence
    \param pos  is the offending position
    \param dist is the distance from path
    \param oObst found offending obstacle
    \param avoidLeft is set to avoid route of found obstacle
    \param firstIsCurrentPose if true, then obstacles close to robot (current pose) is ignored as false detection or a self detection.
    \param nextIsDestination if true, then this is to destination, and should be tested regardless how small.
    \returns true if a point is found.
    \returns the point position in pos, the obstacle (index into aogs array)
             and the distance from the returned point to the path. */
/*  bool getClosestObst(UManPPSeq * manSeq,
                      U2Dseg * visLine,
                      bool tight,
                      UPosition * oPos, double * dist,
                      UAvoidObst ** oObst,
                      int * mHit,
                      bool * avoidLeft,
                      bool firstIsCurrentPose,
                      bool nextIsDestination);*/
  /**
    get most offending point for this manoeuvre, i.e.
    will the manoeuvering hit anything on the way.
    \param manSeq is the manoeuvre sequence to test
    \param visLine is the visibility line we are trying to follow
    \param tight if true, then the tight values are used - rather than the desired clearence
    \param pos  is the offending position
    \param dist is the distance from path
    \param oObst found offending obstacle
    \param avoidLeft is set to avoid route of found obstacle
    \param firstIsCurrentPose if true, then obstacles close to robot (current pose) is ignored as false detection or a self detection.
    \param nextIsDestination if true, then this is to destination, and should be tested regardless how small.
    \returns true if a point is found.
    \returns the point position in pos, the obstacle (index into aogs array)
             and the distance from the returned point to the path. */
  bool getClosestObst2(UManPPSeq * manSeq,
                      U2Dseg * visLine,
                      bool tight,
                      UPosition * oPos, double * dist,
                      UAvoidObst ** oObst,
                      int * mHit,
                      bool * avoidLeft,
                      bool firstIsCurrentPose,
                      bool nextIsDestination);
  /**
  Mark vertices that is embedded inside other obstacles in the group as a nogo edge. As no
  tangent should be created from this vertex. */
  void invalidateEmbeddedVertices();
                      

private:
  /**
  Test if this angle is within some start and end angles, when turning either left or right.
  \param a is the angle under test (radians, within +/- PI)
  \param cvLeft if true, then h2 is more clockwise than h1 - else the other way
  \param h1 is the start angle
  \param h2 is the end angle
  \param angMarg is an (optional) extra margin to be added to the h1...h2 range.
  \returns true, if a is within the limits. */
  bool withinAngleLimits(double a, bool cvLeft, double h1, double h2, double angMarg);
  /**
  Expand this set of visibility lines to amanoeuvre sequence.
  Uses class variable aPointList as an interim list of waypoints.
  \param ls is the visiblity tangent line sequence
  \returns manoeuvre sequence in class variable 'man' */
  bool expandVisLinesToManSeq(UAvoidLnkSeq * ls);
  /**
  Set the closest neighboring obstacle point and the distance to it.
  \param ap is the point on the path - an obstacle point - to be avoided
  The result is set into this class.
  \param dMin is the relevant distance to look for obstacles.
  Current obstacle is not evaluated.
  */
  void setClosestPoint(UAvoidPoint * ap, double dMin);
  /**
  Convert this manoeuvre to a polygon, where one side is the
  original to-from line.
  \param manSeq the pose-to-pose sequence
  \param p40 the destination sequence.
  \param endsMargin is a part near start and end that is to be ommitted.
  \returns nothing */
  void getManAsPolygon(UManPPSeq * manSeq, UPolygon * p40, double endsMargin);
  /**
  Expand the pre-point away from the tight entrance, so that
  the manoeuvre leading to the pre-point is possible, i.e.
  the turn circles do not touch.
  Assumes both points have same avoidance direction.
  \param c1 is the current position of the turn centre that has to move
  \param c2 is the turn centre that needs to be at a manoeuverable distance 
  \param turnRad is the used turn radius for the manoeuvres.
  \param angle is direction of straight safe path towards next narrow entrance. May
  be more alligned with opening - this should be safe too (makes it more omega-like).
  \param mCent is the new position of the turn centre.
  \returns true if possible. */
  bool expandOmegaTurn(UPose c1, UPose c2, double turnRad, double angle, UPose * mCent);
  /**
  Solve a triangle using cosine relations with vertex A,B,C and oppsite sides a,b,c.
  Find a in: c^2 = a^2 + b^2 - 2 a b cos(C);
  \param b side length next to vertex C.
  \param c side length opposite vertex C.
  \param C angle at vertex C
  \param minor the smaller solution is returned here (negative if C is > 90deg or c > b
  \param returns length of side a (longest solution (2 solutions if C is < 90deg and c < b)) (positive).
  \param returns -1.0 if determinant is negative */
  double triangleFinda(double b, double c, double C, double * minor = NULL);
  
  /**
  Are the turn centre of these two near path points crossing. This also returns true,
  if turn centres are closer than a minimum margin to be crossing.
  \param ap1 is the most forward point on the route
  \param ap2 is the previous point on the route
  \param margin an additional margin from crossing
  \returns true if turncentres cross. */
  bool turnCentreCrossing(UAvoidPoint * arg1, UAvoidPoint * arg2, double margin);
  /**
  Get a new polygon from the man-space polygon heap.
  \returns a pointer to a valid UPolygon40 - the next free.
  If no more space, the polygon heap is recycled (and a debug message issued) */
  UPolygon * getFoodprintPoly();
  /**
  Takes the footprint polygons and expand the inclusive polygon to the direct line
  between the end-points.
  The exclude polygon is removed if embedded between the inclusive polygon and the segment.
  \param seg is the 2D line segment from start to end of manoeuvre.
  \param polyInc is the manoeuvere footprint (convex)
  \param polyExcl is the potential concavity area, thet shpuld be cut from the polyInc.
  \param manIsLeft is true if the manoeuvre is to the  left of the direct line - determines which part of the
  exclude polygon to delete.
  \param turningLeft is the turn coveret left or right. */
  void expandPolyToSegment(U2Dseg * seg,
                           UPolygon * polyInc,
                           UPolygon * polyExcl,
                           bool manIsLeft,
                           bool turningLeft);
  /**
  Extend the manoeuvre from avoid point ap2 to avoid point  ap1.
  \param manSeq is the manoeuvre sequence to extend.
  \param ap2 is the structure with the start pose.
  \param ap1 is the structure with the end pose.
  \returns the adeed pose to pose manoeuvre sequence. */
  UManPPSeq * extendManoeuvre(UManSeq * manSeq, UAvoidPoint * ap2, UAvoidPoint * ap1);
  /**
  Validate this obstacle relative to the prevoiusly found obstacles - especially the
  points found in ap1 and ap2.
  \param oPos is the found obstacle position.
  \param dist is the distance to the manoeuvre envalope - positive is outside the envalope and away from the visibility line between ap2 to ap1.
  \param ap2 is the previous point and is the next point, and the obstacle point is found in the manoeuvre between these points.
  \param ap1 is the end point of the manoeuvre.
  \returns true if the found point is valid (to be avoided). */
  bool validNewClosePoint(UPosition oPos, double dist,
                                       UAvoidPoint * ap2, UAvoidPoint *  ap1);
  /**
  Is this obstacle position after the desination pose in ap1.mid.
  \param oPos is the obstacle position.
  \param ap1 is the destination point
  \returns true if the point is after the destination. */
  bool isAfterDestination(UPosition oPos, UAvoidPoint * ap1);
  /**
  Is this obstacle point to be avoided to the left.
  \param oPose is the obstacle point position.
  \param oObs is the obstacle on which the obstacle position belongs.
  \param postObst is it after the destination pose.
  \param ap1 the destination point.
  \returns true if the oPos point is to be avoided to the left */
  bool avoidToTheLeft(UPosition oPos, UAvoidObst * oObs, bool postObst, UAvoidPoint * ap1);
  /**
  Set new point as a pre-point and set it before the ap1-end point to stop before the obstacle. */
  bool setPreStopPoint(bool cvLeft, UPosition oPos, UAvoidPoint * apNew, UAvoidPoint * ap1);
  /**
  Test if this new point potentially could replace the old obstacle avoidance point
  \param apNew the new just added point.
  \param apOld the point that may be replaced.
  \param excludedList is a list of points previously excluded. If the new point is already in this
  exclude list then it can not be replaced.
  \return true if the apOld point can be replaced (potentially) by the new. */
  bool toReplaceOldPoint(UAvoidPoint * apNew, UAvoidPoint * apOld, UAvoidPoint * excludedList);
  /**
  Test if this point is inside the incl polygon for the manHist manoeuvre or the previous manoeuvre,
  but not inside the corresponding exclusive polygon.
  It is assumed that the envalope polygons are created and available in the polys[] array.
  \param pos is the position to be sested.
  \param polyMan0 is the polygon number for manoeuvre number 0 (first manoeuvre polygon to be examined)
  \param manHit is the (last) manoeuvre number to be tested
  \param limit is the required distance inside the incl polygon to be declared 'found'.
  \returns the distance inside the incl polygon if the position is at least limit inside the incl polygon, excluding the excl polygon.
  \returns limit if not inside. */
  double isInsidePoly(UPosition pos, int polyMan0, int * manHit, double limit);
  /**
  Test if this (obstacle) polygon crosses one of the manoeuvre polygons, and that the most inside point is a valid point,
  i.e. not inside the corresponding exclusive polygon.
  It is assumed that the envalope polygons are created and available in the polys[] array.
  \param obst is the polygon, that may cross one of the manoeuvre polygons.
  \param pos is the returned position inside the hit manoeuvre polygon (if crossing)
  \param dist is the distance to the polygon from 'pos' - is negative inside.
  \param polyMan0 is the polygon number for manoeuvre number 0
  \param manHit is the first manoeuvre crossing the obstacle (if crossing)
  \param limit is the required distance inside the incl polygon to be declared 'found'.
  \returns true if valid crossing is found. */
  bool isCrossingManPoly(UPolygon * obst, UPosition * pos, double * dist, int polyMan0, int * manHit, double limit);
  /**
  Add all links leving this obstacle and is compatible with the previous (arriving link)
  \param aog is the current obstacle arrived to by 'prev'
  \param prev is the link by whish we arrived to the current obstacle. */
  void addLnksToOpenSet(UAvoidObst * aog, UAvoidLnkSeq * prev);
  /**
  Add this link sequence entry to the open set of potential solutions.
  Is added to openSet in sorted order with the lowest F-cost (cost so far + cost to destination) first.
  \param ls is the new possible solution candidate - with front position (probably not destination) */
  void addToOpenSet(UAvoidLnkSeq * ls);
  /**
  Add this edge to the closed set of nodes */
  void addToClosedSet(UAvoidLnkSeq * ls);
  /**
  Is this link line's end point already in the closed set
  \param candidate is the line to be tested.
  \param prev is the previous line.
  \param edgeDist returns if not found the cost from last tangent to candidate tangent along edge of this obstacle.
  \returns true if a passed vertex either on this obstacle, or arrived to on the next obstacle is in the closed set */
  bool isInClosedSet(UAvoidLnkSeq * candidate, UAvoidLnkSeq * prev, double * edgeDist);
  /**
  Look for unfinished sets in the open-set list, that ends at the same vertex as this or ends on a
  vertex on candidate's home obstacle that is on the path from the previous tangent.
  When a unfinished entry is found in the open set it is deleted, if it has a worse score than the current.
  \param candidate is the current candidate exit link.
  \param prev is the previous link, when arrived to candidates obstacle.
  \returns true if an unfinished entry has a better score than the candidate (then the candidate should be closed).*/
  bool terminateWorseCandidatesInOpenSet(UAvoidLnkSeq * candidate, UAvoidLnkSeq * prev);
  /**
  Insert this new (unlinked) point just before the ap1 point.
  The new point is validated against the previous point.
  The path mid-points are reevaluated, and if need be an omega manoeuvre expansion is attempted.
  \param ap1 is the current point.
  \param apNew is the new point to be inserted.
  \returns true if successfully inserted and validated. */
  bool insertNewPointAfter(UAvoidPoint * ap1, UAvoidPoint * apNew);
  /**
  Split an obstacle group - used to allow exit and entry of cavities in an obstracle group, e.g. enter a closed room or a dead end.
  Otherwise closest point could not be at the other side of the opening.
  Splits the group left and right of a splitting line from the provided position (x,y) and the COG of the obstacles in the group
  furthest away from this point.
  NB! leaves the non-visibility lines between obstacles in the splittet group, as these still are valid.
  \param grpIde is the group to split.
  \param x,y,h is the splitting line. */
  void splitObstGroup(int grpIdx, double x, double y, double h);
  /**
  Convert no-visibility lines to obstacles, and add the new 2-point obstacles to the obstacle groups.
  \param aogs is an array of existing obstacle groups.
  \param aogsCnt is number of established groups */
  void addNoVisLinesAsObstacles(UAvoidObst * aogs[], const int aogsCnt);

  /**
  Convert cell points to an avoidance point list.
  cvs is the cell vertex points found in the cell graph.
  cvsCnt is the number of cells.
  The start and end points are not included. */
  UAvoidPoint * createCellBasedPointList(UAvoidCellVertex * cvs, const int cvsCnt);
  /**
  Convert the vertex list produced by the cell based obstacle avoidance path finder
  to the avoid-point list needed by the manoeuvre expansion function.
  And convert to a man-sequence.
  \param cvs is the cell based vertex list elements - only those with obstacle reference are valid.
  \param cvsCnt is the total number of elements in the cvs array.
  \returns true if a valid manoeuvre sequence is formed. */
  bool expandCellPntsToManSeq(UAvoidCellVertex * cvs, const int cvsCnt);

  
public:
  /**
  Index of this path in the path pool (used for logging putposes)  */
  int poolIdx;
  const static int MAX_FOODPRINT_POLYS = 200;
  /**
  Debug polygons for dynamic free path polygons */
  UPolygon * polys[MAX_FOODPRINT_POLYS];
  /**
  Number of used polygons */
  int polysCnt;
  /**
    number of footprint polygons before reset */
  int oldFootCnt;
  /**
  Celle based avoidance path structure */
  UAvoidCellGraph * avCellGraph;
  /**
  Max number of UAvoidObstacleGrouups (AOG) groups */
  static const int MAX_AOG_OBST_GRPS = 60;
  /**
  Obstackle groups baset on separation (passable separation) */
  UAvoidObst * aogs[MAX_AOG_OBST_GRPS];
  /**
  Number of used obstacle groups */
  int aogsCnt;
  

protected:
  /**
  Is path used, i.e. this is determined to be the best aviailable path */
  bool pathUsed;
  /**
  Is path valid, that is there is a calculated route, and it may be evaluated for
  best path - if crash flag is not set. */
  bool valid;
  /**
  Is this path failed, i.e. no path found to exit pose i.e. ended in a crash */
  bool crash;
  /**
  Is this path save for crash analysis. if so the valid flag is still true, but
  should not be included when searching for best path */
  UAvoidParams * par;
  /**
  The manoeuvre sequence found in this path */
  UManSeq * manFull;
  /**
  Flag for desired exit point is in an obstacle.
  This may be the case in especially DIRECT type movement,
  and may require special treatmnet (tell calculator). */
  bool endsInObst;
  /**
  Position where route enters obstacle */
  UPosition endsInObstHere;
  /**
  Max number of local obstacles - to replace no-visiblity lines */
  static const int MAX_LOCAL_OBSTS = 200;
  /**
  Obstackle groups baset on separation (passable separation) */
  UObstacle * aogsObst[MAX_LOCAL_OBSTS];
  /**
  Number of used obstacle groups */
  int aogsObstCnt;
  /**
  Obstackle groups baset on separation (passable separation) */
  UAvoidObst * aogRoot;
  /**
  Obstackle groups free list - last used object, the free pointer is not used. */
  UAvoidObst * aogLast;
  /**
  Obstackle groups baset on separation (passable separation) */
  UAvoidLink * aLnkRoot;
  /**
  Obstackle groups free list - last used object, the free pointer is not used. */
  UAvoidLink * aLnkLast;
  /**
  Fake obstacles for start and exit position */
  UObstacle * startExitObst[2];
  /**
  Serial number for this obstacle avoidance path */
  int avoidSerial;
  /**
  Maximum number of saved link sequences */
  static const int MAX_LINK_SEQS = 30;
  /**
  Array of possible link sequences in prioritized order. */
  UAvoidLnkSeq * lnkSeqs[MAX_LINK_SEQS];
  /**
  Count of used sequences */
  int lnkSeqsCnt;
  /**
  Array of possible link sequences in prioritized order. */
  UAvoidLnkSeq * openSet;
  /**
  Array of possible link sequences in prioritized order. */
  UAvoidLnkSeq * closedSet;
  /**
  Linked list of sequence objects for reuse. This pointer is the first element in
  the reuse list, the first may be used, and the next to reuse in in the last pointer. */
  UAvoidLnkSeq * lnkSeqRoot;
  /**
  The last used link */
  UAvoidLnkSeq * lnkSeqLast;
  /// next serial number for fetch of linkSequence.
  int lnkSerial;
  /**
  Logfile for debug */
  FILE * logdbg;
  /**
  Obstackle groups baset on separation (passable separation) */
  UAvoidPoint * aPointRoot;
  /**
  The final point list of points to be avoided */
  UAvoidPoint * pointListFinal;
  /**
  Max distance cost allowed for any sequence, based on currently most promising solution */
  double maxDistCost;
  /**
  Max angle cost allowed for any sequence, based on currently most promising solution */
  double maxAngleCost;
  /**
  Count of tangents searched while finding af tangent sequence */
  int tanSeqCnt;
  /**
  Number of too close obstacles during this path. */
  int closeCnt;
  /**
  The current generation of point creation. */
  int generation;
  /// debug count of mid-point solutions
  int solutionCnt;
  /// next serial number for inserted avoid points
  int serialNext;
  /// additional path cost for path finding
  UAvoidVertexCosts costAdd;

private:
  /**
  A debug variable used to allocate serial numbers to added connection obstacles (from no-visibility lines) */
  long unsigned int maxSerial;

};

#endif
